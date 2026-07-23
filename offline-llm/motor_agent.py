#!/usr/bin/env python3
"""Offline agent bridge: let the local ollama model actually talk to the board.

The plain `ollama run motor` model is text-only -- it can advise but has no way
to reach the ODrive. This wraps it in a tiny agent loop: the model is given a set
of *tools*, ollama returns tool calls, and this script executes them by shelling
out to the already-trusted `spider-motor-tools/` CAN scripts, then feeds the
result back to the model. Fully offline (talks to ollama on localhost:11434 via
stdlib urllib; no internet, no extra pip deps).

Tool protocol: ollama's *native* tool_calls field is not populated for
qwen2.5-coder:7b (it just writes the call as text), so this uses a strict
JSON-in-text contract instead -- the model replies with a single JSON object,
either {"tool": name, "arguments": {...}} or {"answer": text}, and we parse it.
Argument parsing is deliberately tolerant because small models are sloppy.

Design / safety:
  * Board I/O is NOT reimplemented here -- every tool runs an existing, tested
    spider-motor-tools script as a subprocess. One place talks to the bus.
  * READ-ONLY by default. `can_probe` (heartbeat/vbus/encoder/Iq/temp) is always
    allowed. Any motion tool (`can_goto`) is refused unless you pass --allow-move,
    and even then every single move prints the exact command and waits for a y/n
    confirmation (honors the project rule: ask before any physical action).
  * The model NEVER touches hardware directly -- it can only request a whitelisted
    tool; this script decides whether to run it.

Usage:
    # read-only Q&A that can inspect the live bus:
    .venv/bin/python offline-llm/motor_agent.py "are both knee joints healthy?"

    # allow guarded moves (still confirms each one):
    .venv/bin/python offline-llm/motor_agent.py --allow-move \
        "move leg1 knee to 30 degrees"

    # interactive chat:
    .venv/bin/python offline-llm/motor_agent.py
"""
import argparse
import json
import os
import subprocess
import sys
import urllib.request

OLLAMA_URL = "http://localhost:11434/api/chat"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = os.path.join(REPO, ".venv", "bin", "python")
TOOLS_DIR = os.path.join(REPO, "spider-motor-tools")

# Known joint CAN nodes (kept in sync with the Modelfile / CAN_NODE_ID_MAP.md).
DEFAULT_NODES = "13:motor#8 leg1-knee,23:motor#3 leg2-knee"

SYSTEM = """You are an offline motor-bench assistant with REAL tools that reach a
live ODrive robot over a CAN bridge. You are not just answering from memory --
when a question is about the actual state of the hardware, CALL A TOOL to look,
then answer from what it returned.

There are TWO ways a motor is connected, and they use DIFFERENT tools:
- Over the CAN bus (the assembled robot, addressed by node id) -> can_probe / can_goto.
- Over USB directly to one ODrive on the bench (a single motor being brought up /
  tested, NO node id) -> usb_health. If the user says "usb", "on the bench",
  "connected directly", names a motor by number with no node, or a CAN probe
  reports the node is not connected, use usb_health -- do NOT keep probing CAN.

You have these tools:
- can_probe(nodes): read-only health probe of joint CAN node(s). Returns
  heartbeat (axis error + state), vbus, joint encoder angle, motor velocity, Iq,
  temperatures. Safe any time. `nodes` is optional; omit it to probe both known
  knee joints (node 13 = leg1-knee, node 23 = leg2-knee).
- can_goto(node, target_degrees, rate_deg_s): gently move ONE joint to a target
  angle. This PHYSICALLY MOVES the leg. Only use when the user clearly asked to
  move a joint. `node` and `target_degrees` are required; `rate_deg_s` optional.
- usb_health(motor_id, serial_number, gearbox): run the non-destructive USB health
  battery on a single bench ODrive over USB (motor cal repeatability, encoder
  offset scatter, free-spin current sweep). Nothing is written to flash. It DOES
  spin the motor shaft, so the operator confirms before it runs. Args (all
  optional): `motor_id` (e.g. 11) labels the report; omit `serial_number` to use
  the only connected board. Set `gearbox: true` WHENEVER a gearbox is attached to
  the output -- the motor-side offset-cal repeatability check is contaminated by
  reflected gearbox drag/backlash and false-FAILs, so with a gearbox on you MUST
  pass gearbox:true (it skips that invalid sub-check and keeps motor-cal + the
  free-spin sweep). Free-spin current is HIGHER with a gearbox (e.g. ~2 A vs
  ~0.5 A bare for a 1:6) -- that is normal drag, not a fault. Use this for "test
  health" / "check this motor" over USB.

HOW TO REPLY -- output a SINGLE JSON object and NOTHING else, no prose, no code
fences:
  * To use a tool:  {"tool": "can_probe", "arguments": {"nodes": "13:leg1-knee"}}
  * To use a tool with no args:  {"tool": "can_probe", "arguments": {}}
  * When you are done and have the answer for the user:
        {"answer": "both knee joints are healthy: node 13 at 12 deg, node 23 ..."}
After each tool call you will receive a TOOL RESULT message; read it, then either
call another tool or give the final {"answer": ...}.

Board facts: single-axis MKS ODrive-S per joint; joint angle is OUTPUT turns
(1 turn = 360 deg) as reported over CAN. Knee joints: 0 turn = upper endstop,
leg-down is positive, safe window ~5-85 deg. Legs #8 (node 13) and #3 (node 23)
mechanically collide near leg-down -- keep moves inside the safe window.
Give angles in degrees. If a tool reports axis_error != 0, the joint is FAULTED;
say so and do not move it until cleared."""


def run_script(argv):
    """Run a spider-motor-tools script and return combined stdout/stderr text."""
    try:
        p = subprocess.run(
            [PY] + argv, cwd=TOOLS_DIR, capture_output=True, text=True, timeout=120
        )
        out = (p.stdout or "") + (p.stderr or "")
        return out.strip() or "(no output)"
    except subprocess.TimeoutExpired:
        return "ERROR: tool timed out after 120s"
    except Exception as e:  # noqa: BLE001
        return "ERROR running tool: %s" % e


def _coerce_nodes(val):
    """Small models pass nodes as '13:leg', '13,23', ['13','23'], [13], or junk.
    Extract any integers; fall back to the known knee joints if none found."""
    import re
    if val is None:
        return DEFAULT_NODES
    if isinstance(val, (int, float)):
        text = str(int(val))
    elif isinstance(val, (list, tuple)):
        text = ",".join(str(x) for x in val)
    else:
        text = str(val)
    # If it already looks like a proper node:label list, keep it verbatim.
    if ":" in text and any(c.isdigit() for c in text):
        return text
    ids = re.findall(r"\d+", text)
    if not ids:
        return DEFAULT_NODES
    labels = {"13": "leg1-knee", "23": "leg2-knee"}
    return ",".join("%s:%s" % (i, labels.get(i, "node%s" % i)) for i in ids)


def tool_can_probe(args, allow_move):
    nodes = _coerce_nodes(args.get("nodes"))
    return run_script(["can_check.py", "--nodes", nodes])


def _confirm(prompt):
    """Ask the human y/N before a physical action. Returns True to proceed."""
    print("\n  >>> " + prompt)
    try:
        return input("      proceed? [y/N] ").strip().lower() == "y"
    except EOFError:
        return False


def _truthy(v):
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in ("1", "true", "yes", "y", "on")


def tool_usb_health(args, allow_move):
    cmd = ["motor_health_check.py"]
    mid = args.get("motor_id") or args.get("motor") or args.get("id")
    if mid is not None:
        import re
        m = re.findall(r"\d+", str(mid))
        if m:
            cmd += ["--motor-id", m[0]]
    sn = args.get("serial_number") or args.get("serial")
    if sn:
        cmd += ["--serial-number", str(sn)]
    # A gearbox on the output contaminates the motor-side offset-cal repeatability
    # (reflected drag + backlash + cogging through a ~0.5-turn slow sweep), so its
    # "spread high -> FAIL" is a false alarm. Skip that sub-check with a gearbox
    # attached; motor-cal (electrical, at standstill) and the free-spin current
    # sweep (the real gearbox-drag check) stay valid.
    if _truthy(args.get("gearbox")) or _truthy(args.get("skip_offset")):
        cmd += ["--skip-offset"]
    if _truthy(args.get("skip_motorcal")):
        cmd += ["--skip-motorcal"]
    if _truthy(args.get("skip_spin")):
        cmd += ["--skip-spin"]
    pretty = "%s %s" % (PY, " ".join(cmd))
    if not allow_move:
        return ("REFUSED: the USB health battery spins the motor shaft, so motion "
                "must be enabled. Restart the agent with --allow-move. Proposed "
                "command was: " + pretty)
    if not _confirm("MODEL WANTS TO RUN THE USB HEALTH BATTERY (spins the shaft):\n"
                    "      command: " + pretty):
        return "Operator DECLINED the health check. Nothing was run."
    return run_script(cmd)


def tool_can_goto(args, allow_move):
    node = args.get("node")
    deg = args.get("target_degrees")
    rate = args.get("rate_deg_s", 10.0)
    if node is None or deg is None:
        return "ERROR: can_goto needs node and target_degrees"
    target_turns = float(deg) / 360.0
    cmd = ["can_goto.py", "--node", str(int(node)),
           "--target", "%.5f" % target_turns, "--rate", str(rate)]
    pretty = "%s %s" % (PY, " ".join(cmd))
    if not allow_move:
        return ("REFUSED: motion is disabled. The operator must restart the agent "
                "with --allow-move to permit joint moves. Proposed command was: "
                + pretty)
    # Ask the human before any physical action.
    print("\n  >>> MODEL WANTS TO MOVE A JOINT:")
    print("      node %s -> %.1f deg (%.5f turn) at %s deg/s" %
          (node, float(deg), target_turns, rate))
    print("      command: %s" % pretty)
    try:
        ans = input("      run this move? [y/N] ").strip().lower()
    except EOFError:
        ans = "n"
    if ans != "y":
        return "Operator DECLINED the move. Nothing was moved."
    return run_script(cmd)


DISPATCH = {"can_probe": tool_can_probe, "can_goto": tool_can_goto,
            "usb_health": tool_usb_health}


def chat(messages, model):
    payload = {"model": model, "messages": messages, "stream": False,
               "format": "json", "options": {"temperature": 0.1}}
    req = urllib.request.Request(
        OLLAMA_URL, data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read())


def _extract_json(text):
    """Pull the first balanced {...} object out of the model's reply."""
    text = (text or "").strip()
    try:
        return json.loads(text)
    except Exception:  # noqa: BLE001
        pass
    start = text.find("{")
    if start < 0:
        return None
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                try:
                    return json.loads(text[start:i + 1])
                except Exception:  # noqa: BLE001
                    return None
    return None


def answer(question, model, allow_move, max_steps=6):
    messages = [{"role": "system", "content": SYSTEM},
                {"role": "user", "content": question}]
    for _ in range(max_steps):
        try:
            resp = chat(messages, model)
        except Exception as e:  # noqa: BLE001
            return "ERROR talking to ollama (%s). Is `ollama serve` running?" % e
        content = resp.get("message", {}).get("content", "")
        messages.append({"role": "assistant", "content": content})
        obj = _extract_json(content)
        if obj is None:
            return content.strip() or "(no answer)"
        if "answer" in obj and "tool" not in obj:
            ans = obj["answer"]
            return ans if isinstance(ans, str) else json.dumps(ans)
        name = obj.get("tool")
        if not name:
            return content.strip() or "(no answer)"
        raw_args = obj.get("arguments") or obj.get("args") or {}
        if isinstance(raw_args, str):
            raw_args = _extract_json(raw_args) or {}
        print("  [tool] %s(%s)" % (name, json.dumps(raw_args)), flush=True)
        handler = DISPATCH.get(name)
        result = handler(raw_args, allow_move) if handler \
            else ("ERROR: unknown tool %r. Valid tools: can_probe, can_goto, "
                  "usb_health." % name)
        messages.append({"role": "user",
                         "content": "TOOL RESULT (%s):\n%s" % (name, result)})
    return "(stopped: too many tool steps -- possible loop)"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("question", nargs="*", help="one-shot question; omit for chat")
    ap.add_argument("--model", default="motor", help="ollama model (default: motor)")
    ap.add_argument("--allow-move", action="store_true",
                    help="permit guarded joint moves (still confirms each one)")
    args = ap.parse_args()

    mode = "MOVE-ENABLED (confirms each)" if args.allow_move else "READ-ONLY"
    if args.question:
        print(answer(" ".join(args.question), args.model, args.allow_move))
        return
    print("motor agent [%s] model=%s -- ask about the live board. Ctrl-C to quit."
          % (mode, args.model))
    while True:
        try:
            q = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not q:
            continue
        if q in ("exit", "quit"):
            return
        print(answer(q, args.model, args.allow_move))


if __name__ == "__main__":
    main()
