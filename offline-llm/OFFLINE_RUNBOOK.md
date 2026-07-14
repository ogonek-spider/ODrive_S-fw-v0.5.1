# Offline LLM Runbook (forest / no internet)

Hardware: Apple M5, 16 GB unified memory. Everything below runs **fully offline**
once the one-time downloads (done on setup day) are complete.

## What needs NO LLM and no internet (already works)
- `odrivetool`, calibration, flashing over USB/CAN
- All `spider-motor-tools/` scripts
- Firmware build: `./dockerbuild.sh build` (Docker image `odrive-build-img` is cached)
- Flash: `.venv/bin/odrivetool dfu build/ODriveFirmware.elf`

The local LLM is ONLY for reasoning/coding help — a weaker offline stand-in for Claude.

## Models installed
- `qwen2.5-coder:7b` — primary. ~4.5 GB, ~5–6 GB RAM in use. Safe to run WHILE
  Docker builds / odrivetool is connected.
- `motor` — the 7B wrapped with this project's board/patch facts as a system prompt
  (built from `offline-llm/Modelfile.motor`). **Use this one for motor questions.**
- `qwen2.5-coder:14b` — smarter but ~9 GB. **Do NOT run it while Docker builds or
  odrivetool is connected** on 16 GB — it will swap. Use it for harder reasoning when
  the board is idle; drop back to 7B/`motor` for anything alongside a build.

## Daily use
```bash
# Make sure the server is up (Homebrew starts it at login):
brew services list | grep ollama          # should say "started"
# or run it manually with better memory settings:
OLLAMA_FLASH_ATTENTION=1 OLLAMA_KV_CACHE_TYPE=q8_0 ollama serve

# Ask the motor-aware model:
ollama run motor
# then type your question, e.g.:
#   write a python snippet using .venv/bin/odrivetool to sweep velocity 1..8 t/s and log iq
#   what does ENCODER_ERROR_ILLEGAL_HALL_STATE mean and how do I clear it here?

# Plain coding model (no project context):
ollama run qwen2.5-coder:7b

# One-shot from the shell (no chat):
ollama run motor "how do I safely arm axis0 into closed-loop from UNDEFINED?"
```

## Rebuild the motor model (after editing the Modelfile)
```bash
cd /Users/alarin/Documents/art/ogonek25-spider/ODrive_S-fw-v0.5.1
ollama create motor -f offline-llm/Modelfile.motor
```

## Memory tips for 16 GB
- Run 7B, not 14B, while Docker or odrivetool are active.
- `ollama ps` shows loaded models + RAM. `ollama stop <model>` frees it.
- Models unload themselves after ~5 min idle by default.

## Before you leave (checklist — needs internet, do it now)
- [x] `brew install ollama`
- [x] `ollama pull qwen2.5-coder:7b`
- [x] `ollama create motor -f offline-llm/Modelfile.motor`
- [x] `ollama pull qwen2.5-coder:14b`
- [x] Docker image `odrive-build-img` cached (`docker images`)
- [x] `.venv` deps installed: `.venv/bin/pip install -r requirements.txt` (pyusb + pyelftools verified)
- [x] Local ODrive docs — already in-repo and **version-matched** (see below)

## Local docs & references (all offline, version-matched to this firmware)
Don't download from docs.odriverobotics.com — the live site serves newer versions
that don't match this v0.5.1 fork. Use the in-repo copies instead:
- `docs/` — full v0.5.1 doc set: `can-protocol.md`, `control.md`, `encoders.md`,
  `commands.md`, `ascii-protocol.md`, `troubleshooting.md`, `getting-started.md`, etc.
- `tools/odrive/enums.py` — every error/state/mode enum (AXIS/MOTOR/ENCODER/CONTROLLER
  error bits, control modes, input modes). Authoritative for decoding error fields.
- `Firmware/odrive-interface.yaml` — complete API/attribute definition (incl. the local
  MT6701 / split-feedback / zero-offset additions).
- `AGENTS.md` / `CLAUDE.md` — this project's board + patch facts (the source the `motor`
  model's system prompt was distilled from).

## Honest limits
A 7B/14B local model is much weaker than Claude. Good for: snippets, syntax, error
lookups, boilerplate. Weak at: subtle control tuning, the MT6701 SSI patch, root-causing
USB-drop vs magnet-slip. Treat its output as a draft and verify against the real board.
