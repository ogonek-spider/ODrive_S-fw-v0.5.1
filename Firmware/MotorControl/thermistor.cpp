#include "odrive_main.h"

#include "low_level.h"

ThermistorCurrentLimiter::ThermistorCurrentLimiter(uint16_t adc_channel,
                                                   const float* const coefficients,
                                                   size_t num_coeffs,
                                                   const float& temp_limit_lower,
                                                   const float& temp_limit_upper,
                                                   const bool& enabled) :
    adc_channel_(adc_channel),
    coefficients_(coefficients),
    num_coeffs_(num_coeffs),
    temperature_(NAN),
    temp_limit_lower_(temp_limit_lower),
    temp_limit_upper_(temp_limit_upper),
    enabled_(enabled),
    error_(ERROR_NONE)
{
}

void ThermistorCurrentLimiter::update() {
    const float voltage = get_adc_voltage_channel(adc_channel_);
    const float normalized_voltage = voltage / adc_ref_voltage;
    const float raw_temp = horner_fma(normalized_voltage, coefficients_, num_coeffs_);

    // Low-pass filter the temperature. The thermistor ADC reads pick up motor
    // PWM noise on a high-impedance divider, so a single raw sample can glitch
    // by tens of degrees (seen up to +24 C). Both the over-temp trip and the
    // current-derate path consume this value every control loop, so the noise
    // would false-trip and jitter the current limit near the thermal limit.
    // The winding temperature itself only moves over seconds, so a 1 s time
    // constant rejects the noise without meaningfully lagging real heating.
    // alpha is derived from the loop period so it is independent of loop rate.
    const float tau_s = 1.0f;
    const float alpha = current_meas_period / (tau_s + current_meas_period);
    if (!(temperature_ == temperature_)) { // seed on the first sample (NaN)
        temperature_ = raw_temp;
    } else {
        temperature_ += alpha * (raw_temp - temperature_);
    }
}

bool ThermistorCurrentLimiter::do_checks() {
    if (enabled_ && temperature_ >= temp_limit_upper_ + 5) {
        error_ = ERROR_OVER_TEMP;
        axis_->error_ |= Axis::ERROR_OVER_TEMP;
        return false;
    }
    return true;
}

float ThermistorCurrentLimiter::get_current_limit(float base_current_lim) const {
    if (!enabled_) {
        return base_current_lim;
    }

    const float temp_margin = temp_limit_upper_ - temperature_;
    const float derating_range = temp_limit_upper_ - temp_limit_lower_;
    float thermal_current_lim = base_current_lim * (temp_margin / derating_range);
    if (!(thermal_current_lim >= 0.0f)) { // Funny polarity to also catch NaN
        thermal_current_lim = 0.0f;
    }

    return std::min(thermal_current_lim, base_current_lim);
}

OnboardThermistorCurrentLimiter::OnboardThermistorCurrentLimiter(const ThermistorHardwareConfig_t& hw_config, Config_t& config) :
    ThermistorCurrentLimiter(hw_config.adc_ch,
                             hw_config.coeffs,
                             hw_config.num_coeffs,
                             config.temp_limit_lower,
                             config.temp_limit_upper,
                             config.enabled),
    config_(config)
{
}

OffboardThermistorCurrentLimiter::OffboardThermistorCurrentLimiter(Config_t& config) :
    ThermistorCurrentLimiter(UINT16_MAX,
                             &config.thermistor_poly_coeffs[0],
                             num_coeffs_,
                             config.temp_limit_lower,
                             config.temp_limit_upper,
                             config.enabled),
    config_(config)
{
    decode_pin();
}

void OffboardThermistorCurrentLimiter::decode_pin() {
    const GPIO_TypeDef* const port = get_gpio_port_by_pin(config_.gpio_pin);
    const uint16_t pin = get_gpio_pin_by_pin(config_.gpio_pin);

    adc_channel_ = channel_from_gpio(port, pin);
}
