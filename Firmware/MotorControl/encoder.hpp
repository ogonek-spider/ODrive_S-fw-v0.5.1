#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif


class Encoder : public ODriveIntf::EncoderIntf {
public:
    static constexpr uint32_t MODE_FLAG_ABS = 0x100;

    struct Config_t {
        Mode mode = MODE_INCREMENTAL;
        bool use_index = false;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        bool zero_count_on_find_idx = true;
        int32_t cpr = (2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        int32_t offset = 0;        // Offset between encoder count and rotor electrical phase
        float offset_float = 0.0f; // Sub-count phase alignment offset
        int32_t direction = 1;     // Direction of published position/velocity estimates (1 or -1)
        int32_t zero_offset = 0;   // Raw encoder count that maps to published position zero
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 1000.0f;
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool idx_search_unidirectional = false; // Only allow index search in known direction
        bool ignore_illegal_hall_state = false; // dont error on bad states like 000 or 111
        uint16_t abs_spi_cs_gpio_pin = 1;
        uint16_t sincos_gpio_pin_sin = 3;
        uint16_t sincos_gpio_pin_cos = 4;
        // Harmonic (eccentricity) compensation for magnetic absolute encoders.
        // Subtracts a fitted 1st/2nd harmonic of the per-revolution encoder
        // error, expressed in counts, from the position estimate and the
        // electrical phase. Coefficients are measured offline (see
        // spider-motor-tools) and saved to NVM.
        bool enable_harmonic_compensation = false;
        float harmonic_cos_1 = 0.0f; // [count]
        float harmonic_sin_1 = 0.0f; // [count]
        float harmonic_cos_2 = 0.0f; // [count]
        float harmonic_sin_2 = 0.0f; // [count]

        // custom setters
        Encoder* parent = nullptr;
        void set_mode(Mode value) {
            if (mode == value && parent->mode_ == value) {
                return;
            }
            mode = value;
            parent->set_mode(value);
        }
        void set_use_index(bool value) { use_index = value; parent->set_idx_subscribe(); }
        void set_find_idx_on_lockin_only(bool value) { find_idx_on_lockin_only = value; parent->set_idx_subscribe(); }
        void set_abs_spi_cs_gpio_pin(uint16_t value) {
            if (abs_spi_cs_gpio_pin == value) {
                return;
            }
            abs_spi_cs_gpio_pin = value;
            parent->abs_spi_cs_pin_init();
        }
        void set_pre_calibrated(bool value) { pre_calibrated = value; parent->check_pre_calibrated(); }
        void set_bandwidth(float value) { bandwidth = value; parent->update_pll_gains(); }
        void set_direction(int32_t value) {
            direction = (value < 0) ? -1 : 1;
            parent->reset_user_position();
        }
        void set_zero_offset(int32_t value) {
            zero_offset = mod(value, cpr);
            parent->reset_user_position();
        }
    };

    Encoder(const EncoderHardwareConfig_t& hw_config,
            Config_t& config, const Motor::Config_t& motor_config);
    
    void setup();
    void set_error(Error error);
    void set_mode(Mode mode);
    bool do_checks();

    void enc_index_cb();
    void set_idx_subscribe(bool override_enable = false);
    void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    void set_zero();
    void reset_user_position();
    bool calib_enc_offset(float voltage_magnitude);

    bool run_index_search();
    bool run_direction_find();
    bool run_offset_calibration();
    void sample_now();
    bool update();
    void mt6701_debug_sample();

    const EncoderHardwareConfig_t& hw_config_;
    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    float phase_ = 0.0f;        // [count]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float pos_cpr_counts_ = 0.0f;  // [count]
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float user_pos_estimate_counts_ = 0.0f; // Direction- and zero-adjusted position [count]
    float last_raw_pos_estimate_counts_ = 0.0f;
    bool user_position_initialized_ = false;
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float calib_scan_response_ = 0.0f; // debug report from offset calib
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;
    uint16_t mt6701_debug_word0_ = 0;
    uint16_t mt6701_debug_word1_ = 0;
    uint32_t mt6701_debug_raw24_ = 0;
    uint16_t mt6701_debug_pos_ = 0;
    uint8_t mt6701_debug_crc_calc_ = 0;
    uint8_t mt6701_debug_crc_recv_ = 0;
    bool mt6701_debug_crc_ok_ = false;
    uint32_t mt6701_debug_sample_count_ = 0;
    uint32_t mt6701_debug_bad_crc_count_ = 0;
    uint32_t mt6701_debug_request_count_ = 0;
    uint32_t mt6701_debug_start_ok_count_ = 0;
    uint32_t mt6701_debug_start_fail_count_ = 0;
    uint32_t mt6701_debug_mode_ = 0;
    float harmonic_error_ = 0.0f; // live harmonic correction applied [count]

    float pos_estimate_ = 0.0f; // [turn]
    float vel_estimate_ = 0.0f; // [turn/s]
    float pos_cpr_ = 0.0f;      // [turn]
    float pos_circular_ = 0.0f; // [turn]

    bool pos_estimate_valid_ = false;
    bool vel_estimate_valid_ = false;

    int16_t tim_cnt_sample_ = 0; // 
    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC
    float sincos_sample_s_ = 0.0f;
    float sincos_sample_c_ = 0.0f;

    bool abs_spi_init();
    bool abs_spi_start_transaction();
    void abs_spi_cb();
    void abs_spi_cs_pin_init();
    uint16_t abs_spi_dma_tx_[2] = {0xFFFF, 0xFFFF};
    uint16_t abs_spi_dma_rx_[2];
    bool abs_spi_pos_updated_ = false;
    Mode mode_ = MODE_INCREMENTAL;
    GPIO_TypeDef* abs_spi_cs_port_;
    uint16_t abs_spi_cs_pin_;
    uint32_t abs_spi_cr1;
    uint32_t abs_spi_cr2;

    constexpr float getCoggingRatio(){
        return 1.0f / 3600.0f;
    }
};

#endif // __ENCODER_HPP
