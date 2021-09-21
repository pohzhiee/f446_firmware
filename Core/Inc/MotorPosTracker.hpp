#ifndef F446_SIMPLE1_MOTORPOSTRACKER_HPP
#define F446_SIMPLE1_MOTORPOSTRACKER_HPP

#include <cstdint>

class MotorPosTracker {
public:
    void init(int64_t start_angle);
    void set_gear_ratio(float ratio);
    void update(uint16_t encoder_pos);
    /**
     * This function is to be called by using data from the 0x92 command
     * @param angle Motor shaft angle in 0.01deg/LSB
     */
    void set_angle(int64_t angle);

    [[nodiscard]] inline int32_t get_encoder_total() const
    {
        return current_encoder_val_+loop_count*65535;
    }
    [[nodiscard]] inline bool is_within_limits(int32_t lower_limit, int32_t upper_limit) const
    {
        const auto encoder_total = get_encoder_total();
        return encoder_total<upper_limit && encoder_total>lower_limit;
    }
    [[nodiscard]] inline float get_shaft_angle_rad() const
    {
        constexpr float encoder_to_deg_constant = 1/65535.0*360.0;
        constexpr float deg_to_rad_constant = 3.141592653589793238/180;
        constexpr float encoder_to_rad_constant = encoder_to_deg_constant*deg_to_rad_constant;
        return get_encoder_total()*encoder_to_rad_constant;
    }
    [[nodiscard]] inline float get_motor_angle_rad() const
    {
        return get_shaft_angle_rad()/gear_ratio_;
    }

private:
    uint16_t current_encoder_val_ = 0;
    int16_t loop_count = 0;
    float gear_ratio_ = 6.0;

};
#endif //F446_SIMPLE1_MOTORPOSTRACKER_HPP
