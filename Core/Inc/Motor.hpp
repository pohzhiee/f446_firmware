#ifndef F446_SIMPLE1_MOTOR_HPP
#define F446_SIMPLE1_MOTOR_HPP
#include <array>
#include <cstdint>
#include <cmath>
#include "InterfaceData.hpp"

static constexpr double shaft_rad_to_encoder_val = 65535.0/2.0/M_PI;
static constexpr double encoder_val_to_shaft_rad = M_PI*2.0/65535.0;
static constexpr double rad_to_deg = 180/M_PI;
static constexpr double deg_to_rad = M_PI/180.0;
class Motor {
public:
    [[nodiscard]] inline int32_t GetEncoderTotal() const
    {
        return current_encoder_val_+encoder_loop_count_*65535;
    }

    [[nodiscard]] inline float GetOutputAngleRad() const
    {
        return static_cast<float>(GetEncoderTotal()*encoder_val_to_shaft_rad/gear_ratio_);
    }

    [[nodiscard]] inline double GetGearRatio() const{
        return gear_ratio_;
    };

    [[nodiscard]] std::array<uint8_t, 8> GetCommand(const MotorCommandSingle& cmd) const;

    void SetConfig(const MotorConfigSingle& config, int index, uint32_t message_id);

    void UpdateEncoderVal(uint16_t encoder_pos);

private:
    uint16_t current_encoder_val_ = 0;
    int32_t encoder_loop_count_ = 0;

    double gear_ratio_{6.0};
    bool ignore_limits_{false};
    double lower_limit_{-M_PI_2};
    int32_t lower_limit_encoder_val_{static_cast<int32_t>(-M_PI_2*6.0*shaft_rad_to_encoder_val)};
    double upper_limit_{M_PI_2};
    int32_t upper_limit_encoder_val_{static_cast<int32_t>(M_PI_2*6.0*shaft_rad_to_encoder_val)};
    double speed_rad_{720.0/6.0*M_PI/180.0}; // position mode output speed in radians/sec
    uint16_t speed_shaft_deg_{720}; // position mode shaft speed in degrees/sec
    const double base_torque_constant_{6.0};
    double torque_multiplier_{1.0}; // default value of 1.0 to represent uncalibrated

};
#endif //F446_SIMPLE1_MOTOR_HPP
