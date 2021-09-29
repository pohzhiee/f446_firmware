#include <Motor.hpp>
#include <Logger.hpp>

std::array<uint8_t, 8> Motor::GetCommand(const MotorCommandSingle& cmd) const
{
    std::array<uint8_t, 8> can_command{};
    if (!ignore_limits_) {
        auto current_encoder_total = GetEncoderTotal();
        if (current_encoder_total<lower_limit_encoder_val_ || current_encoder_total>upper_limit_encoder_val_) {
            // If no ignore limits and exceeded joint limits, we send a 0 torque command to minimise damage
            can_command[0] = 0xa1;
            for (int i = 0; i<7; i++) {
                can_command[i+1] = 0;
            }
            return can_command;
        }
    }

    double param;
    switch (cmd.CommandMode) {
    default:
    case MotorCommandMode::Read:
        can_command[0] = 0x9c;
        break;
    case MotorCommandMode::Position:
        can_command[0] = 0xa4;
        std::memcpy(&param, cmd.Param.data(), 8);
        *reinterpret_cast<uint16_t*>(&can_command[2]) = speed_shaft_deg_; // set max speed to 720deg/s (shaft, output = 120deg/s)
        // Position command takes position in units of 0.01deg/LSB, so we need to convert from rad to deg and then times 100
        *reinterpret_cast<int32_t*>(&can_command[4]) = static_cast<int32_t>(param*gear_ratio_*rad_to_deg*100);
        break;
    case MotorCommandMode::Velocity:
        can_command[0] = 0xa2;
        std::memcpy(&param, cmd.Param.data(), 8);
        // Input param is in rad/s output speed, motor expects value in 0.01deg/s (shaft speed, so need to consider gear ratio)
        *reinterpret_cast<int32_t*>(&can_command[4]) = static_cast<int32_t>(param*rad_to_deg*gear_ratio_*100);
        break;
    case MotorCommandMode::Torque:
        can_command[0] = 0xa1;
        // Input param is in Nm output torque, so we need to consider gearing
        // Output is mapped value from -2048-2048 representing torque values between -33-33
        // By default the gear ratio and torque multiplier cancel each other out so 1Nm of input torque will set send command equivalent
        // to 1.0 to the motor
        std::memcpy(&param, cmd.Param.data(), 8);
        *reinterpret_cast<int16_t*>(&can_command[4]) = static_cast<int16_t>(param*base_torque_constant_*torque_multiplier_*2048.0
                /gear_ratio_/33.0);
        break;
    case MotorCommandMode::SetZero:
        // Note that after this command is sent the motor will take some time resetting itself, during which it would not respond at all
        can_command[0] = 0x19;
        for (int i = 0; i<7; i++) {
            can_command[i+1] = 0;
        }
        break;
    }

    return can_command;
}

void Motor::SetConfig(const MotorConfigSingle& config, int index, uint32_t message_id)
{
    switch (config.ConfigType) {
    case MotorConfigType::GearRatio:
        [&]() {
            double val;
            std::memcpy(&val, config.Data.data(), 8);
            gear_ratio_ = val;
            // We update all data that has to do with gear ratio when gear ratio changes to make it such that the order of setting the
            // config becomes irrelevant. i.e. you can set gear ratio first then joint limits, or joint limits then gear ratio
            // This is also why there is duplicated data for each field
            // Also because we want to avoid gear ratio calculations during "hot" code which is during CAN command generation
            double shaft_lower_limit = lower_limit_*gear_ratio_;
            auto encoder_val = (int32_t)(shaft_lower_limit*shaft_rad_to_encoder_val);
            lower_limit_encoder_val_ = encoder_val;

            double shaft_upper_limit = upper_limit_*gear_ratio_;
            encoder_val = (int32_t)(shaft_upper_limit*shaft_rad_to_encoder_val);
            upper_limit_encoder_val_ = encoder_val;

            const double shaft_speed = speed_rad_*gear_ratio_; // shaft speed in rad/s
            speed_shaft_deg_ = (int16_t)(shaft_speed/M_PI*180);
        }(); // we wrap it in a lambda because otherwise we cannot declare a variable inside a case statement
        break;
    case MotorConfigType::IgnoreLimits:
        [&]() {
            ConsiderLimits a;
            std::memcpy(&a, config.Data.data(), 8);
            if (a==ConsiderLimits::Ignore) {
                ignore_limits_ = true;
//                    logger.log("Motor %d ignore limits\n", index);
            }
            else { // We maybe need to check for validity of the enum data, but whatever, not much difference
                if(ignore_limits_){
                    main_logger.AddLog(SimpleNumLogger<1,int>("Motor %d no longer ignore limits\n", {index}));
                }
                ignore_limits_ = false;
            }
        }();
        break;
    case MotorConfigType::LowerJointLimit:
        [&]() {
            double val;
            std::memcpy(&val, config.Data.data(), 8);
            lower_limit_ = val;
            const double shaft_lower_limit = val*gear_ratio_;
            const auto encoder_val = (int32_t)(shaft_lower_limit*shaft_rad_to_encoder_val);
            lower_limit_encoder_val_ = encoder_val;
//                logger.log("Motor %d lower limit: %f\n", index, val);
        }();
        break;
    case MotorConfigType::UpperJointLimit:
        [&]() {
            double val;
            std::memcpy(&val, config.Data.data(), 8);
            upper_limit_ = val;
            const double shaft_upper_limit = val*gear_ratio_;
            const auto encoder_val = (int32_t)(shaft_upper_limit*shaft_rad_to_encoder_val);
            upper_limit_encoder_val_ = encoder_val;
//                logger.log("Motor %d upper limit: %f\n", index, val);
        }();
        break;
    case MotorConfigType::PositionModeSpeed:
        [&]() {
            double val;
            std::memcpy(&val, config.Data.data(), 8);
            speed_rad_ = val;
            const double shaft_speed = val*gear_ratio_; // shaft speed in rad/s
            speed_shaft_deg_ = (int16_t)(shaft_speed/M_PI*180);
//                logger.log("Motor %d pos mode speed: %f\n", index, val);
        }();
        break;
    case MotorConfigType::SetTorqueMultiplier:
        [&]() {
            double val;
            std::memcpy(&val, config.Data.data(), 8);
            torque_multiplier_ = val;
//                logger.log("Motor %d torque multiplier: %f, after multiplying gear ratio: %f\n", index, val, val*gear_ratio_);
        }();
        break;
    default:
//            logger.log("Ignoring Motor %d on msg %u\n", index, message_id);
        break;
    }
}
void Motor::UpdateEncoderVal(uint16_t encoder_pos)
{
    int32_t encoder_diff = (int32_t)encoder_pos-(int32_t)current_encoder_val_;
    if (encoder_diff<-40000) {
        encoder_loop_count_++;
    }
    else if (encoder_diff>40000) {
        encoder_loop_count_--;
    }
    current_encoder_val_ = encoder_pos;
}