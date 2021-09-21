//
// Created by pohzhiee on 15/09/21.
//

#include "MotorPosTracker.hpp"

void MotorPosTracker::update(uint16_t encoder_pos)
{
    int32_t encoder_diff = (int32_t)encoder_pos-(int32_t)current_encoder_val_;
    if (encoder_diff<-40000) {
        loop_count++;
    }
    else if (encoder_diff>40000) {
        loop_count--;
    }
    current_encoder_val_ = encoder_pos;
}

void MotorPosTracker::set_angle(int64_t angle)
{
    int64_t encoder_total = angle*65535/36000;
    loop_count = encoder_total/65535;
    current_encoder_val_ = encoder_total%65535;
}
void MotorPosTracker::init(int64_t start_angle)
{
    if (start_angle<0)
        loop_count = -1;
    else
        loop_count = 0;
}
void MotorPosTracker::set_gear_ratio(float ratio)
{
    gear_ratio_ = ratio;
}
