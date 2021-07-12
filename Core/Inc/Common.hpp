#ifndef F446_SIMPLE1_COMMON_HPP
#define F446_SIMPLE1_COMMON_HPP

#include <array>
#include <span>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include "stm32f4xx.h"

class Logger {
public:
    void log(const char* format, ...)
    {
        va_list arglist;
        va_start(arglist, format);
        int size = vsprintf(buffer_[next_log_buf_index_].data(), format, arglist);
        va_end(arglist);
        buffer_sizes_[next_log_buf_index_] = size;
        next_log_buf_index_++;
        if(next_log_buf_index_ > 15)
            next_log_buf_index_ = 0;
    }

    [[nodiscard]] uint8_t get_num_unprocessed() const{
        if(next_log_buf_index_ < next_processed_index_)
            return next_log_buf_index_ + 16 - next_processed_index_;
        else
            return next_log_buf_index_ - next_processed_index_;
    }
    std::span<char> get_next_unprocessed_buf(){
        std::span<char> buf(buffer_[next_processed_index_].data(), buffer_sizes_[next_processed_index_]);
        next_processed_index_++;
        if(next_processed_index_ > 15)
            next_processed_index_ = 0;
        return buf;
    }
    uint8_t next_log_buf_index_{0};
    uint8_t next_processed_index_{0};
    std::array<std::array<char, 150>, 16> buffer_{};
    std::array<int, 16> buffer_sizes_{};
};

extern Logger logger;


inline uint32_t CalculateCRC(const uint32_t* start_ptr, uint32_t len)
{
    SET_BIT(CRC->CR, CRC_CR_RESET);
    for (unsigned i = 0; i<len; ++i) {
        CRC->DR = start_ptr[i];
    }
    return CRC->DR;
}

struct MotorFeedbackSingle {
    float torque;
    float angle;
    int16_t velocity: 16;
    uint16_t temperature: 16;
};
static_assert(sizeof(MotorFeedbackSingle)==12);

struct MotorFeedbackFull {
    uint8_t header1 = 0xCC;
    uint8_t header2 = 0xDD;
    uint16_t status = 0;
    std::array<MotorFeedbackSingle, 6> Feedbacks{};
    uint32_t CRC32{};
};
static_assert(sizeof(MotorFeedbackFull)==80);

enum MotorCommandMode: uint32_t {
    Read = 0,
    Position = 1,
    Velocity = 2,
    Torque = 3
};
struct MotorFeedbackRaw {
    uint8_t Command: 8;
    int8_t Temperature: 8;
    int16_t TorqueCurrentRaw: 16;
    int16_t Speed: 16;
    uint16_t EncoderPos: 16;
};
static_assert(sizeof(MotorFeedbackRaw)==8);
struct MotorCommandSingle{
    MotorCommandMode Mode;
    std::array<uint8_t, 8> Param;
};
static_assert(sizeof(MotorCommandSingle) == 12);
struct MotorCommandFull{
    std::array<MotorCommandSingle, 6> Commands;
    uint32_t CRC32;
};
static_assert(sizeof(MotorCommandFull) == 76);

#endif //F446_SIMPLE1_COMMON_HPP
