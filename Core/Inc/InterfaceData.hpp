#ifndef F446_SIMPLE1_INTERFACEDATA_HPP
#define F446_SIMPLE1_INTERFACEDATA_HPP

#include "Common.hpp"

enum MotorCommandMode: uint16_t {
    Ignore = 0,
    Position = 0b1,
    Velocity = 0b11,
    Torque = 0b111,
    Read = 0b1111
};

struct MotorCommandSingle {
    MotorCommandMode CommandMode;
    double Param;
}__attribute__((packed));
static_assert(sizeof(MotorCommandSingle)==10);


enum MotorCommandType : uint16_t{
    NormalCommand = 0xAA | (0x11 << 8),
    StopCommand = 0xAA | (0x22 << 8)
};

struct MotorCommandMsg {
    const MotorCommandType CommandType{NormalCommand};
    const uint16_t Reserved1{0};
    uint32_t MessageId{0};
    std::array<MotorCommandSingle, 6> MotorCommands{};
    const uint64_t Reserved2{0};
    uint32_t CRC32{0};

    bool CheckCRC()
    {
        uint32_t CRCCalc = GetCRC32(reinterpret_cast<uint32_t*>(this), 19);
        return CRCCalc==this->CRC32;
    }
}__attribute__((packed));
static_assert(sizeof(MotorCommandMsg)==80);


struct MotorFeedbackSingle {
    float torque;
    float angle;
    int16_t velocity: 16;
    uint16_t temperature: 16;
};
static_assert(sizeof(MotorFeedbackSingle)==12);

struct MotorFeedbackStatus {
    bool Motor1Ready: 1;
    bool Motor2Ready: 1;
    bool Motor3Ready: 1;
    bool Motor4Ready: 1;
    bool Motor5Ready: 1;
    bool Motor6Ready: 1;
    bool IsError: 1;
    bool IsIdle: 1;
    uint8_t ErrorCode: 8;
};
static_assert(sizeof(MotorFeedbackStatus)==2);

struct MotorFeedbackFull {
    uint8_t header1 = 0xBB;
    uint8_t header2 = 0x11;
    MotorFeedbackStatus status{};
    std::array<MotorFeedbackSingle, 6> Feedbacks{};
    uint32_t CRC32{};

    bool CheckCRC()
    {
        uint32_t CRCCalc = GetCRC32(reinterpret_cast<uint32_t*>(this), 19);
        return CRCCalc==this->CRC32;
    }
};
static_assert(sizeof(MotorFeedbackFull)==80);


struct MotorFeedbackRaw {
    uint8_t Command: 8;
    uint8_t Temperature: 8;
    int16_t TorqueCurrentRaw: 16;
    int16_t Speed: 16;
    uint16_t EncoderPos: 16;
};
static_assert(sizeof(MotorFeedbackRaw)==8);
#endif //F446_SIMPLE1_INTERFACEDATA_HPP
