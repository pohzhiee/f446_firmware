#ifndef F446_SIMPLE1_INTERFACEDATA_HPP
#define F446_SIMPLE1_INTERFACEDATA_HPP

#include "Common.hpp"

enum class MotorCommandMode: uint16_t {
    Ignore = 0,
    Position = 0b1,
    Velocity = 0b11,
    Torque = 0b111,
    Read = 0b1111,
    SetZero = 0b10101
};

struct MotorCommandSingle {
    MotorCommandMode CommandMode;
    std::array<uint8_t, 8> Param; // if we change this to a double the padding would be different leading to bigger than expected size
    uint32_t Reserved;
};
static_assert(sizeof(MotorCommandSingle)==16);

enum class MotorCommandType : uint16_t{
    NormalCommand = 0xAA | (0x11 << 8),
    ConfigCommand = 0xAB | (0x33 << 8)
};

struct MotorCommandMsg {
    MotorCommandType CommandType;
//    uint16_t Reserved{0};
    uint32_t MessageId;
    std::array<MotorCommandSingle, 6> MotorCommands;
    uint32_t CRC32;
};
static_assert(sizeof(MotorCommandMsg)==108);

enum class MotorConfigType: uint16_t {
    Nothing = 0,
    GearRatio = 0b110, // Can be negative value to indicate flip in direction
    IgnoreLimits = 0b1100,
    LowerJointLimit = 0b11100,
    UpperJointLimit = 0b111100,
    PositionModeSpeed = 0b1111100,
    SetTorqueMultiplier = 0b11111100
};

enum class ConsiderLimits : uint64_t{
    Ignore = 0b1010101,
    Consider = 0b1111111 // technically we don't need this, we just need to check for ignore
};

struct MotorConfigSingle {
    MotorConfigType ConfigType;
    uint16_t Reserved;
    std::array<uint8_t,12> Data;
};
static_assert(sizeof(MotorCommandSingle)==16);

struct MotorConfigMsg{
    MotorCommandType CommandType;
    uint16_t Reserved;
    uint32_t MessageId;
    std::array<MotorConfigSingle, 6> MotorConfigs;
    uint32_t CRC32;

    bool CheckCRC()
    {
        uint32_t CRCCalc = GetCRC32(reinterpret_cast<uint32_t*>(this), sizeof(MotorConfigMsg)/4-1);
        return CRCCalc==this->CRC32;
    }
};
static_assert(sizeof(MotorConfigMsg)==108);


struct MotorFeedbackSingle {
    float Torque; // motor raw torque value, from -33 to 33
    float Angle; // in output rad
    float Velocity; // in output rad/s
    uint16_t Temperature; // in deg C
    uint16_t Reserved;
};
static_assert(sizeof(MotorFeedbackSingle)==16);

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
    uint8_t Header1 = 0xBB;
    uint8_t Header2 = 0x11;
    MotorFeedbackStatus status{};
    uint32_t MessageId{0};
    std::array<MotorFeedbackSingle, 6> Feedbacks{};
    uint32_t CRC32{};

    bool CheckCRC()
    {
        uint32_t CRCCalc = GetCRC32(reinterpret_cast<uint32_t*>(this), sizeof(MotorFeedbackFull)/4-1);
        return CRCCalc==this->CRC32;
    }

    void GenerateCRC(){
        uint32_t CRCCalc = GetCRC32(reinterpret_cast<uint32_t*>(this), sizeof(MotorFeedbackFull)/4-1);
        this->CRC32 = CRCCalc;
    }
};
static_assert(sizeof(MotorFeedbackFull)==108);

struct MotorFeedbackRaw {
    uint8_t Command: 8;
    uint8_t Temperature: 8;
    int16_t TorqueCurrentRaw: 16;
    int16_t Speed: 16; // in deg/s
    uint16_t EncoderPos: 16;
};
static_assert(sizeof(MotorFeedbackRaw)==8);
#endif //F446_SIMPLE1_INTERFACEDATA_HPP
