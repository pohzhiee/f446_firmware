#include "Common.hpp"
#include "DataValidators.hpp"

bool MotorSPIInputValidator::parse_data(std::span<uint8_t, 80> raw_transmission_data)
{
    if (raw_transmission_data[0]!=headers_[0]){
        logger.log("Wrong header0, actual: %x, expected: %x\n", raw_transmission_data[0], headers_[0]);
        return false;
    }
    if (raw_transmission_data[1]!=headers_[1]){
        logger.log("Wrong header1, actual: %x, expected: %x\n", raw_transmission_data[1], headers_[1]);
        return false;
    }

    std::span<uint8_t> data = raw_transmission_data.subspan(4, 72);
    std::span<uint8_t> crc = raw_transmission_data.subspan(76, 4);
    uint32_t crc_transmission = *reinterpret_cast<uint32_t*>(crc.data());
    uint32_t crc_data = CalculateCRC(reinterpret_cast<uint32_t*>(raw_transmission_data.data()), 19);
    if(crc_data == crc_transmission){
//        logger.log("CRC match\n");
        return true;
    }
    else{
        logger.log("CRC mismatch\n");
        return false;
    }
}