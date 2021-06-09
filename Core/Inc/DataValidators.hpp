#ifndef F446_SIMPLE1_DATAVALIDATORS_HPP
#define F446_SIMPLE1_DATAVALIDATORS_HPP

#include <span>
#include <cstdint>

class MotorSPIInputValidator {
public:
    using IsrFunc = void (*)();
    explicit MotorSPIInputValidator(std::array<uint8_t, 2> headers = {0xAA, 0xBB})
            :
            headers_(headers) { };

    bool parse_data(std::span<uint8_t, 80> raw_transmission_data);
private:
    const std::array<uint8_t, 2> headers_;
};
#endif //F446_SIMPLE1_DATAVALIDATORS_HPP
