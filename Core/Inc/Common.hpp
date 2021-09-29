#ifndef F446_SIMPLE1_COMMON_HPP
#define F446_SIMPLE1_COMMON_HPP

#include <array>
#include <span>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include "stm32f4xx.h"

inline uint32_t GetCRC32(const uint32_t* start_ptr, uint32_t len)
{
    SET_BIT(CRC->CR, CRC_CR_RESET);
    for (unsigned i = 0; i<len; ++i) {
        CRC->DR = start_ptr[i];
    }
    return CRC->DR;
}

#endif //F446_SIMPLE1_COMMON_HPP
