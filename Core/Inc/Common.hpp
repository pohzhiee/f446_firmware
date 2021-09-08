#ifndef F446_SIMPLE1_COMMON_HPP
#define F446_SIMPLE1_COMMON_HPP

#include <array>
#include <span>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include "stm32f4xx.h"

template<unsigned BufLen=16>
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
        if(next_log_buf_index_ >= BufLen)
            next_log_buf_index_ = 0;
    }

    [[nodiscard]] uint8_t get_num_unprocessed() const{
        if(next_log_buf_index_ < next_processed_index_)
            return next_log_buf_index_ + BufLen - next_processed_index_;
        else
            return next_log_buf_index_ - next_processed_index_;
    }
    std::span<char> get_next_unprocessed_buf(){
        std::span<char> buf(buffer_[next_processed_index_].data(), buffer_sizes_[next_processed_index_]);
        next_processed_index_++;
        if(next_processed_index_ >= BufLen)
            next_processed_index_ = 0;
        return buf;
    }
    uint8_t next_log_buf_index_{0};
    uint8_t next_processed_index_{0};
    std::array<std::array<char, 150>, BufLen> buffer_{};
    std::array<int, BufLen> buffer_sizes_{};
};

extern Logger<16> logger;

inline uint32_t GetCRC32(const uint32_t* start_ptr, uint32_t len)
{
    SET_BIT(CRC->CR, CRC_CR_RESET);
    for (unsigned i = 0; i<len; ++i) {
        CRC->DR = start_ptr[i];
    }
    return CRC->DR;
}

#endif //F446_SIMPLE1_COMMON_HPP
