#ifndef F446_SIMPLE1_LOGGER_HPP
#define F446_SIMPLE1_LOGGER_HPP
#include <optional>
#include <atomic>

class CharBuffer {
public:
    static constexpr unsigned BufLen = 72;
    static constexpr unsigned BufSize = 128;
    void InsertCharBuf(std::span<char> data)
    {
        std::memcpy(buffer_[next_buf_index_].data(), data.data(), data.size());
        buffer_sizes_[next_buf_index_] = data.size();
        next_buf_index_++;
        if (next_buf_index_>=BufLen)
            next_buf_index_ = 0;
        total_inserted_++;
    }

    std::optional<std::span<char>> GetNextUnprocessedBuf()
    {
        if (total_processed_>=total_inserted_)
            return std::nullopt;

        std::span<char> buf(buffer_[next_process_index_].data(), buffer_sizes_[next_process_index_]);
        next_process_index_++;
        total_processed_++;
        if (next_process_index_>=BufLen)
            next_process_index_ = 0;
        return buf;
    }
private:
    uint8_t next_buf_index_{0};
    uint8_t next_process_index_{0};
    std::atomic<uint32_t> total_inserted_{0};
    std::atomic<uint32_t> total_processed_{0};
    std::array<std::array<char, BufSize>, BufLen> buffer_{};
    std::array<int, BufLen> buffer_sizes_{};
};

class ILog {
public:
    virtual int Log(char*) = 0;
};

class Logger {
public:
    static const unsigned BufLen = 72;
    static const unsigned BufferSize = 160;

    using InsertCharBufFunc = void (*)(std::span<char>);
    template<typename T>
    void AddLog(T data)
    {
        static_assert(std::is_base_of_v<ILog, T>);
        auto current_buf_ptr = buffer_[next_log_index].data();
        new(current_buf_ptr) T(std::move(data));
        next_log_index++;
        total_inserted_++;
        if (next_log_index>=BufLen)
            next_log_index = 0;
    }

    void ProcessLog(InsertCharBufFunc func)
    {
//        assert(total_inserted_ - total_processed_ < BufLen);

        while (total_inserted_>total_processed_) {
            char a[CharBuffer::BufSize];
            int size = reinterpret_cast<ILog*>(buffer_[next_process_index].data())->Log(a);
            func(std::span(a, size));
            total_processed_++;
            next_process_index++;
            if (next_process_index>=BufLen)
                next_process_index = 0;
        }
    };
private:

private:
    std::atomic<uint8_t> next_log_index{0};
    std::atomic<uint8_t> next_process_index{0};
    std::atomic<uint32_t> total_inserted_{0};
    std::atomic<uint32_t> total_processed_{0};
    std::array<std::array<std::byte, BufferSize>, BufLen> buffer_{};
};

class CANLogger: ILog {
public:
    CANLogger(std::array<uint8_t, 8> command, uint8_t index, double raw_data, // NOLINT(cppcoreguidelines-pro-type-member-init)
            uint32_t message_id) // NOLINT(cppcoreguidelines-pro-type-member-init)
            :command_(command), index_(index), raw_data_(raw_data), message_id_(message_id) { };
    int Log(char* buf) override
    {
        if (command_[0]==0xa4 || command_[0]==0xa2) { // NOLINT(bugprone-branch-clone)
            return sprintf(buf, "[%lu]Cmd %d: %x, val: %ld\n", message_id_, index_, command_[0],
                    *reinterpret_cast<int32_t*>(&command_[4]));
        }
        else if (command_[0]==0xa1) {
            return sprintf(buf, "[%lu]Cmd %d: %x, val: %d, raw: %f\n", message_id_, index_, command_[0],
                    *reinterpret_cast<int16_t*>(&command_[4]),
                    raw_data_);
        }
        else if (command_[0]==0x9c) {
            return sprintf(buf, "[%lu]Cmd %d: %x\n", message_id_, index_, command_[0]);
        }
        else {
            return sprintf(buf, "[%lu]Unsupported CAN command logged: %x, index: %d\n", message_id_, command_[0], index_);
        }
    };
private:
    std::array<uint8_t, 8> command_;
    uint8_t index_;
    double raw_data_;
    uint32_t message_id_;
};

template<unsigned N, typename T>
class SimpleNumLogger: ILog {
public:
    explicit SimpleNumLogger(const char* format_text, std::array<T, N> data)
            :data_(data)
    {
        std::strcpy(format_text_.data(), format_text);
    };
    int Log(char* buf) override
    {
        if constexpr(N==1) { // NOLINT(bugprone-branch-clone)
            return sprintf(buf, format_text_.data(), data_[0]);
        }
        else if constexpr(N==2) {
            return sprintf(buf, format_text_.data(), data_[0], data_[1]);
        }
        else if constexpr(N==3) {
            return sprintf(buf, format_text_.data(), data_[0], data_[1], data_[2]);
        }
        else if constexpr(N==4) {
            return sprintf(buf, format_text_.data(), data_[0], data_[1], data_[2], data_[3]);
        }
        else if constexpr(N==5) {
            return sprintf(buf, format_text_.data(), data_[0], data_[1], data_[2], data_[3], data_[4]);
        }
        else if constexpr(N==6) {
            return sprintf(buf, format_text_.data(), data_[0], data_[1], data_[2], data_[3], data_[4], data_[5]);
        }
        return sprintf(buf, format_text_.data(), data_.data());
    }
private:
    std::array<T, N> data_{};
    std::array<char, 100> format_text_{};
};

class TextLogger: ILog {
public:
    explicit TextLogger(const char* text)
    {
        std::strcpy(text_.data(), text);
    };
    int Log(char* buf) override
    {
        return sprintf(buf, "%s", text_.data());
    }
private:
    std::array<char, 128> text_{};
};

extern CharBuffer LogCharBuffer;
extern Logger main_logger;

#endif //F446_SIMPLE1_LOGGER_HPP
