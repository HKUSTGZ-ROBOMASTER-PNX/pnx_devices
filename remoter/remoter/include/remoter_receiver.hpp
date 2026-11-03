#pragma once

#include "remoter.hpp"
#include "bsp_usart.hpp"
#include <cstring>

namespace remotes
{
template <uint8_t FrameSize,
    pnxtypes::status (*Decode)(const uint8_t*, uint16_t, reading&)>
class receiver
{
public:
    receiver() = default;
    receiver(const receiver&) = delete;
    receiver& operator=(const receiver&) = delete;

    pnxtypes::status start(const bsp::usart::config& port,
        const bsp::usart::line_config& line, memory::buffer<64>& dma)
    {
        using pnxtypes::status;
        auto result = bsp::usart::configure(port.instance, line);
        if (result == status::ok)
        {
            result = bsp::usart::init(port, bsp::usart::mode::block);
        }
        if (result == status::ok)
        {
            result = bsp::usart::start_rx_to_idle(port.instance, dma,
                bsp::usart::rx_callback::bind<&receiver::receive>(this));
        }
        return result;
    }

    bool read(reading& output, uint32_t now, uint32_t timeout)
    {
        uint8_t bytes[queue_size];
        uint32_t count = 0;
        const auto previous = tx_interrupt_control(TX_INT_DISABLE);
        if (overflow_)
        {
            read_pos_ = write_pos_;
            overflow_ = false;
            used_ = 0;
        }
        count = write_pos_ - read_pos_;
        const uint32_t offset = read_pos_ % queue_size;
        const uint32_t first = count < queue_size - offset ? count : queue_size - offset;
        std::memcpy(bytes, queue_ + offset, first);
        std::memcpy(bytes + first, queue_, count - first);
        read_pos_ = write_pos_;
        tx_interrupt_control(previous);

        for (uint32_t i = 0; i < count; ++i)
        {
            frame_[used_++] = bytes[i];
            if (used_ == FrameSize)
            {
                reading decoded{};
                if (Decode(frame_, FrameSize, decoded) == pnxtypes::status::ok)
                {
                    latest_ = decoded;
                    received_ = true;
                    last_valid_ = now;
                    used_ = 0;
                }
                else
                {
                    std::memmove(frame_, frame_ + 1, FrameSize - 1);
                    --used_;
                }
            }
        }
        output = latest_;
        return received_ && remotes::enabled() && now - last_valid_ < timeout;
    }

private:
    void receive(bsp::usart::handle, uint8_t* data, uint32_t size)
    {
        for (uint32_t i = 0; i < size; ++i)
        {
            if (write_pos_ - read_pos_ == queue_size)
            {
                overflow_ = true;
            }
            else
            {
                queue_[write_pos_ % queue_size] = data[i];
                ++write_pos_;
            }
        }
    }

    static constexpr uint32_t queue_size = 128;
    uint8_t queue_[queue_size]{};
    uint32_t read_pos_ = 0;
    uint32_t write_pos_ = 0;
    uint8_t frame_[FrameSize]{};
    uint32_t used_ = 0;
    uint32_t last_valid_ = 0;
    bool received_ = false;
    volatile bool overflow_ = false;
    reading latest_{};
};

namespace dr16
{
using receiver = remotes::receiver<frame_size, decode>;
}

namespace vt03
{
using receiver = remotes::receiver<frame_size, decode>;
}
} // namespace remotes
