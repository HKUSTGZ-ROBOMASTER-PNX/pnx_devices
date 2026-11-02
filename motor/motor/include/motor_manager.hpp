#pragma once

#include <cstdint>
#include <cstring>
#include "tx_api.h"

namespace motor
{

// The CAN interrupt stores the latest frame; the control task takes it.
class feedback_buffer
{
public:
    void push(const uint8_t data[8]) noexcept
    {
        std::memcpy(data_, data, sizeof(data_));
        pending_ = true;
    }

    bool take(uint8_t data[8]) noexcept
    {
        const auto previous = tx_interrupt_control(TX_INT_DISABLE);
        const bool pending = pending_;
        if (pending)
        {
            std::memcpy(data, data_, sizeof(data_));
            pending_ = false;
        }
        tx_interrupt_control(previous);
        return pending;
    }

private:
    uint8_t data_[8]{};
    bool pending_ = false;
};

class manager_interface
{
public:
    virtual ~manager_interface() = default;
    virtual void update() = 0;
    virtual void transmit(uint32_t now_us) = 0;
};

class motor_manager
{
public:
    static constexpr uint8_t max_managers = 8;

    bool register_manager(manager_interface& manager) noexcept
    {
        for (uint8_t i = 0; i < count_; ++i)
        {
            if (managers_[i] == &manager)
            {
                return false;
            }
        }
        if (count_ == max_managers)
        {
            return false;
        }
        managers_[count_++] = &manager;
        return true;
    }

    void update()
    {
        for (uint8_t i = 0; i < count_; ++i)
        {
            managers_[i]->update();
        }
    }

    void transmit(uint32_t now_us)
    {
        for (uint8_t i = 0; i < count_; ++i)
        {
            managers_[i]->transmit(now_us);
        }
    }

private:
    manager_interface* managers_[max_managers]{};
    uint8_t count_ = 0;
};

} // namespace motor
