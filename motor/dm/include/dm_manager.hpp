#pragma once
#include "motor_utils.hpp"
#include "motor_manager.hpp"
#include "dm_motor.hpp"
#include "bsp_can.hpp"
#include <cstdint>

namespace motor::dm
{
class manager final : public manager_interface
{
public:
    manager() = default;
    manager(const manager&) = delete;
    manager& operator=(const manager&) = delete;

    // Register before CAN init. The manager and motors must outlive the CAN callbacks.
    bool register_motor(motor& m, bsp::can::handle bus, uint16_t control_id,
        uint16_t feedback_id) noexcept;
    bool receive(bsp::can::handle bus, uint16_t feedback_id, const uint8_t data[8]) noexcept;
    bool update_feedback(motor& m) noexcept;
    bool special(motor& m, uint8_t opcode) noexcept;
    void update() override;

    void set_period(float seconds) noexcept
    {
        period_ = seconds;
    }

    bool set_tx_period(const motor& m, uint32_t us) noexcept
    {
        for (uint8_t i = 0; i < count_; ++i)
        {
            if (entries_[i].ptr == &m)
            {
                return tx_[i].set_period(us);
            }
        }
        return false;
    }

    void transmit(uint32_t now_us) override;

private:
    struct entry
    {
        motor* ptr = nullptr;
        uint16_t feedback_id = 0;
        bsp::can::handle bus = bsp::can::handle::none;
        feedback_buffer feedback{};
    };

    void on_receive(bsp::can::handle bus, const bsp::can::rx_frame& frame) noexcept;
    entry entries_[24]{};
    tx_schedule tx_[24]{};
    uint8_t count_ = 0;
    float period_ = 0.001f;
};
} // namespace motor::dm
