#pragma once
#include "motor_manager.hpp"
#include "motor_utils.hpp"
#include "dji_motor.hpp"
#include "bsp_can.hpp"
#include <cstdint>

namespace motor::dji
{
struct frame_t
{
    uint16_t id = 0;
    uint8_t data[8]{};
    uint8_t length = 8;
};

class manager final : public manager_interface
{
public:
    manager() = default;
    manager(const manager&) = delete;
    manager& operator=(const manager&) = delete;

    // Register before CAN init. The manager and motors must outlive the CAN callbacks.
    bool register_motor(motor& instance, bsp::can::handle bus, uint16_t feedback_id) noexcept;
    bool receive(bsp::can::handle bus, uint16_t id, const uint8_t data[8]) noexcept;
    bool update_feedback(motor& instance) noexcept;
    void update() override;

    void set_period(float seconds) noexcept
    {
        period_ = seconds;
    }

    bool set_tx_period(bsp::can::handle bus, uint16_t id, uint32_t us) noexcept
    {
        const auto index = static_cast<uint8_t>(bus);
        return index > 0 && index < buses && (id == 0x200 || id == 0x1FF) &&
               tx_[index][id == 0x200 ? 0 : 1].set_period(us);
    }

    uint32_t transmit_count(bsp::can::handle bus, uint16_t id) const noexcept;
    void transmit(uint32_t now_us) override;

private:
    static constexpr uint8_t buses = static_cast<uint8_t>(bsp::can::handle::fdcan3) + 1;
    void on_receive(bsp::can::handle bus, const bsp::can::rx_frame& frame) noexcept;
    motor* motors_[buses][8]{};
    feedback_buffer feedback_[buses][8]{};
    bool registered_[buses]{};
    tx_schedule tx_[buses][2]{};
    uint32_t transmit_count_[buses][2]{};
    float period_ = 0.001f;
};
} // namespace motor::dji
