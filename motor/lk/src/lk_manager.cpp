#include "lk_manager.hpp"

namespace motor::lk
{
bool manager::register_motor(motor& m, bsp::can::handle bus, uint16_t id) noexcept
{
    const auto index = static_cast<uint8_t>(bus);
    if (index == 0 || index >= buses || id < 0x141 || id > 0x144)
    {
        return false;
    }
    for (const auto& registered_bus : motors_)
    {
        for (const auto* registered_motor : registered_bus)
        {
            if (registered_motor == &m)
            {
                return false;
            }
        }
    }
    auto& slot = motors_[index][id - 0x141];
    if (slot)
    {
        return false;
    }
    if (!registered_[index])
    {
        if (bsp::can::register_rx_callback(bus,
                bsp::can::rx_callback::bind<&manager::on_receive>(this)) != pnxtypes::status::ok)
        {
            return false;
        }
        registered_[index] = true;
    }
    const auto previous = tx_interrupt_control(TX_INT_DISABLE);
    slot = &m;
    tx_interrupt_control(previous);
    return true;
}

void manager::on_receive(bsp::can::handle bus, const bsp::can::rx_frame& frame) noexcept
{
    const auto index = static_cast<uint8_t>(bus);
    if (index == 0 || index >= buses || frame.id_kind != bsp::can::id_type::standard ||
        frame.remote || frame.len != 8 || frame.id < 0x141 || frame.id > 0x144)
    {
        return;
    }
    const auto slot = frame.id - 0x141;
    if (motors_[index][slot])
    {
        feedback_[index][slot].push(frame.data);
    }
}

bool manager::update_feedback(motor& m) noexcept
{
    for (uint8_t bus = 1; bus < buses; ++bus)
    {
        for (uint8_t slot = 0; slot < 4; ++slot)
        {
            if (motors_[bus][slot] == &m)
            {
                uint8_t data[8];
                if (feedback_[bus][slot].take(data))
                {
                    m.receive(data);
                    return true;
                }
                return false;
            }
        }
    }
    return false;
}

bool manager::receive(bsp::can::handle bus, uint16_t id, const uint8_t data[8]) noexcept
{
    const auto index = static_cast<uint8_t>(bus);
    if (!data || index == 0 || index >= buses || id < 0x141 || id > 0x144)
    {
        return false;
    }
    auto* m = motors_[index][id - 0x141];
    if (!m)
    {
        return false;
    }
    m->receive(data);
    return true;
}

void manager::update()
{
    for (auto& bus : motors_)
    {
        for (auto* m : bus)
        {
            if (m)
            {
                m->health_tick(period_);
                m->update();
            }
        }
    }
}

void manager::transmit(uint32_t now_us)
{
    for (uint8_t bus = 1; bus < buses; ++bus)
    {
        if (!tx_[bus].due(now_us))
        {
            continue;
        }
        frame_t frame{};
        frame.id = 0x280;
        bool populated = false;
        for (uint8_t i = 0; i < 4; ++i)
        {
            auto* m = motors_[bus][i];
            if (!m)
            {
                continue;
            }
            populated = true;
            uint16_t raw = static_cast<uint16_t>(m->output());
            frame.data[2 * i] = static_cast<uint8_t>(raw);
            frame.data[2 * i + 1] = static_cast<uint8_t>(raw >> 8);
        }
        if (populated && bsp::can::transmit(static_cast<bsp::can::handle>(bus), frame.id,
                             frame.data, frame.length) == pnxtypes::status::ok)
        {
            tx_[bus].mark_sent(now_us);
        }
    }
}
} // namespace motor::lk
