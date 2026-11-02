#include "dm_manager.hpp"

namespace motor::dm
{
bool manager::register_motor(motor& m, bsp::can::handle bus, uint16_t control_id,
    uint16_t feedback_id) noexcept
{
    if (bus == bsp::can::handle::none || bus > bsp::can::handle::fdcan3 ||
        control_id == 0 || control_id > 0x7ff ||
        feedback_id > 0x7ff || count_ >= 24)
    {
        return false;
    }
    bool registered_bus = false;
    for (uint8_t i = 0; i < count_; ++i)
    {
        if (entries_[i].ptr == &m ||
            (entries_[i].bus == bus && entries_[i].feedback_id == feedback_id))
        {
            return false;
        }
        registered_bus = registered_bus || entries_[i].bus == bus;
    }
    if (!registered_bus &&
        bsp::can::register_rx_callback(bus,
            bsp::can::rx_callback::bind<&manager::on_receive>(this)) != pnxtypes::status::ok)
    {
        return false;
    }
    m.set_id(control_id);
    const auto previous = tx_interrupt_control(TX_INT_DISABLE);
    auto& entry = entries_[count_++];
    entry.ptr = &m;
    entry.feedback_id = feedback_id;
    entry.bus = bus;
    tx_interrupt_control(previous);
    return true;
}

void manager::on_receive(bsp::can::handle bus, const bsp::can::rx_frame& frame) noexcept
{
    if (frame.id_kind != bsp::can::id_type::standard || frame.remote || frame.len != 8)
    {
        return;
    }
    for (uint8_t i = 0; i < count_; ++i)
    {
        auto& entry = entries_[i];
        if (entry.bus == bus && frame.id == entry.feedback_id)
        {
            entry.feedback.push(frame.data);
            return;
        }
    }
}

bool manager::update_feedback(motor& m) noexcept
{
    for (uint8_t i = 0; i < count_; ++i)
    {
        if (entries_[i].ptr == &m)
        {
            uint8_t data[8];
            return entries_[i].feedback.take(data) && m.receive(data);
        }
    }
    return false;
}

bool manager::receive(bsp::can::handle bus, uint16_t id, const uint8_t data[8]) noexcept
{
    if (!data)
    {
        return false;
    }
    for (uint8_t i = 0; i < count_; ++i)
    {
        if (entries_[i].bus == bus && entries_[i].feedback_id == id)
        {
            return entries_[i].ptr->receive(data);
        }
    }
    return false;
}

bool manager::special(motor& m, uint8_t opcode) noexcept
{
    if (opcode != 0xfc && opcode != 0xfd && opcode != 0xfe && opcode != 0xfb)
    {
        return false;
    }
    for (uint8_t i = 0; i < count_; ++i)
    {
        if (entries_[i].ptr == &m)
        {
            const auto frame = m.special(opcode);
            return bsp::can::transmit(entries_[i].bus, frame.id, frame.data, frame.length) ==
                   pnxtypes::status::ok;
        }
    }
    return false;
}

void manager::update()
{
    for (uint8_t i = 0; i < count_; ++i)
    {
        entries_[i].ptr->health_tick(period_);
        entries_[i].ptr->update();
    }
}

void manager::transmit(uint32_t now_us)
{
    for (uint8_t i = 0; i < count_; ++i)
    {
        if (!tx_[i].due(now_us))
        {
            continue;
        }
        const auto frame = entries_[i].ptr->output();
        if (bsp::can::transmit(entries_[i].bus, frame.id, frame.data, frame.length) ==
            pnxtypes::status::ok)
        {
            tx_[i].mark_sent(now_us);
        }
    }
}
} // namespace motor::dm
