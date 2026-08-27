#include "motorhandler.hpp"

namespace motors
{

uint8_t motorhandler::bus_index(bsp::can::bus bus)
{
    return static_cast<uint8_t>(bus);
}

bool motorhandler::ensure_bus(bsp::can::bus bus, bsp::can::bus_type type,
                              bsp::can::rx_handler entry, void* user_data)
{
    const uint8_t idx = bus_index(bus);
    if (idx >= bsp::can::bus_count)
    {
        return false;
    }
    if (rx_ready_[idx])
    {
        return true;
    }
    if (bsp::can::init(bus, type) != types::status::ok)
    {
        return false;
    }
    if (bsp::can::register_rx_handler(bus, entry, user_data) != types::status::ok)
    {
        return false;
    }
    rx_ready_[idx] = true;
    return true;
}

} // namespace motors
