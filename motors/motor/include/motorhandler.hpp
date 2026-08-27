#pragma once

#include "bsp_can.hpp"
#include "motor.hpp"

#include <cstdint>

namespace motors
{

class motorhandler
{
public:
    virtual ~motorhandler() = default;

    virtual protocol supported_protocol() const noexcept = 0;
    virtual bool register_motor(motor& device) noexcept = 0;
    virtual bool start() noexcept { return true; }
    virtual void stop() noexcept {}
    virtual void send_control() noexcept = 0;

protected:
    static uint8_t bus_index(bsp::can::bus bus);

    bool ensure_bus(bsp::can::bus bus, bsp::can::bus_type type, bsp::can::rx_handler entry,
                    void* user_data);

    virtual void dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) = 0;

    bool rx_ready_[bsp::can::bus_count]{};
};

} // namespace motors
