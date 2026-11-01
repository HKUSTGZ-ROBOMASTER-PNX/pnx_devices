#pragma once
#include "bsp_gpio.hpp"
#include "bsp_spi.hpp"
#include "status.hpp"
#include <stdint.h>

namespace remotes::ps2
{
inline constexpr uint8_t frame_size = 9U;

struct frame
{
    uint8_t bytes[frame_size]{};
};

struct button_state
{
    bool select = false;
    bool l3 = false;
    bool r3 = false;
    bool start = false;
    bool up = false;
    bool right = false;
    bool down = false;
    bool left = false;
    bool l2 = false;
    bool r2 = false;
    bool l1 = false;
    bool r1 = false;
    bool triangle = false;
    bool circle = false;
    bool cross = false;
    bool square = false;
};

struct sample
{
    button_state buttons{}; // True while pressed.
    float lx = 0.0f;
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
};

// Scheduling, button edges and publication belong to the application.
pnxtypes::status decode(const frame& raw, sample& out, float deadzone = 0.02f) noexcept;

class spi
{
public:
    constexpr spi(bsp::spi::handle spi_bus, bsp::gpio::config cs,
        uint8_t (&tx)[frame_size], uint8_t (&rx)[frame_size]) noexcept
        : spi_bus_(spi_bus), cs_(cs), tx_(tx), rx_(rx)
    {
    }

    pnxtypes::status init();
    pnxtypes::status start();
    pnxtypes::status read(frame& raw);
    pnxtypes::status cancel();

private:
    bsp::spi::handle spi_bus_;
    bsp::gpio::config cs_;
    uint8_t (&tx_)[frame_size];
    uint8_t (&rx_)[frame_size];
    bool pending_ = false;
};

} // namespace remotes::ps2
