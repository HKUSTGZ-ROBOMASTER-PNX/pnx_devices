#pragma once

#include "motor.hpp"
#include "motorhandler.hpp"

#include <array>
#include <cstddef>

namespace motors
{

class motor_service
{
public:
    bool register_motor(motor& device) noexcept;

    types::status start() noexcept;
    void stop() noexcept;
    void send_control() noexcept;
    void alive_check() noexcept;

private:
    static constexpr std::size_t protocol_count = static_cast<std::size_t>(protocol::count);
    static constexpr std::size_t max_motors = bsp::can::bus_count * 8U;

    // Resolves the static-lifetime handler for a protocol.
    static motorhandler* handler_for(protocol value) noexcept;

    // Registered handlers and motors are non-owning and must outlive this service.
    std::array<motorhandler*, protocol_count> handlers_{};
    std::array<motor*, max_motors> motors_{};
    std::size_t motor_count_ = 0U;
    bool started_ = false;
};

} // namespace motors
