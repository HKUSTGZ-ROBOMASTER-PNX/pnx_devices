#pragma once

#include "bsp_can.hpp"

#include <array>
#include <cstddef>

namespace motors
{

class motor;

// Collects static-lifetime motor objects as they are constructed.  A service
// calls register_all() after its application has created the motors it owns.
class motor_registry
{
public:
    static motor_registry& instance() noexcept;

    bool add(motor& device) noexcept;
    std::size_t size() const noexcept { return count_; }
    motor* at(std::size_t index) const noexcept;
    bool overflowed() const noexcept { return overflowed_; }

private:
    static constexpr std::size_t max_motors = bsp::can::bus_count * 8U;

    std::array<motor*, max_motors> motors_{};
    std::size_t count_ = 0U;
    bool overflowed_ = false;
};

} // namespace motors
