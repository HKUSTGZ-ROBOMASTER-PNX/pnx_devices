#include "motorservice.hpp"

#include "djimotorhandler.hpp"
#include "dmmotorhandler.hpp"
#include "lkmotorhandler.hpp"
#include "xv2motorhandler.hpp"
#include "motorregistry.hpp"
#include "robot_config.hpp"

namespace motors
{

motor_registry& motor_registry::instance() noexcept
{
    static motor_registry registry;
    return registry;
}

bool motor_registry::add(motor& device) noexcept
{
    for (std::size_t index = 0U; index < count_; ++index)
    {
        if (motors_[index] == &device)
        {
            return true;
        }
    }
    if (count_ >= motors_.size())
    {
        overflowed_ = true;
        return false;
    }
    motors_[count_++] = &device;
    return true;
}

motor* motor_registry::at(std::size_t index) const noexcept
{
    return index < count_ ? motors_[index] : nullptr;
}

motorhandler* motor_service::handler_for(protocol value) noexcept
{
    switch (value)
    {
        case protocol::dji:
            if constexpr (robot::motors::has_dji)
            {
                return &djimotorhandler::instance();
            }
            return nullptr;
        case protocol::dm:
            if constexpr (robot::motors::has_dm)
            {
                return &dmmotorhandler::instance();
            }
            return nullptr;
        case protocol::lk:
            if constexpr (robot::motors::has_lk)
            {
                return &lkmotorhandler::instance();
            }
            return nullptr;
        case protocol::xv2:
            if constexpr (robot::motors::has_xv2)
            {
                return &xv2motorhandler::instance();
            }
            return nullptr;
        case protocol::count:
        default:
            return nullptr;
    }
}

} // namespace motors
