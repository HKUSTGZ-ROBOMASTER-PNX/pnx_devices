#include "motorservice.hpp"

#include "motorregistry.hpp"

namespace motors
{

namespace
{

constexpr std::size_t protocol_index(protocol value) noexcept
{
    return static_cast<std::size_t>(value);
}

} // namespace

bool motor_service::register_motor(motor& device) noexcept
{
    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        if (motors_[i] == &device)
        {
            return true;
        }
    }

    const std::size_t index = protocol_index(device.get_protocol());
    if (index >= handlers_.size() || motor_count_ >= motors_.size())
    {
        return false;
    }

    motorhandler*& handler = handlers_[index];
    if (handler == nullptr)
    {
        handler = detail::handler_for(device.get_protocol());
    }
    if (handler == nullptr || handler->supported_protocol() != device.get_protocol() ||
        !handler->register_motor(device))
    {
        return false;
    }

    motors_[motor_count_++] = &device;
    return true;
}

bool motor_service::start() noexcept
{
    bool ok = true;
    for (motorhandler* handler : handlers_)
    {
        if (handler != nullptr && !handler->start())
        {
            ok = false;
        }
    }
    return ok;
}

void motor_service::stop() noexcept
{
    for (motorhandler* handler : handlers_)
    {
        if (handler != nullptr)
        {
            handler->stop();
        }
    }
}

void motor_service::send_control() noexcept
{
    for (motorhandler* handler : handlers_)
    {
        if (handler != nullptr)
        {
            handler->send_control();
        }
    }
}

void motor_service::alive_check() noexcept
{
    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        motors_[i]->alive_check();
    }
}

} // namespace motors
