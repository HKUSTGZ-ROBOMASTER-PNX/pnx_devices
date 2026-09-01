#include "motorservice.hpp"

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
        handler = handler_for(device.get_protocol());
    }
    if (handler == nullptr || handler->supported_protocol() != device.get_protocol() ||
        !handler->register_motor(device))
    {
        return false;
    }

    motors_[motor_count_++] = &device;
    return true;
}

types::status motor_service::start(const motor_service_config& config) noexcept
{
    if (config.control_period_ticks == 0U || config.alive_check_period_ticks == 0U)
    {
        return types::status::invalid_arg;
    }
    if (started_)
    {
        return types::status::ok;
    }

    bool ok = true;
    for (motorhandler* handler : handlers_)
    {
        if (handler != nullptr && !handler->start())
        {
            ok = false;
        }
    }
    if (!ok)
    {
        return types::status::error;
    }

    config_ = config;
    const std::uint32_t now = static_cast<std::uint32_t>(tx_time_get());
    last_control_tick_ = now - config_.control_period_ticks;
    last_alive_check_tick_ = now - config_.alive_check_period_ticks;
    started_ = true;
    return types::status::ok;
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
    started_ = false;
}

void motor_service::process(std::uint32_t now_ticks) noexcept
{
    if (!started_)
    {
        return;
    }

    if ((now_ticks - last_control_tick_) >= config_.control_period_ticks)
    {
        send_control();
        last_control_tick_ = now_ticks;
    }
    if ((now_ticks - last_alive_check_tick_) >= config_.alive_check_period_ticks)
    {
        alive_check();
        last_alive_check_tick_ = now_ticks;
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
