#pragma once

#include "bsp_pwm.hpp"

namespace buzzer
{
struct config
{
    bsp::pwm::config timer;
    bsp::pwm::channel channel;
    uint32_t period_us = 500;
    uint32_t on_ms = 100;
    uint32_t gap_ms = 100;
};

// Caller-driven, single thread, exclusive timer ownership. Call update frequently.
// A new play replaces the pending sequence. No thread or timer callback is created.
class player
{
public:
    explicit player(config settings) noexcept : cfg_(settings)
    {
    }

    pnxtypes::status play(uint8_t count, uint32_t now_ms) noexcept
    {
        if (count == 0 || cfg_.on_ms == 0 || cfg_.gap_ms == 0)
        {
            return pnxtypes::status::invalid_arg;
        }
        remaining_ = 0;
        auto result = bsp::pwm::set_duty(cfg_.timer.instance, cfg_.channel, 0);
        if (result != pnxtypes::status::ok)
        {
            return result;
        }
        result = bsp::pwm::set_period_us(cfg_.timer, cfg_.period_us);
        if (result != pnxtypes::status::ok)
        {
            return result;
        }
        // HAL rejects starting an already running channel. Keep PWM running at
        // zero duty between pulses and only enable the channel once.
        if (!started_)
        {
            result = bsp::pwm::start(cfg_.timer.instance, cfg_.channel);
            if (result != pnxtypes::status::ok)
            {
                return result;
            }
            started_ = true;
        }
        result = bsp::pwm::set_duty(cfg_.timer.instance, cfg_.channel, 0.5f);
        if (result == pnxtypes::status::ok)
        {
            remaining_ = count;
            sounding_ = true;
            changed_ms_ = now_ms;
        }
        return result;
    }

    pnxtypes::status update(uint32_t now_ms) noexcept
    {
        if (remaining_ == 0 || now_ms - changed_ms_ < (sounding_ ? cfg_.on_ms : cfg_.gap_ms))
        {
            return pnxtypes::status::ok;
        }
        const bool next = !sounding_;
        auto result = bsp::pwm::set_duty(cfg_.timer.instance, cfg_.channel, next ? 0.5f : 0);
        if (result == pnxtypes::status::ok)
        {
            sounding_ = next;
            changed_ms_ = now_ms;
            if (!sounding_)
            {
                --remaining_;
            }
        }
        return result;
    }

private:
    config cfg_;
    uint32_t changed_ms_ = 0;
    uint8_t remaining_ = 0;
    bool sounding_ = false;
    bool started_ = false;
};
} // namespace buzzer
