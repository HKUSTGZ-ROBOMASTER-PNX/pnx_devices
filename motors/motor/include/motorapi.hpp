#pragma once

#include <cstdint>

namespace motors
{

enum class state : uint8_t
{
    offline = 0,
    online,
    blocked,
};

enum class mode : uint8_t
{
    relax = 0,
    current,
    torque,
    mit,
    pos_speed,
    speed,
    multi,
};

struct command
{
    int16_t current = 0;
    float torque = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
};

struct feedback
{
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float current = 0.0f;
    float temperature = 0.0f;
    uint8_t error_code = 0;
};

struct capabilities
{
    bool current = false;
    bool torque = false;
    bool mit = false;
    bool pos_speed = false;
    bool velocity = false;
};

class api
{
public:
    virtual ~api() = default;

    virtual void set_command(const command& next_command, mode next_mode) = 0;
    virtual const feedback& get_feedback() const = 0;
    virtual state status() const = 0;
    virtual capabilities get_capabilities() const = 0;
    virtual bool supports(mode request_mode) const = 0;
    virtual void relax() = 0;
};

} // namespace motors
