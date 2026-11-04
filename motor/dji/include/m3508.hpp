#pragma once
#include "dji_motor.hpp"

namespace motor::dji
{
class m3508 final : public motor
{
public:
    enum class gear_ratio
    {
        standard_gearbox, // 3591 / 187
        direct_drive,     // 1
        xroll_gearbox,    // 268 / 17
    };

    m3508() noexcept : m3508(gear_ratio::standard_gearbox)
    {
    }

    explicit m3508(gear_ratio ratio) noexcept
        : motor(config_t{ratio_value(ratio), 16384, 16384.0f, 100.0f, 8192})
    {
    }

private:
    static constexpr float ratio_value(gear_ratio ratio) noexcept
    {
        if (ratio == gear_ratio::direct_drive)
        {
            return 1.0f;
        }
        if (ratio == gear_ratio::xroll_gearbox)
        {
            return 268.0f / 17.0f;
        }
        return 3591.0f / 187.0f;
    }
};
} // namespace motor::dji
