#pragma once

#include "tx_api.h"

#include "dmmotors.hpp"
#include "motorhandler.hpp"
#include "robot_config.hpp"

#include <cstdint>

namespace motors
{

constexpr int dm_max_motors = static_cast<int>(robot::motors::dm::max_motors);
constexpr uint32_t dm_id_base = robot::motors::dm::id_base;
constexpr uint32_t dm_master_id_base = robot::motors::dm::master_id_base;

class dmmotorhandler : public motorhandler
{
public:
    static dmmotorhandler& instance();

    protocol supported_protocol() const noexcept override { return protocol::dm; }
    bool register_motor(motor& device) noexcept override;
    bool start() noexcept override;
    void stop() noexcept override;
    void send_control() noexcept override;

    void enable_block(dmmotor* motor);
    void enable(dmmotor* motor);
    void disable(dmmotor* motor);
    void save_zero(dmmotor* motor);
    void clear_error(dmmotor* motor);

private:
    dmmotorhandler();

    static void rx_entry(bsp::can::bus bus, const bsp::can::rx_frame& frame, void* user_data);

    void dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) override;

    uint32_t control_id(const dmmotor* motor) const;

    dmmotor* motor_list_[bsp::can::bus_count][dm_max_motors]{};
};

} // namespace motors
