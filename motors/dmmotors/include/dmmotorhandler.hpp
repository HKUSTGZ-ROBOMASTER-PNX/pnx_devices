#pragma once

#include "dmmotor.hpp"

#include <cstdint>

namespace devices {

constexpr int kMaxCanNum = 3;
constexpr int kMaxMotorNum = 4;
constexpr uint32_t kDmCanIdBase = 0x01;
constexpr uint32_t kDmMasterIdBase = 0x11;

class DmMotorHandler {
 public:
  static DmMotorHandler& instance();

  void register_motor(DmMotor* motor);
  void send_control_data();
  void update_feedback(bsp::CanBus bus, const uint8_t* rx_data, int index);

  void enable_motor_block(DmMotor* motor);
  void enable_motor(DmMotor* motor);
  void disable_motor(DmMotor* motor);
  void save_zero_position(DmMotor* motor);
  void clear_error(DmMotor* motor);
  void all_motor_alive_check();

 private:
  DmMotorHandler();

  static void on_can_rx(bsp::CanBus bus, uint32_t id, const uint8_t* data, uint8_t len,
                        void* user_data);
  void ensure_bus_ready(bsp::CanBus bus, bsp::CanType type);
  uint32_t control_id(const DmMotor* motor) const;

  DmMotor* motor_list_[kMaxCanNum][kMaxMotorNum]{};
  bool rx_registered_[kMaxCanNum]{};
};

}  // namespace devices
