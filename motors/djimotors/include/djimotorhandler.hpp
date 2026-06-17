#pragma once

#include "djimotor.hpp"

#include <cstdint>

namespace devices {

constexpr uint32_t kControlIdLow = 0x200;
constexpr uint32_t kControlIdHigh = 0x1FF;

class DjiMotorHandler {
 public:
  static DjiMotorHandler& instance();

  void register_motor(DjiMotor* motor);
  void send_control_data();
  void update_feedback(bsp::CanBus bus, const uint8_t* rx_data, int index);
  bool all_motor_alive();
  void all_motor_blocked_check();

 private:
  DjiMotorHandler();

  static void on_can_rx(bsp::CanBus bus, uint32_t id, const uint8_t* data, uint8_t len,
                        void* user_data);

  void ensure_bus_ready(bsp::CanBus bus, bsp::CanType type);

  DjiMotor* motor_list_[3][8]{};
  uint8_t send_data_low_[3][8]{};
  uint8_t send_data_high_[3][8]{};
  bool use_low_frame_[3]{};
  bool use_high_frame_[3]{};
  bool rx_registered_[3]{};
};

}  // namespace devices
