#include "lkmotorhandler.hpp"

#include "bsp_can.hpp"

namespace devices {

namespace {

uint8_t bus_index(bsp::CanBus bus) { return static_cast<uint8_t>(bus); }

}  // namespace

LkMotorHandler& LkMotorHandler::instance() {
  static LkMotorHandler handler;
  return handler;
}

LkMotorHandler::LkMotorHandler() {
  for (auto& row : motor_list_) {
    for (auto& slot : row) {
      slot = nullptr;
    }
  }
}

void LkMotorHandler::ensure_bus_ready(bsp::CanBus bus, bsp::CanType type) {
  const uint8_t idx = bus_index(bus);
  if (!rx_registered_[idx]) {
    bsp::can_init(bus, type);
    bsp::can_register_rx_callback(bus, on_can_rx, this);
    rx_registered_[idx] = true;
  }
}

void LkMotorHandler::register_motor(LkMotor* motor) {
  if (motor == nullptr || motor->can_id() < 0x141 || motor->can_id() > 0x144) {
    return;
  }

  ensure_bus_ready(motor->bus(), motor->can_type());

  const uint8_t bus = bus_index(motor->bus());
  const uint8_t slot = static_cast<uint8_t>(motor->can_id() - 0x141);
  motor_list_[bus][slot] = motor;
}

void LkMotorHandler::send_control_data() {
  for (uint8_t bus = 0; bus < 3; ++bus) {
    for (uint8_t i = 0; i < 4; ++i) {
      LkMotor* motor = motor_list_[bus][i];
      if (motor == nullptr) {
        continue;
      }
      motor->set_output();

      if (motor->can_id() >= 0x141 && motor->can_id() <= 0x144) {
        const int index = static_cast<int>((motor->can_id() - 0x140) * 2);
        send_data_[bus][index - 2] = static_cast<uint8_t>(motor->current_set);
        send_data_[bus][index - 1] = static_cast<uint8_t>(motor->current_set >> 8);
      }
    }
    bsp::can_transmit(static_cast<bsp::CanBus>(bus), kControlId, send_data_[bus], 8);
  }
}

void LkMotorHandler::update_feedback(bsp::CanBus bus, const uint8_t* rx_data, int index) {
  if (index < 0 || index >= 4) {
    return;
  }
  LkMotor* motor = motor_list_[bus_index(bus)][static_cast<uint8_t>(index)];
  if (motor != nullptr) {
    motor->on_can_rx(rx_data, 8);
  }
}

void LkMotorHandler::on_can_rx(bsp::CanBus bus, uint32_t id, const uint8_t* data, uint8_t len,
                               void* user_data) {
  if (id < 0x140 || id > 0x160 || user_data == nullptr) {
    return;
  }
  auto* handler = static_cast<LkMotorHandler*>(user_data);
  handler->update_feedback(bus, data, static_cast<int>(id - 0x141));
  (void)len;
}

}  // namespace devices
