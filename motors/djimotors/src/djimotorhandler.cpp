#include "djimotorhandler.hpp"

#include "bsp_can.hpp"

namespace devices {

namespace {

uint8_t bus_index(bsp::CanBus bus) { return static_cast<uint8_t>(bus); }

}  // namespace

DjiMotorHandler& DjiMotorHandler::instance() {
  static DjiMotorHandler handler;
  return handler;
}

DjiMotorHandler::DjiMotorHandler() {
  for (auto& row : motor_list_) {
    for (auto& slot : row) {
      slot = nullptr;
    }
  }
}

void DjiMotorHandler::ensure_bus_ready(bsp::CanBus bus, bsp::CanType type) {
  const uint8_t idx = bus_index(bus);
  if (!rx_registered_[idx]) {
    bsp::can_init(bus, type);
    bsp::can_register_rx_callback(bus, on_can_rx, this);
    rx_registered_[idx] = true;
  }
}

void DjiMotorHandler::register_motor(DjiMotor* motor) {
  if (motor == nullptr || motor->can_id() < 0x201 || motor->can_id() > 0x208) {
    return;
  }

  ensure_bus_ready(motor->bus(), motor->can_type());

  const uint8_t bus = bus_index(motor->bus());
  const uint8_t slot = static_cast<uint8_t>(motor->can_id() - 0x201);
  motor_list_[bus][slot] = motor;

  if (motor->can_id() <= 0x204) {
    use_low_frame_[bus] = true;
  } else {
    use_high_frame_[bus] = true;
  }
}

void DjiMotorHandler::send_control_data() {
  for (uint8_t bus = 0; bus < 3; ++bus) {
    for (uint8_t i = 0; i < 8; ++i) {
      DjiMotor* motor = motor_list_[bus][i];
      if (motor == nullptr) {
        continue;
      }
      motor->set_output();

      if (motor->can_id() >= 0x201 && motor->can_id() <= 0x204) {
        const int index = static_cast<int>((motor->can_id() - 0x200) * 2);
        send_data_low_[bus][index - 2] = static_cast<uint8_t>(motor->current_set >> 8);
        send_data_low_[bus][index - 1] = static_cast<uint8_t>(motor->current_set);
      }
      if (motor->can_id() >= 0x205 && motor->can_id() <= 0x208) {
        const int index = static_cast<int>((motor->can_id() - 0x204) * 2);
        send_data_high_[bus][index - 2] = static_cast<uint8_t>(motor->current_set >> 8);
        send_data_high_[bus][index - 1] = static_cast<uint8_t>(motor->current_set);
      }
    }

    const auto can_bus = static_cast<bsp::CanBus>(bus);
    if (use_low_frame_[bus]) {
      bsp::can_transmit(can_bus, kControlIdLow, send_data_low_[bus], 8);
    }
    if (use_high_frame_[bus]) {
      bsp::can_transmit(can_bus, kControlIdHigh, send_data_high_[bus], 8);
    }
  }
}

void DjiMotorHandler::update_feedback(bsp::CanBus bus, const uint8_t* rx_data, int index) {
  if (index < 0 || index >= 8) {
    return;
  }
  DjiMotor* motor = motor_list_[bus_index(bus)][static_cast<uint8_t>(index)];
  if (motor != nullptr) {
    motor->on_can_rx(rx_data, 8);
  }
}

void DjiMotorHandler::on_can_rx(bsp::CanBus bus, uint32_t id, const uint8_t* data, uint8_t len,
                                void* user_data) {
  if (id < 0x201 || id > 0x208 || user_data == nullptr) {
    return;
  }
  auto* handler = static_cast<DjiMotorHandler*>(user_data);
  handler->update_feedback(bus, data, static_cast<int>(id - 0x201));
  (void)len;
}

bool DjiMotorHandler::all_motor_alive() {
  for (const auto& row : motor_list_) {
    for (DjiMotor* motor : row) {
      if (motor != nullptr && motor->alive_check() == State::Offline) {
        return false;
      }
    }
  }
  return true;
}

void DjiMotorHandler::all_motor_blocked_check() {
  for (auto& row : motor_list_) {
    for (DjiMotor* motor : row) {
      if (motor != nullptr) {
        motor->blocked_check();
      }
    }
  }
}

}  // namespace devices
