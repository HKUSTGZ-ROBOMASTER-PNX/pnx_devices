#include "dmmotorhandler.hpp"

#include "bsp_can.hpp"
#include "os.hpp"

namespace devices {

namespace {

uint8_t bus_index(bsp::CanBus bus) { return static_cast<uint8_t>(bus); }

}  // namespace

DmMotorHandler& DmMotorHandler::instance() {
  static DmMotorHandler handler;
  return handler;
}

DmMotorHandler::DmMotorHandler() {
  for (auto& row : motor_list_) {
    for (auto& slot : row) {
      slot = nullptr;
    }
  }
}

void DmMotorHandler::ensure_bus_ready(bsp::CanBus bus, bsp::CanType type) {
  const uint8_t idx = bus_index(bus);
  if (!rx_registered_[idx]) {
    bsp::can_init(bus, type);
    bsp::can_register_rx_callback(bus, on_can_rx, this);
    rx_registered_[idx] = true;
  }
}

void DmMotorHandler::register_motor(DmMotor* motor) {
  if (motor == nullptr || motor->can_id() < kDmCanIdBase ||
      motor->can_id() >= kDmCanIdBase + kMaxMotorNum) {
    return;
  }

  ensure_bus_ready(motor->bus(), motor->can_type());

  const uint8_t bus = bus_index(motor->bus());
  const uint8_t slot = static_cast<uint8_t>(motor->can_id() - kDmCanIdBase);
  motor_list_[bus][slot] = motor;
}

uint32_t DmMotorHandler::control_id(const DmMotor* motor) const {
  uint32_t frame_id = motor->can_id();
  switch (motor->control_mode) {
    case ActuatorMode::PosSpeed:
      return frame_id + 0x100;
    case ActuatorMode::Speed:
      return frame_id + 0x200;
    case ActuatorMode::Multi:
      return 0x300;
    default:
      return frame_id;
  }
}

void DmMotorHandler::send_control_data() {
  for (int i = 0; i < kMaxCanNum; ++i) {
    for (int j = 0; j < kMaxMotorNum; ++j) {
      if (motor_list_[i][j] != nullptr) {
        motor_list_[i][j]->set_output();
      }
    }
  }
}

void DmMotorHandler::update_feedback(bsp::CanBus bus, const uint8_t* rx_data, int index) {
  if (index < 0 || index >= kMaxMotorNum) {
    return;
  }
  DmMotor* motor = motor_list_[bus_index(bus)][static_cast<uint8_t>(index)];
  if (motor != nullptr) {
    motor->on_can_rx(rx_data, 8);
  }
}

void DmMotorHandler::on_can_rx(bsp::CanBus bus, uint32_t id, const uint8_t* data, uint8_t len,
                               void* user_data) {
  if (id < kDmMasterIdBase || id > kDmMasterIdBase + 3 || user_data == nullptr) {
    return;
  }
  auto* handler = static_cast<DmMotorHandler*>(user_data);
  handler->update_feedback(bus, data, static_cast<int>(id - kDmMasterIdBase));
  (void)len;
}

void DmMotorHandler::enable_motor_block(DmMotor* motor) {
  if (motor == nullptr || motor->control_mode == ActuatorMode::Multi) {
    return;
  }

  uint16_t timeout = 0;
  do {
    timeout++;
    if (timeout > 1000) {
      motor->enable_failed_ = true;
      break;
    }
    motor->transmit_frame(control_id(motor), DmMotor::kEnableFrame, 8);
    os::thread_sleep(1);
  } while (motor->raw_.err != ErrorCode::Enable);
}

void DmMotorHandler::enable_motor(DmMotor* motor) {
  if (motor == nullptr || motor->control_mode == ActuatorMode::Multi) {
    return;
  }
  motor->transmit_frame(control_id(motor), DmMotor::kEnableFrame, 8);
}

void DmMotorHandler::disable_motor(DmMotor* motor) {
  if (motor == nullptr || motor->control_mode == ActuatorMode::Multi) {
    return;
  }
  motor->transmit_frame(control_id(motor), DmMotor::kDisableFrame, 8);
}

void DmMotorHandler::save_zero_position(DmMotor* motor) {
  if (motor == nullptr) {
    return;
  }
  motor->transmit_frame(control_id(motor), DmMotor::kSaveZeroFrame, 8);
}

void DmMotorHandler::clear_error(DmMotor* motor) {
  if (motor == nullptr) {
    return;
  }
  motor->transmit_frame(control_id(motor), DmMotor::kClearErrorFrame, 8);
}

void DmMotorHandler::all_motor_alive_check() {
  for (auto& row : motor_list_) {
    for (DmMotor* motor : row) {
      if (motor != nullptr) {
        motor->alive_check();
      }
    }
  }
}

}  // namespace devices
