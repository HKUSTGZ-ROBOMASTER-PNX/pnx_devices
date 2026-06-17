#pragma once

#include "bsp_can.hpp"

#include <cstdint>
#include <type_traits>

namespace devices {

struct Config {
  bsp::CanBus bus = bsp::CanBus::Bus1;
  bsp::CanType can_type = bsp::CanType::Classic;
  uint32_t can_id = 0;
};

enum class Family : uint8_t {
  Dji = 0,
  Lk,
  Dm,
};

enum class State : uint8_t {
  Offline = 0,
  Online,
};

struct Feedback {
  float position_fdb = 0.0f;
  float speed_fdb = 0.0f;
  float torque_fdb = 0.0f;
  float temperature_fdb = 0.0f;
  float current_fdb = 0.0f;
};

enum class ActuatorMode : uint8_t {
  Relax = 0,
  Speed,
  Position,
  PositionNoSpeed,
  Torque,
  Mit,
  PosSpeed,
  Multi,
};

/**
 * 统一执行器接口：所有具体电机 public 字段布局一致，模板只需替换类型。
 * 例：template<typename HipType> void foo(HipType* m) { m->feedback.speed_fdb; }
 */
class Actuator {
 public:
  explicit Actuator(Config config) : config_(config) {}
  virtual ~Actuator() = default;

  virtual Family family() const = 0;
  virtual void set_output() = 0;
  virtual State alive_check() = 0;
  virtual void on_can_rx(const uint8_t* data, uint8_t len) = 0;

  void relax();

  const Config& config() const { return config_; }
  bsp::CanBus bus() const { return config_.bus; }
  bsp::CanType can_type() const { return config_.can_type; }
  uint32_t can_id() const { return config_.can_id; }

  Feedback feedback{};
  ActuatorMode control_mode = ActuatorMode::Relax;
  State motor_state = State::Offline;

  float speed_set = 0.0f;
  float position_set = 0.0f;
  float torque_set = 0.0f;
  int16_t current_set = 0;

  float kp = 0.0f;
  float kd = 0.0f;

  uint16_t max_current = 0;
  float offset = 0.0f;

 protected:
  Config config_;
};

using Motor = Actuator;//类型别名，便于理解

template <typename T>
inline constexpr bool is_actuator_v = std::is_base_of_v<Actuator, T>;
//is_actuator_v<T> 用于编译期检查 T 是否继承自 Actuator(T是电机类型)，如果是则为 true，否则为 false

template <typename T, typename = void>
struct ActuatorTraits {
  static constexpr float kTorqueConstant = 0.0f;
  static constexpr float kTorqueToCurrent = 0.0f;
  static constexpr float kCurrentToTorque = 0.0f;
};

//用“类型”查“参数”

#define MOTOR_DEFINE_TRAITS(Type, torque_const, to_current, to_torque)        \
  template <>                                                                 \
  struct ActuatorTraits<Type> {                                               \
    static constexpr float kTorqueConstant = (torque_const);                  \
    static constexpr float kTorqueToCurrent = (to_current);                   \
    static constexpr float kCurrentToTorque = (to_torque);                    \
  }

}  // namespace devices
