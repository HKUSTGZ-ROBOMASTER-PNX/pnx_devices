#pragma once

#include <cstdint>

namespace devices {

enum class Color : uint8_t {
  Red = 0,
  Green,
  Blue,
  White,
};

void init();
void on(Color color);
void off(Color color);
void toggle(Color color);
void blink(Color color);
void all_off();
void all_on();

}  // namespace devices
