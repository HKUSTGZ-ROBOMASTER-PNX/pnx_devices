#include "led.hpp"

#include "bsp_ws2812.hpp"

namespace devices {

void init() { all_on(); }

void all_on() { bsp::ws2812_set_rgb(7, 7, 7); }

void all_off() { bsp::ws2812_set_rgb(0, 0, 0); }

void on(Color color) {
  switch (color) {
    case Color::Red:
      bsp::ws2812_set_rgb(7, 0, 0);
      break;
    case Color::Green:
      bsp::ws2812_set_rgb(0, 7, 0);
      break;
    case Color::Blue:
      bsp::ws2812_set_rgb(0, 0, 7);
      break;
    case Color::White:
      bsp::ws2812_set_rgb(7, 7, 7);
      break;
  }
}

void off(Color) { all_off(); }

void toggle(Color color) {
  static bool enabled = false;
  enabled = !enabled;
  if (enabled) {
    on(color);
  } else {
    off(color);
  }
}

void blink(Color color) {
  static uint32_t flash_count = 0;
  flash_count++;

  if (flash_count <= 250) {
    all_off();
  } else if (flash_count <= 500) {
    on(color);
  }

  if (flash_count >= 500) {
    flash_count = 0;
  }
}

}  // namespace devices
