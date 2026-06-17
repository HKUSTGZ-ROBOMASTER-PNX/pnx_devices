# pnx_devices

设备驱动层：电机（DJI/LK/DM）、IMU、LED 等，统一 `devices::Actuator` 接口。

## 依赖

- `pnx_bsp`、`pnx_libraries`、`pnx_os`、`pnx_platform`

## CMake 目标

`pnx_devices`（静态库）

## Submodule

```bash
git submodule add <url> pnx_devices
```

## 目录

```
motors/    djimotors, lkmotors, dmmotors
imu/       Bmi088 等
led/       WS2812 指示灯
```
