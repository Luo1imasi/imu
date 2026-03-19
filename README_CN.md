# roboto_imu

Roboto 平台 IMU 驱动库，提供 C++ 和 Python SDK。当前支持 **超核电子 (HiPNUC)** 系列 IMU，通过 CAN (J1939 / CANopen) 和串口 (UART) 接口通信。

## 架构

```
roboto_imu/
├── include/
│   └── imu_driver.hpp          # 抽象基类 + 工厂模式
├── src/
│   ├── imu_driver.cpp          # 工厂方法: create_imu()
│   ├── pybind_module.cpp       # Python 绑定 (imu_py)
│   ├── drivers/
│   │   └── hipnuc/             # 超核驱动 + C 协议解码器
│   │       ├── hipnuc_imu_driver.hpp / .cpp
│   │       ├── hipnuc_dec.h / .c        # 超核私有协议 (HI91/HI92/HI81/HI83)
│   │       ├── hipnuc_j1939_parser.h / .c  # J1939 PGN 解析器
│   │       ├── hipnuc_can_common.h      # CAN 数据结构
│   │       ├── canopen_parser.h / .c    # CANopen TPDO 解析器
│   │       └── nmea_decode.h / .c       # NMEA 语句解析器
│   └── protocol/
│       ├── can/
│       │   └── socket_can.hpp / .cpp    # SocketCAN 接口（单例，SCHED_FIFO 实时调度）
│       └── serial/
│           └── serial_port.hpp / .cpp   # 串口 UART 接口
```

## 支持设备

| IMU 型号 | 接口 | 协议 | 数据 |
|----------|------|------|------|
| 超核电子 (HiPNUC) | CAN | J1939 | 加速度、陀螺仪、四元数、欧拉角、磁力计、温度、气压 |
| 超核电子 (HiPNUC) | CAN | CANopen | TPDO: 加速度、陀螺仪、欧拉角、四元数、气压 |
| 超核电子 (HiPNUC) | 串口 | 私有协议 (HI91) | 四元数、陀螺仪、加速度 |

## 传感器数据

| 数据 | 方法 | 返回值 | 单位 |
|------|------|--------|------|
| 四元数 | `get_quat()` | `[w, x, y, z]` | - |
| 角速度 | `get_ang_vel()` | `[x, y, z]` | rad/s |
| 线加速度 | `get_lin_acc()` | `[x, y, z]` | m/s² |
| 温度 | `get_temperature()` | `float` | °C |

## 编译

依赖: `spdlog`, `fmt`, `pybind11`, `ament_cmake` (ROS 2)

```bash
colcon build --packages-select roboto_imu
```

## 使用方法

### Python

```python
from imu_py import IMUDriver

# CAN 接口 (J1939)
imu = IMUDriver.create_imu(
    imu_id=0x08,
    interface_type="can",
    interface="can0",
    imu_type="HIPNUC",
    baudrate=0          # baudrate 仅用于串口
)

# 串口接口
imu = IMUDriver.create_imu(
    imu_id=0x01,
    interface_type="serial",
    interface="/dev/ttyUSB0",
    imu_type="HIPNUC",
    baudrate=115200
)

# 读取传感器数据
quat = imu.get_quat()           # [w, x, y, z]
gyro = imu.get_ang_vel()        # [x, y, z] rad/s
accel = imu.get_lin_acc()       # [x, y, z] m/s²
temp = imu.get_temperature()    # °C
```

### C++

```cpp
#include "imu_driver.hpp"

auto imu = IMUDriver::create_imu(0x08, "can", "can0", "HIPNUC");

auto quat = imu->get_quat();         // {w, x, y, z}
auto gyro = imu->get_ang_vel();      // {x, y, z} rad/s
auto accel = imu->get_lin_acc();     // {x, y, z} m/s²
float temp = imu->get_temperature(); // °C
```

## CAN J1939 PGN 表

| PGN | 数据 | 精度 |
|-----|------|------|
| 0xFF34 | 加速度 (x, y, z) | 0.48828 mG/LSB |
| 0xFF37 | 陀螺仪 (x, y, z) | 0.061035 °/s/LSB |
| 0xFF3A | 磁力计 (x, y, z) | 0.030517 |
| 0xFF3D | 俯仰角 & 横滚角 | 0.001° |
| 0xFF41 | 航向角 | 0.001° |
| 0xFF43 | 温度 & 气压 | - |
| 0xFF46 | 四元数 (w, x, y, z) | 0.0001 |
| 0xFF4A | 倾角 | - |
| 0xFF10 | 经度 / 纬度 | 1e-7 |
| 0xFF14 | 海拔高度 | - |
| 0xFF18 | 状态（解算质量、卫星数、INS 状态） | - |
| 0xFF26 | ENU 速度 | - |
| 0xFF2F | UTC 时间 | - |

## CAN J1939 修改波特率

示例：将设备从 500k 改为 1M。

确保 CAN 接口当前配置为设备能通信的波特率（例如 500k），然后发送以下指令：

**步骤 1 — 修改波特率为 1M：**

- ID: `0CEF0800`（发送给 08，来自 00）
- 数据: `9A 00 06 00 00 00 00 00`（地址 009A，写命令 06，数据 0 = 1000K）

```bash
cansend can4 0CEF0800#9A00060000000000
```

**步骤 2 — 保存配置：**

- 数据: `00 00 06 00 00 00 00 00`（地址 0000，写命令 06，数据 0 = 保存）

```bash
cansend can4 0CEF0800#0000060000000000
```

**步骤 3 — 复位设备：**

- 数据: `00 00 06 00 FF 00 00 00`（地址 0000，写命令 06，数据 FF = 复位）

```bash
cansend can4 0CEF0800#00000600FF000000
```

## 许可证

Apache License 2.0
