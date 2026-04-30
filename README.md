# roboto_imu

IMU driver library for the Roboto platform with C++ and Python SDK support. Currently supports **HiPNUC** series IMU via CAN (J1939 / CANopen) and Serial (UART) interfaces.

## Architecture

```
roboto_imu/
├── include/
│   └── imu_driver.hpp          # Abstract base class with factory pattern
├── src/
│   ├── imu_driver.cpp          # Factory: create_imu()
│   ├── pybind_module.cpp       # Python bindings (imu_py)
│   ├── drivers/
│   │   └── hipnuc/             # HiPNUC driver + C protocol decoders
│   │       ├── hipnuc_imu_driver.hpp / .cpp
│   │       ├── hipnuc_dec.h / .c        # HiPNUC proprietary protocol (HI91/HI92/HI81/HI83)
│   │       ├── hipnuc_j1939_parser.h / .c  # J1939 PGN parser
│   │       ├── hipnuc_can_common.h      # CAN data structures
│   │       ├── canopen_parser.h / .c    # CANopen TPDO parser
│   │       └── nmea_decode.h / .c       # NMEA sentence parser
│   └── protocol/
│       ├── can/
│       │   └── socket_can.hpp / .cpp    # SocketCAN interface (singleton, SCHED_FIFO)
│       └── serial/
│           └── serial_port.hpp / .cpp   # Serial UART interface
```

## Supported Hardware

| IMU Type | Interface | Protocol | Key Features |
|----------|-----------|----------|--------------|
| HiPNUC | CAN | J1939 | Accel, Gyro, Quaternion, Euler, Mag, Temp, Pressure |
| HiPNUC | CAN | CANopen | TPDOs for Accel, Gyro, Euler, Quaternion, Pressure |
| HiPNUC | Serial | HiPNUC proprietary (HI91) | Quaternion, Gyro, Accel |

## Sensor Data

| Data | Method | Return | Unit |
|------|--------|--------|------|
| Quaternion | `get_quat()` | `[w, x, y, z]` | - |
| Angular Velocity | `get_ang_vel()` | `[x, y, z]` | rad/s |
| Linear Acceleration | `get_lin_acc()` | `[x, y, z]` | m/s² |
| Temperature | `get_temperature()` | `float` | °C |

## Build

Requires: `spdlog`, `fmt`, `pybind11`, `ament_cmake` (ROS 2)

```bash
colcon build --packages-select roboto_imu
```

## Usage

### Python

```python
from imu_py import IMUDriver

# CAN interface (J1939)
imu = IMUDriver.create_imu(
    imu_id=0x08,
    interface_type="can",
    interface="can0",
    imu_type="HIPNUC",
    baudrate=0          # baudrate is only used for serial
)

# Serial interface
imu = IMUDriver.create_imu(
    imu_id=0x01,
    interface_type="serial",
    interface="/dev/ttyUSB0",
    imu_type="HIPNUC",
    baudrate=115200
)

# Read sensor data
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

## CAN J1939 PGN Table

| PGN | Data | Scale |
|-----|------|-------|
| 0xFF34 | Acceleration (x, y, z) | 0.48828 mG/LSB |
| 0xFF37 | Gyroscope (x, y, z) | 0.061035 °/s/LSB |
| 0xFF3A | Magnetometer (x, y, z) | 0.030517 |
| 0xFF3D | Pitch & Roll | 0.001° |
| 0xFF41 | Yaw | 0.001° |
| 0xFF43 | Temperature & Pressure | - |
| 0xFF46 | Quaternion (w, x, y, z) | 0.0001 |
| 0xFF4A | Inclination | - |
| 0xFF10 | Lon / Lat | 1e-7 |
| 0xFF14 | Altitude | - |
| 0xFF18 | Status (solq, sat count, INS status) | - |
| 0xFF26 | Velocity ENU | - |
| 0xFF2F | UTC Time | - |

## CAN J1939 Baud Rate Configuration

Example: change from 500k to 1M.

Make sure the CAN interface is configured at the current device baud rate (e.g. 500k), then send:

**Step 1 — Set baud rate to 1M:**

```bash
cansend can4 0CEF0800#9A00060000000000
```

**Step 2 — Save configuration:**

```bash
cansend can4 0CEF0800#0000060000000000
```

**Step 3 — Reset device:**

```bash
cansend can4 0CEF0800#00000600FF000000
```

## License

Apache License 2.0
