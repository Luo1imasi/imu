#ifndef HIPNUC_IMU_H
#define HIPNUC_IMU_H

// ROS2 核心
#include "rclcpp/rclcpp.hpp"

// ROS2 消息类型
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// HiPNUC 协议解析库
extern "C" {
#include "hipnuc_lib_package/hipnuc_dec.h"
#include "hipnuc_lib_package/nmea_decode.h"
#include "hipnuc_lib_package/hipnuc_can_common.h"
#include "hipnuc_lib_package/hipnuc_j1939_parser.h"
#include "hipnuc_lib_package/canopen_parser.h"
}

// 标准库
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

// 常量定义
#define GRA_ACC     (9.8)
#define DEG_TO_RAD  (0.01745329)

#endif // HIPNUC_IMU_H
