#include "hipnuc_imu/hipnuc_imu.h"
#include <memory>
#include <string>

// 前置声明驱动类
namespace hipnuc_driver {
    class SerialDriver;
    class CANDriver;
}

// 主节点类
class HipnucIMUNode : public rclcpp::Node
{
public:
    HipnucIMUNode() : Node("hipnuc_imu_node")
    {
        // ===== 声明参数 =====
        this->declare_parameter<std::string>("protocol", "serial");  // "serial" 或 "can"
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<std::string>("imu_topic", "/IMU_data");
        this->declare_parameter<int>("publish_rate", 100);

        // ===== 获取参数 =====
        protocol_ = this->get_parameter("protocol").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        int publish_rate = this->get_parameter("publish_rate").as_int();

        RCLCPP_INFO(this->get_logger(), "========== HiPNUC IMU Node ==========");
        RCLCPP_INFO(this->get_logger(), "IMU Topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish Rate: %d Hz", publish_rate);

        // ===== 创建发布器 =====
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, qos);
        
        // CAN 协议才需要额外的 topic
        if (protocol_ == "can") {
            mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);
            euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/euler", qos);
            temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", qos);
            pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("imu/pressure", qos);
            gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("imu/gps/fix", qos);
            velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("imu/gps/velocity", qos);
        }
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("imu/gps/fix", qos);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("imu/gps/velocity", qos);

        // ===== 根据协议创建驱动 =====
        if (protocol_ == "serial") {
            initSerialDriver();
        } else if (protocol_ == "can") {
            initCANDriver();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown protocol: %s (must be 'serial' or 'can')", protocol_.c_str());
            throw std::runtime_error("Invalid protocol specified");
        }

        // ===== 创建发布定时器 =====
        auto publish_interval = std::chrono::milliseconds(1000 / publish_rate);
        publish_timer_ = this->create_wall_timer(
            publish_interval,
            std::bind(&HipnucIMUNode::publishData, this)
        );

        RCLCPP_INFO(this->get_logger(), "===== Node initialized successfully =====");
    }

    ~HipnucIMUNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down HiPNUC IMU Node...");
    }

private:
    // ===== 初始化串口驱动 =====
    void initSerialDriver();

    // ===== 初始化 CAN 驱动 =====
    void initCANDriver();

    // ===== 定时发布数据 =====
    void publishData();

    // ===== 成员变量 =====
    std::string protocol_;
    std::string frame_id_;

    // 驱动实例（只有一个会被创建）
    std::unique_ptr<hipnuc_driver::SerialDriver> serial_driver_;
    std::unique_ptr<hipnuc_driver::CANDriver> can_driver_;

    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

// ===== 包含驱动实现 =====
#include "serial_port.cpp"
#include "can_interface.cpp"

// ===== 实现初始化函数 =====
void HipnucIMUNode::initSerialDriver()
{
    std::string port = this->get_parameter("serial_port").as_string();
    int baudrate = this->get_parameter("baud_rate").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Initializing Serial driver: %s @ %d", port.c_str(), baudrate);
    
    serial_driver_ = std::make_unique<hipnuc_driver::SerialDriver>(this);
    
    if (!serial_driver_->initialize(port, baudrate, frame_id_)) {
        throw std::runtime_error("Failed to initialize serial driver");
    }
    
    serial_driver_->start();
}

void HipnucIMUNode::initCANDriver()
{
    std::string interface = this->get_parameter("can_interface").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Initializing CAN driver: %s", interface.c_str());
    
    can_driver_ = std::make_unique<hipnuc_driver::CANDriver>(this);
    
    if (!can_driver_->initialize(interface, frame_id_)) {
        throw std::runtime_error("Failed to initialize CAN driver");
    }
    
    can_driver_->start();
    RCLCPP_INFO(this->get_logger(), "CAN driver started successfully");
}

void HipnucIMUNode::publishData()
{
    if (protocol_ == "serial" && serial_driver_) {
        auto imu_data = serial_driver_->getData();
        if (imu_data) {
            imu_pub_->publish(*imu_data);
        }
    } 
    else if (protocol_ == "can" && can_driver_) {
        // 发布 IMU 数据
        sensor_msgs::msg::Imu imu_msg;
        if (can_driver_->getIMUData(imu_msg)) {
            imu_pub_->publish(imu_msg);
        }
        
        // 发布磁力计数据
        sensor_msgs::msg::MagneticField mag_msg;
        if (can_driver_->getMagData(mag_msg)) {
            mag_pub_->publish(mag_msg);
        }
        
        // 发布欧拉角
        geometry_msgs::msg::Vector3Stamped euler_msg;
        if (can_driver_->getEulerData(euler_msg)) {
            euler_pub_->publish(euler_msg);
        }
        
        // 发布温度
        sensor_msgs::msg::Temperature temp_msg;
        if (can_driver_->getTemperatureData(temp_msg)) {
            temp_pub_->publish(temp_msg);
        }
        
        // 发布气压
        sensor_msgs::msg::FluidPressure pressure_msg;
        if (can_driver_->getPressureData(pressure_msg)) {
            pressure_pub_->publish(pressure_msg);
        }
        
        // 发布 GPS 位置
        sensor_msgs::msg::NavSatFix gps_msg;
        if (can_driver_->getGPSData(gps_msg)) {
            gps_pub_->publish(gps_msg);
        }
        
        // 发布 GPS 速度
        geometry_msgs::msg::TwistStamped vel_msg;
        if (can_driver_->getVelocityData(vel_msg)) {
            velocity_pub_->publish(vel_msg);
        }
    }
}

// ===== Main 函数 =====
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<HipnucIMUNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("hipnuc_imu_node"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}

// ===== ROS2 Component 注册 =====
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(HipnucIMUNode)
