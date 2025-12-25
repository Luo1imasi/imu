#include "hipnuc_imu/hipnuc_imu.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

extern "C" {
#include "hipnuc_lib_package/hipnuc_can_common.h"
#include "hipnuc_lib_package/hipnuc_j1939_parser.h"
#include "hipnuc_lib_package/canopen_parser.h"
}

namespace hipnuc_driver {

// ===== CAN 驱动类 =====
class CANDriver
{
public:
    CANDriver(rclcpp::Node* node)
        : node_(node), sockfd_(-1), running_(false)
    {
        memset(&imu_data_, 0, sizeof(imu_data_));
        memset(&mag_data_, 0, sizeof(mag_data_));
        memset(&euler_data_, 0, sizeof(euler_data_));
        memset(&gps_data_, 0, sizeof(gps_data_));
    }

    ~CANDriver()
    {
        stop();
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

    bool initialize(const std::string& interface, const std::string& frame_id)
    {
        frame_id_ = frame_id;
        
        sockfd_ = can_open_socket_internal(interface.c_str());
        if (sockfd_ < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open CAN interface: %s", interface.c_str());
            return false;
        }
        
        RCLCPP_INFO(node_->get_logger(), "CAN interface opened: %s (socket: %d)", interface.c_str(), sockfd_);
        return true;
    }

    void start()
    {
        if (sockfd_ < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot start: CAN socket not initialized");
            return;
        }
        
        running_ = true;
        receive_thread_ = std::thread(&CANDriver::receiveThread, this);
        RCLCPP_INFO(node_->get_logger(), "CAN receive thread started");
    }

    void stop()
    {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

    // ===== 数据获取接口 =====
    bool getIMUData(sensor_msgs::msg::Imu& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!imu_data_.valid) return false;
        
        msg.header.frame_id = frame_id_;
        msg.header.stamp = node_->now();
        
        msg.orientation.w = imu_data_.quat[0];
        msg.orientation.x = imu_data_.quat[1];
        msg.orientation.y = imu_data_.quat[2];
        msg.orientation.z = imu_data_.quat[3];
        
        msg.angular_velocity.x = imu_data_.gyr[0] * DEG_TO_RAD;
        msg.angular_velocity.y = imu_data_.gyr[1] * DEG_TO_RAD;
        msg.angular_velocity.z = imu_data_.gyr[2] * DEG_TO_RAD;
        
        msg.linear_acceleration.x = imu_data_.acc[0] * GRA_ACC;
        msg.linear_acceleration.y = imu_data_.acc[1] * GRA_ACC;
        msg.linear_acceleration.z = imu_data_.acc[2] * GRA_ACC;
        
        return true;
    }

    bool getMagData(sensor_msgs::msg::MagneticField& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!mag_data_.valid) return false;
        
        msg.header.frame_id = frame_id_;
        msg.header.stamp = node_->now();
        msg.magnetic_field.x = mag_data_.mag[0];
        msg.magnetic_field.y = mag_data_.mag[1];
        msg.magnetic_field.z = mag_data_.mag[2];
        
        return true;
    }

    bool getEulerData(geometry_msgs::msg::Vector3Stamped& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!euler_data_.valid) return false;
        
        msg.header.frame_id = frame_id_;
        msg.header.stamp = node_->now();
        msg.vector.x = euler_data_.euler[0];
        msg.vector.y = euler_data_.euler[1];
        msg.vector.z = euler_data_.euler[2];
        
        return true;
    }

    bool getTemperatureData(sensor_msgs::msg::Temperature& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!imu_data_.valid) return false;
        
        msg.header.frame_id = frame_id_;
        msg.header.stamp = node_->now();
        msg.temperature = imu_data_.temperature;
        
        return true;
    }

    bool getPressureData(sensor_msgs::msg::FluidPressure& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!imu_data_.valid) return false;
        
        msg.header.frame_id = frame_id_;
        msg.header.stamp = node_->now();
        msg.fluid_pressure = imu_data_.pressure;
        
        return true;
    }

    bool getGPSData(sensor_msgs::msg::NavSatFix& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!gps_data_.valid) return false;
        
        msg.header.frame_id = frame_id_;
        msg.header.stamp = node_->now();
        msg.latitude = gps_data_.lat;
        msg.longitude = gps_data_.lon;
        msg.altitude = gps_data_.alt;
        
        return true;
    }

    bool getVelocityData(geometry_msgs::msg::TwistStamped& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!gps_data_.valid) return false;
        
        msg.header.frame_id = frame_id_;
        msg.header.stamp = node_->now();
        msg.twist.linear.x = gps_data_.vel_n;
        msg.twist.linear.y = gps_data_.vel_e;
        msg.twist.linear.z = gps_data_.vel_d;
        
        return true;
    }

private:
    void receiveThread()
    {
        pthread_setname_np(pthread_self(), "can_rx");
        
        struct can_frame frame;
        struct timespec ts;
        
        while (running_ && rclcpp::ok()) {
            int nbytes = can_receive_frame_ts_internal(sockfd_, &frame, &ts);
            if (nbytes < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(node_->get_logger(), "CAN receive error");
                break;
            }
            
            if (nbytes == 0) continue;
            
            // 解析 CAN 帧
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            // 尝试 J1939 解析
            if (hipnuc_j1939_parse_frame(&frame, &imu_data_, &mag_data_, &euler_data_, &gps_data_) == 0) {
                continue;
            }
            
            // 尝试 CANopen 解析
            if (canopen_parse_frame(&frame, &imu_data_, &mag_data_, &euler_data_, &gps_data_) == 0) {
                continue;
            }
        }
    }

    // ===== SocketCAN 底层函数 =====
    int can_open_socket_internal(const char* ifname)
    {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create CAN socket");
            return -1;
        }

        struct ifreq ifr;
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get interface index for %s", ifname);
            close(s);
            return -1;
        }

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to bind CAN socket");
            close(s);
            return -1;
        }

        // 启用硬件时间戳（可选）
        int enable = 1;
        setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable));

        return s;
    }

    int can_receive_frame_ts_internal(int sockfd, struct can_frame* frame, struct timespec* ts)
    {
        struct iovec iov;
        struct msghdr msg;
        char ctrlmsg[CMSG_SPACE(sizeof(struct timeval))];

        iov.iov_base = frame;
        iov.iov_len = sizeof(*frame);

        memset(&msg, 0, sizeof(msg));
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = ctrlmsg;
        msg.msg_controllen = sizeof(ctrlmsg);

        int nbytes = recvmsg(sockfd, &msg, 0);
        if (nbytes < 0) {
            return -1;
        }

        // 提取时间戳
        struct cmsghdr* cmsg;
        for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
                struct timeval* tv = (struct timeval*)CMSG_DATA(cmsg);
                ts->tv_sec = tv->tv_sec;
                ts->tv_nsec = tv->tv_usec * 1000;
                break;
            }
        }

        return nbytes;
    }

    // ===== 数据结构 =====
    struct IMUData {
        bool valid = false;
        float quat[4];
        float acc[3];
        float gyr[3];
        float temperature;
        float pressure;
    };

    struct MagData {
        bool valid = false;
        float mag[3];
    };

    struct EulerData {
        bool valid = false;
        float euler[3];
    };

    struct GPSData {
        bool valid = false;
        double lat, lon, alt;
        float vel_n, vel_e, vel_d;
    };

    // ===== 成员变量 =====
    rclcpp::Node* node_;
    int sockfd_;
    std::atomic<bool> running_;
    std::thread receive_thread_;
    std::string frame_id_;
    
    std::mutex data_mutex_;
    IMUData imu_data_;
    MagData mag_data_;
    EulerData euler_data_;
    GPSData gps_data_;
};

} // namespace hipnuc_driver
