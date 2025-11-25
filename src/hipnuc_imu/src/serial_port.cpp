#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <thread>
#include <atomic>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C"{
#endif
#include <poll.h>

#include "hipnuc_lib_package/hipnuc_dec.h"

#define GRA_ACC     (9.8)
#define DEG_TO_RAD  (0.01745329)
#define BUF_SIZE    (1024)
#ifdef __cplusplus
}
#endif

using namespace std::chrono_literals;
using namespace std;
static hipnuc_raw_t raw;

namespace hipnuc_driver
{

class IMUPublisher : public rclcpp::Node
{
public:
    int fd = 0;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    IMUPublisher(const rclcpp::NodeOptions &options) : Node("imu_node", options)
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<std::string>("imu_topic", "/IMU_data");
        this->declare_parameter<int>("publish_rate", 100);

        this->get_parameter("serial_port", serial_port);
        this->get_parameter("baud_rate", baud_rate);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("imu_topic", imu_topic);
        this->get_parameter("publish_rate", publish_rate);

        RCLCPP_INFO(this->get_logger(),"serial_port: %s\r\n", serial_port.c_str());
        RCLCPP_INFO(this->get_logger(), "baud_rate: %d\r\n", baud_rate);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s\r\n", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "imu_topic: %s\r\n", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "publish_rate: %d Hz\r\n", publish_rate);
        
        write_buffer_ = std::make_shared<sensor_msgs::msg::Imu>();
        write_buffer_->header.frame_id = frame_id;
        read_buffer_ = std::make_shared<sensor_msgs::msg::Imu>();
        read_buffer_->header.frame_id = frame_id;
        
        auto sensor_data_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, sensor_data_qos);

        fd = open_serial(serial_port, baud_rate);

        if (fd > 0) {
            decode_thread_ = std::thread(&IMUPublisher::decode_thread, this);
        }

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / publish_rate),
            std::bind(&IMUPublisher::publish_data, this)
        );
    }

    ~IMUPublisher()
    {
        if (decode_thread_.joinable()) {
            decode_thread_.join();
        }
        if (fd > 0) {
            close(fd);
        }
    }

private: 
    void decode_thread(void)
    {
        pthread_setname_np(pthread_self(), "serial_rx");
        struct sched_param sp{}; 
        sp.sched_priority = 60;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        
        uint8_t buf[BUF_SIZE] = {0};
        
        while(rclcpp::ok())
        {
            int total_read = 0;
            int ret;

            // Setup for select() timeout
            struct timeval tv;
            fd_set readfds;
            int timeout_ms = 1;
            
            while (total_read < BUF_SIZE)
            {
                // Reset select() parameters for each iteration
                FD_ZERO(&readfds);
                FD_SET(fd, &readfds);

                // Configure timeout for this iteration
                tv.tv_sec = timeout_ms / 1000;
                tv.tv_usec = (timeout_ms % 1000) * 1000;

                // Wait for data or timeout
                ret = select(fd + 1, &readfds, NULL, NULL, &tv);

                if (ret < 0)
                {
                    // Handle interruption by signal
                    if (errno == EINTR)
                        continue;
                    perror("select");
                    return;
                }
                else if (ret == 0)
                {
                    // No data received within timeout period
                    break;
                }

                // Data is available, read it
                ret = read(fd, buf + total_read, BUF_SIZE - total_read);
                if (ret < 0)
                {
                    // Handle non-blocking operations
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                        continue;
                    perror("read");
                    return;
                }
                else if (ret == 0)
                {
                    // Port closed or disconnected
                    break;
                }

                // Update total bytes read
                total_read += ret;
            }
            
            if(total_read > 0)
            {
                for (int i = 0; i < total_read; i++) {
                    if (hipnuc_input(&raw, buf[i])) {
                        write_buffer_->orientation.w = raw.hi91.quat[0];
                        write_buffer_->orientation.x = raw.hi91.quat[1];
                        write_buffer_->orientation.y = raw.hi91.quat[2];
                        write_buffer_->orientation.z = raw.hi91.quat[3];
                        write_buffer_->angular_velocity.x = raw.hi91.gyr[0] * DEG_TO_RAD;
                        write_buffer_->angular_velocity.y = raw.hi91.gyr[1] * DEG_TO_RAD;
                        write_buffer_->angular_velocity.z = raw.hi91.gyr[2] * DEG_TO_RAD;
                        write_buffer_->linear_acceleration.x = raw.hi91.acc[0] * GRA_ACC;
                        write_buffer_->linear_acceleration.y = raw.hi91.acc[1] * GRA_ACC;
                        write_buffer_->linear_acceleration.z = raw.hi91.acc[2] * GRA_ACC;
                        write_buffer_->header.stamp = rclcpp::Clock().now();
                        write_buffer_ = std::atomic_exchange(&read_buffer_, write_buffer_);
                    }
                }
                memset(buf, 0, sizeof(buf));
            }
        }
    }

    void publish_data(void)
    {
        auto data = std::atomic_load(&read_buffer_);
        imu_pub->publish(*data);
    }

    int open_serial(std::string port, int baud) {
        const char* port_device = port.c_str();
        int fd = open(port_device, O_RDWR | O_NOCTTY | O_NDELAY);

        if (fd < 0) {
            perror("unable to open serial port");
            return -1;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd, &tty) != 0)
        {
            perror("Error from tcgetattr");
            close(fd);
            return -1;
        }

        // Try to set custom baud rate using termios2 if available
        #ifdef __linux__
        #include <asm/termbits.h>
        struct termios2 tty2;
        if (ioctl(fd, TCGETS2, &tty2) == 0)
        {
            tty2.c_cflag &= ~CBAUD;
            tty2.c_cflag |= BOTHER;
            tty2.c_ispeed = baud;
            tty2.c_ospeed = baud;

            // Configure other port settings
            tty2.c_cflag &= ~CSIZE;
            tty2.c_cflag |= CS8;      // 8 data bits
            tty2.c_cflag &= ~PARENB;  // No parity
            tty2.c_cflag &= ~CSTOPB;  // 1 stop bit
            
            tty2.c_cflag |= (CLOCAL | CREAD);
            tty2.c_cflag &= ~CRTSCTS;
        
            tty2.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
            tty2.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty2.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
            tty2.c_oflag &= ~OPOST;
        
            tty2.c_cc[VMIN] = 0;
            tty2.c_cc[VTIME] = 0;
        
            if (ioctl(fd, TCSETS2, &tty2) != 0)
            {
                perror("Error from TCSETS2 IOCTL");
                close(fd);
                return -1;
            }
        }
        else
        #endif
        {
            // Fallback to standard termios
            speed_t speed;
            switch(baud) {
                case 9600: speed = B9600; break;
                case 19200: speed = B19200; break;
                case 38400: speed = B38400; break;
                case 57600: speed = B57600; break;
                case 115200: speed = B115200; break;
                case 230400: speed = B230400; break;
                case 460800: speed = B460800; break;
                case 921600: speed = B921600; break;
                default:
                    fprintf(stderr, "Unsupported baud rate: %d\n", baud);
                    close(fd);
                    return -1;
            }

            cfsetispeed(&tty, speed);
            cfsetospeed(&tty, speed);

            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~CRTSCTS;

            tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

            tty.c_oflag &= ~OPOST;

            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 0;

            if (tcsetattr(fd, TCSANOW, &tty) != 0)
            {
                perror("Error from tcsetattr");
                close(fd);
                return -1;
            }
        }

        tcflush(fd, TCIOFLUSH);
        return fd;
    }

    std::string serial_port;
    int baud_rate;
    int publish_rate;
    std::string frame_id;
    std::string imu_topic;
    std::shared_ptr<sensor_msgs::msg::Imu> write_buffer_, read_buffer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    std::thread decode_thread_;
};

} // namespace hipnuc_driver

int main(int argc, const char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::spin(std::make_shared<hipnuc_driver::IMUPublisher>(options));
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hipnuc_driver::IMUPublisher)