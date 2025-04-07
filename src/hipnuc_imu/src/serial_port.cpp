#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

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

static const struct {
    int rate;
    speed_t constant;
} baud_map[] = {
    {4800, B4800}, {9600, B9600}, {19200, B19200}, {38400, B38400},
    {57600, B57600}, {115200, B115200}, {230400, B230400}, {460800, B460800}, {921600, B921600},
    {0, B0}  // Sentinel
};

class IMUPublisher : public rclcpp::Node
{
	public:
		int fd = 0;
		uint8_t buf[BUF_SIZE] = {0};
		rclcpp::TimerBase::SharedPtr timer_;
		IMUPublisher() : Node("IMU_publisher")	
		{
			this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
			this->declare_parameter<int>("baud_rate", 460800);
			this->declare_parameter<std::string>("frame_id", "base_link");
			this->declare_parameter<std::string>("imu_topic", "/IMU_data");

			this->get_parameter("serial_port", serial_port);
			this->get_parameter("baud_rate", baud_rate);
			this->get_parameter("frame_id", frame_id);
			this->get_parameter("imu_topic", imu_topic);

			RCLCPP_INFO(this->get_logger(),"serial_port: %s\r\n", serial_port.c_str());
			RCLCPP_INFO(this->get_logger(), "baud_rate: %d\r\n", baud_rate);
			RCLCPP_INFO(this->get_logger(), "frame_id: %s\r\n", frame_id.c_str());
			RCLCPP_INFO(this->get_logger(), "imu_topic: %s\r\n", imu_topic.c_str());
			
			imu_data.header.frame_id = frame_id;
			imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);

			fd = open_serial(serial_port, baud_rate);

			timer_ = this->create_wall_timer(
			    std::chrono::milliseconds(1), // 1000 Hz
			    std::bind(&IMUPublisher::imu_read, this)
			);
		}

	private: 
		void imu_read(void)
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
    		        // This means the line has been idle for timeout_ms
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
			
			for(int i = 0; i < total_read; i++)
			{
				if(hipnuc_input(&raw, buf[i])){
					imu_data.orientation.w = raw.hi91.quat[0];
					imu_data.orientation.x = raw.hi91.quat[1];	
					imu_data.orientation.y = raw.hi91.quat[2];
					imu_data.orientation.z = raw.hi91.quat[3];
					imu_data.angular_velocity.x = raw.hi91.gyr[0] * DEG_TO_RAD;
					imu_data.angular_velocity.y = raw.hi91.gyr[1] * DEG_TO_RAD;
					imu_data.angular_velocity.z = raw.hi91.gyr[2] * DEG_TO_RAD;
					imu_data.linear_acceleration.x = raw.hi91.acc[0] * GRA_ACC;
					imu_data.linear_acceleration.y = raw.hi91.acc[1] * GRA_ACC;
					imu_data.linear_acceleration.z = raw.hi91.acc[2] * GRA_ACC;

					imu_data.header.stamp = rclcpp::Clock().now();
					
					imu_pub->publish(imu_data);
				}
			}
			memset(buf,0,sizeof(buf));
		}

		int open_serial(std::string port, int baud)
		{
			const char* port_device = port.c_str();
			int fd = open(port_device, O_RDWR | O_NOCTTY | O_NDELAY);
		
			if(fd == -1)
			{
				perror("unable to open serial port");
				exit(0);
			}

			struct termios options;
			memset(&options, 0, sizeof(options));
			tcgetattr(fd, &options);
	
			// Set baud rate
    		speed_t baud_constant = B0;
    		for (int i = 0; baud_map[i].rate != 0; i++) {
    		    if (baud_map[i].rate == baud) {
    		        baud_constant = baud_map[i].constant;
    		        break;
    		    }
    		}

    		if (baud_constant == B0) {
    		    fprintf(stderr, "Unsupported baud rate: %d\n", baud);
    		    return -1;
    		}

    		if (cfsetispeed(&options, baud_constant) < 0 || 
    		    cfsetospeed(&options, baud_constant) < 0) {
    		    perror("Error setting baud rate");
    		    return -1;
    		}

			 // Configure other port settings
    		options.c_cflag &= ~PARENB;  // No parity
    		options.c_cflag &= ~CSTOPB;  // 1 stop bit
    		options.c_cflag &= ~CSIZE;
    		options.c_cflag |= CS8;      // 8 data bits
    		options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem control lines
		
    		// Disable hardware flow control
    		options.c_cflag &= ~CRTSCTS;
		
    		// Set input mode (non-canonical, no echo,...)
    		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    		options.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
    		options.c_iflag &= ~(INLCR | ICRNL);  // Disable newline & carriage return translation
		
    		// Set output mode (raw output)
    		options.c_oflag &= ~OPOST;
		
    		// Set read timeout and minimum character count
    		options.c_cc[VMIN] = 0;  // Minimum number of characters
    		options.c_cc[VTIME] = 0;  // Timeout in deciseconds
		
    		// Apply the new settings
    		if (tcsetattr(fd, TCSANOW, &options) != 0) {
    		    perror("Error setting port attributes");
    		    return -1;
    		}
		
    		// Flush the buffer
    		tcflush(fd, TCIOFLUSH);

			return fd;
		}


		std::string serial_port;
		int baud_rate;
		std::string frame_id;
		std::string imu_topic;
		sensor_msgs::msg::Imu imu_data = sensor_msgs::msg::Imu();
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
};


int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IMUPublisher>());
	rclcpp::shutdown();

	return 0;
}
