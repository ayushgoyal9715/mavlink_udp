#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <poll.h>
#include "mavlink/common/mavlink.h"


class MavlinkNode : public rclcpp::Node {
public:
    MavlinkNode(const std::string& serial_port) : Node("mavlink_serial_node") {
        // Open UART
        uart_fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY);
        if (uart_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", serial_port.c_str(), strerror(errno));
            return;
        }

        // Configure UART
        struct termios tty;
        if (tcgetattr(uart_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
            close(uart_fd_);
            uart_fd_ = -1;
            return;
        }
        tty.c_cflag &= ~PARENB;  // No parity
        tty.c_cflag &= ~CSTOPB;  // One stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      // 8 bits
        tty.c_cflag &= ~CRTSCTS; // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        tty.c_cc[VTIME] = 10; // 1 second timeout
        tty.c_cc[VMIN] = 0;
        cfsetispeed(&tty, B115200); // 115200 baud (common for Pixhawk)
        cfsetospeed(&tty, B115200);
        if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            close(uart_fd_);
            uart_fd_ = -1;
            return;
        }

        // Set up pollfd for UART
        pfd_.fd = uart_fd_;
        pfd_.events = POLLIN;

        // Set up timer to send HEARTBEAT
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), // 1 Hz
            std::bind(&MavlinkNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Connected to ArduPilot on %s, sending HEARTBEAT", serial_port.c_str());
    }

    ~MavlinkNode() {
        if (uart_fd_ >= 0) {
            close(uart_fd_);
        }
    }

    void run() {
        while (rclcpp::ok()) {
            int retval = poll(&pfd_, 1, 5000); // 5-second timeout
            if (retval > 0 && (pfd_.revents & POLLIN)) {
                unsigned char buf[1024];
                ssize_t avail = read(uart_fd_, buf, sizeof(buf));
                if (avail > 0) {
                    mavlink_message_t msg;
                    mavlink_status_t status = {0};
                    for (int i = 0; i < avail; i++) {
                        if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                                mavlink_heartbeat_t hb;
                                mavlink_msg_heartbeat_decode(&msg, &hb);
                                RCLCPP_INFO(this->get_logger(), "Received HEARTBEAT from sysid %d, type %d, autopilot %d, mode %d, status %d",
                                            msg.sysid, hb.type, hb.autopilot, hb.base_mode, hb.system_status);
                                mav_sysid_ = msg.sysid; // Store system ID
                            }
                        }
                    }
                }
            } else if (retval < 0) {
                RCLCPP_ERROR(this->get_logger(), "Poll error: %s", strerror(errno));
            }
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

private:
    void timerCallback() {
        if (uart_fd_ < 0) return;

        // Send MAVLink HEARTBEAT
        mavlink_message_t msg;
        unsigned char buf[1024];
        unsigned int len;

        mavlink_msg_heartbeat_pack(
            mav_sysid_, MY_COMP_ID, &msg,
            MAV_TYPE_ONBOARD_CONTROLLER, // System type
            MAV_AUTOPILOT_INVALID,       // No autopilot
            MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, // Base mode
            0,                           // Custom mode
            MAV_STATE_ACTIVE             // System status
        );
        len = mavlink_msg_to_send_buffer(buf, &msg);
        if (write(uart_fd_, buf, len) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send HEARTBEAT: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent HEARTBEAT to ArduPilot");
        }
    }

    // Member variables
    int uart_fd_ = -1;
    struct pollfd pfd_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint8_t mav_sysid_ = 1; // Default system ID
    const uint8_t MY_COMP_ID = 191; // From original code
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Use command-line argument or default serial port
    std::string serial_port = (argc > 1) ? argv[1] : "/dev/ttyACM1";
    auto node = std::make_shared<MavlinkNode>(serial_port);

    if (node->uart_fd_ >= 0) {
        node->run();
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize node, exiting");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Shutting down");
    return 0;
}
