#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <sys/wait.h>
#include <poll.h>
#include <time.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "mavlink/common/mavlink.h"


#define MY_COMP_ID 191
#define MY_NUM_PFDS 2 // UART and ipc_fd2
#define SERVER_PATH2 "/tmp/chobits_server2"

class MavlinkNode : public rclcpp::Node {
public:
    MavlinkNode() : Node("mavlink_udp") {
        // Subscribe to /odom topic
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MavlinkNode::odomCallback, this, std::placeholders::_1));
        
        // Create publisher for /chobits/odometry
        odo_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/chobits/odometry", 10);

        RCLCPP_INFO(this->get_logger(), "mavlink_udp ready");
    }

   void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        // Extract position, orientation, and velocity
        latest_vins_alt_ = msg->pose.pose.position.z;
        att_q_x_ = msg->pose.pose.orientation.x;
        att_q_y_ = msg->pose.pose.orientation.y;
        att_q_z_ = msg->pose.pose.orientation.z;
        att_q_w_ = msg->pose.pose.orientation.w;
        xacc_ = msg->twist.twist.angular.x;
        yacc_ = msg->twist.twist.angular.y;
        zacc_ = msg->twist.twist.angular.z;

        // Prepare pose array for MAVLink
        float pose[10];
        pose[0] = msg->pose.pose.position.x;  // x
        pose[1] = msg->pose.pose.position.y;  // y
        pose[2] = msg->pose.pose.position.z;  // z
        pose[3] = latest_vins_alt_;           // altitude
        pose[4] = att_q_x_;                   // qx
        pose[5] = att_q_y_;                   // qy
        pose[6] = att_q_z_;                   // qz
        pose[7] = msg->twist.twist.linear.x;  // vx
        pose[8] = msg->twist.twist.linear.y;  // vy
        pose[9] = msg->twist.twist.linear.z;  // vz

        // Coordinate transformations
        pose[2] = -pose[2];
        pose[3] = -pose[3];

        // Send ATT_POS_MOCAP
        float covar[21] = {0};
        mavlink_message_t mav_msg;
        unsigned char buf[1024];
        unsigned int len;
        struct timeval tv;
        gettimeofday(&tv, NULL);
        mavlink_msg_att_pos_mocap_pack(mav_sysid_, MY_COMP_ID, &mav_msg, tv.tv_sec*1000000+tv.tv_usec, pose, pose[4], -pose[5], -pose[6], covar);
        len = mavlink_msg_to_send_buffer(buf, &mav_msg);
        write(uart_fd_, buf, len);

        // Send VISION_SPEED_ESTIMATE
        mavlink_msg_vision_speed_estimate_pack(mav_sysid_, MY_COMP_ID, &mav_msg, tv.tv_sec*1000000+tv.tv_usec, pose[7], -pose[8], -pose[9], covar, 0);
        len = mavlink_msg_to_send_buffer(buf, &mav_msg);
        write(uart_fd_, buf, len);
    }

    // Public members
    int uart_fd_;
    uint8_t mav_sysid_ = 0;
    int64_t time_offset_us_ = 0;
    bool no_local_pos_ = true;
    int parse_error_ = 0, packet_rx_drop_count_ = 0;
    int64_t tc1_sent_ = 0;
    float vins_apm_alt_diff_ = 0;
    float latest_vins_alt_ = 0;
    float att_q_x_ = 0, att_q_y_ = 0, att_q_z_ = 0, att_q_w_ = 0;
    float xacc_ = 0, yacc_ = 0, zacc_ = 0;

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkNode>();

    struct pollfd pfds[MY_NUM_PFDS];
    struct timeval tv;
    int retval;
    unsigned int len;
    unsigned char buf[1024];
    ssize_t avail;
    mavlink_status_t status;
    mavlink_message_t msg;
    struct termios tty;
    struct sockaddr_un ipc_addr2;
    int ipc_fd2;

    // Open UART
    if (argc > 1)
        node->uart_fd_ = open(argv[1], O_RDWR | O_NOCTTY);
    else
        node->uart_fd_ = open("/dev/ttyACM1", O_RDWR | O_NOCTTY);
    if (node->uart_fd_ < 0) {
        printf("can not open serial port\n");
        return 1;
    }

    // Configure UART
    if (tcgetattr(node->uart_fd_, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
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
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B1500000);
    cfsetospeed(&tty, B1500000);
    if (tcsetattr(node->uart_fd_, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    // Setup second Unix socket
    if ((ipc_fd2 = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr2, 0, sizeof(ipc_addr2));
    ipc_addr2.sun_family = AF_UNIX;
    strcpy(ipc_addr2.sun_path, SERVER_PATH2);
    unlink(SERVER_PATH2);
    if (bind(ipc_fd2, (const struct sockaddr *)&ipc_addr2, sizeof(ipc_addr2)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    // Setup pollfd
    pfds[0].fd = node->uart_fd_;
    pfds[0].events = POLLIN;
    pfds[1].fd = ipc_fd2;
    pfds[1].events = POLLIN;

    printf("hello\n");

    memset(&status, 0, sizeof(status));

    while (rclcpp::ok()) {
        retval = poll(pfds, MY_NUM_PFDS, 5000);
        if (retval > 0) {
            // UART handling
            if (pfds[0].revents & POLLIN) {
                avail = read(node->uart_fd_, buf, 1024);
                for (int i = 0; i < avail; i++) {
                    if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                        if (node->parse_error_ != status.parse_error) {
                            node->parse_error_ = status.parse_error;
                            printf("mavlink parse_error %d\n", node->parse_error_);
                        }
                        if (node->packet_rx_drop_count_ != status.packet_rx_drop_count) {
                            node->packet_rx_drop_count_ = status.packet_rx_drop_count;
                            printf("mavlink drop %d\n", node->packet_rx_drop_count_);
                        }
                        if (msg.sysid == 255) continue;
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            if (msg.sysid != node->mav_sysid_) {
                                node->mav_sysid_ = msg.sysid;
                                printf("found MAV %d\n", node->mav_sysid_);
                            }
                            if (node->time_offset_us_ == 0) {
                                gettimeofday(&tv, NULL);
                                node->tc1_sent_ = tv.tv_sec * 1000000000 + tv.tv_usec * 1000;
                                mavlink_msg_timesync_pack(node->mav_sysid_, MY_COMP_ID, &msg, 0, node->tc1_sent_, node->mav_sysid_, 1);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(node->uart_fd_, buf, len);

                                mavlink_msg_system_time_pack(node->mav_sysid_, MY_COMP_ID, &msg, tv.tv_sec * 1000000 + tv.tv_usec, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(node->uart_fd_, buf, len);

                                mavlink_msg_set_gps_global_origin_pack(node->mav_sysid_, MY_COMP_ID, &msg, node->mav_sysid_, 247749434, 1210443077, 100000, tv.tv_sec * 1000000 + tv.tv_usec);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(node->uart_fd_, buf, len);
                            }
                            if (node->no_local_pos_) {
                                mavlink_msg_command_long_pack(node->mav_sysid_, MY_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(node->uart_fd_, buf, len);
                            }
                        } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
                            mavlink_timesync_t ts;
                            mavlink_msg_timesync_decode(&msg, &ts);
                            if (ts.ts1 == node->tc1_sent_) {
                                node->time_offset_us_ = (ts.ts1 - ts.tc1) / 1000;
                                printf("time offset %ld\n", node->time_offset_us_);
                            }
                        } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                            mavlink_statustext_t txt;
                            mavlink_msg_statustext_decode(&msg, &txt);
                            printf("fc: %s\n", txt.text);
                        } else if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                            node->no_local_pos_ = false;
                            mavlink_local_position_ned_t local_pos;
                            mavlink_msg_local_position_ned_decode(&msg, &local_pos);

                            if (node->vins_apm_alt_diff_ == 0) {
                                node->vins_apm_alt_diff_ = node->latest_vins_alt_ - (-local_pos.z);
                                printf("alt: vins, apm, diff %f %f %f\n", node->latest_vins_alt_, -local_pos.z, node->vins_apm_alt_diff_);
                            }

                            auto odo = nav_msgs::msg::Odometry();
                            odo.header.stamp = rclcpp::Clock().now();
                            odo.header.frame_id = "world";
                            odo.child_frame_id = "world";
                            odo.pose.pose.position.x = local_pos.x;
                            odo.pose.pose.position.y = -local_pos.y;
                            odo.pose.pose.position.z = -local_pos.z + node->vins_apm_alt_diff_;
                            odo.pose.pose.orientation.x = node->att_q_x_;
                            odo.pose.pose.orientation.y = -node->att_q_y_;
                            odo.pose.pose.orientation.z = -node->att_q_z_;
                            odo.pose.pose.orientation.w = node->att_q_w_;
                            odo.twist.twist.linear.x = local_pos.vx;
                            odo.twist.twist.linear.y = -local_pos.vy;
                            odo.twist.twist.linear.z = -local_pos.vz;
                            odo.twist.twist.angular.x = node->xacc_;
                            odo.twist.twist.angular.y = -node->yacc_;
                            odo.twist.twist.angular.z = -node->zacc_;
                           // odo_pub_->publish(odo);
                        }
                    }
                }
            }
#if 0
            // Second socket handling (unchanged)
            if (pfds[1].revents & POLLIN) {
                float planner_msg[9];
                if (recv(ipc_fd2, &planner_msg, sizeof(planner_msg), 0) > 0) {
                    if (planner_msg[3] == 0 && planner_msg[4] == 0 && planner_msg[5] == 0) {
                        if (demo_stage == 3) {
                            demo_stage = 100;
                            gettimeofday(&tv, NULL);
                            mavlink_msg_set_mode_pack(node->mav_sysid_, MY_COMP_ID, &msg, node->mav_sysid_, 1, 9);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(node->uart_fd_, buf, len);
                        }
                    } else {
                        gettimeofday(&tv, NULL);
                        mavlink_msg_set_position_target_local_ned_pack(node->mav_sysid_, MY_COMP_ID, &msg, tv.tv_sec * 1000 + tv.tv_usec * 0.001, 0, 0, MAV_FRAME_LOCAL_NED, 0xc00, planner_msg[0], -planner_msg[1], -(planner_msg[2] - node->vins_apm_alt_diff_), planner_msg[3], -planner_msg[4], -planner_msg[5], planner_msg[6], -planner_msg[7], -planner_msg[8], 0, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(node->uart_fd_, buf, len);
                    }
                }
            }
#endif
        }
        rclcpp::spin_some(node);
    }

    close(node->uart_fd_);
    close(ipc_fd2);

    printf("bye\n");
    rclcpp::shutdown();
    return 0;
}
