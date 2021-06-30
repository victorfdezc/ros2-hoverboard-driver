#include "rclcpp/rclcpp.hpp"
#include "wheel_msgs/msg/wheel_speeds.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "config.h"
#include "protocol.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <functional>
#include <memory>

using namespace std::chrono_literals;

class Hoverboard : public rclcpp::Node {
public:
    Hoverboard(); //Constructor
    ~Hoverboard(); //Destructor
    
    void read(); // Function to read data coming from BLDC controller
    void write(); // Function to send comand references to the BLDC controller
 
 private:
    void protocol_recv (char c); // Function to recontruct serial packets coming from BLDC controller
    void setpoint_callback(const wheel_msgs::msg::WheelSpeeds msg); // Callback for subscriber

    // ROS2 Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;

    // ROS2 Suscriber
    // TODO : this msg sends setpoints for 4 wheels, but we can control only 2 wheels
    rclcpp::Subscription<wheel_msgs::msg::WheelSpeeds>::SharedPtr speeds_sub_;

    void callback(std_msgs::msg::String::UniquePtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


    // Hoverboard protocol variables
    int port_fd;
    int msg_len = 0;
    char prev_byte = 0;
    uint16_t start_frame = 0;
    char* p;
    SerialFeedback msg, prev_msg;

    // Other variables
    double setpoint[2] = {0,0};
};
