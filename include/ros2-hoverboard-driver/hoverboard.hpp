#include "rclcpp/rclcpp.hpp"
#include "wheel_msgs/msg/wheel_speeds.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "config.hpp"
#include "protocol.hpp"

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
    void protocol_recv (uint8_t c); // Function to recontruct serial packets coming from BLDC controller

    // ROS2 Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;

    // ROS2 Suscriber
    // TODO : this msg sends setpoints for 4 wheels, but we can control only 2 wheels
    rclcpp::Subscription<wheel_msgs::msg::WheelSpeeds>::SharedPtr speeds_sub_;
    // Callback for subscriber
    void setpoint_callback(wheel_msgs::msg::WheelSpeeds::UniquePtr msg); // Be careful with UniquePtr, in speeds_sub_ definition we remove it... TODO :  understand what UniquePtr is doing!

    // Hoverboard protocol variables
    int port_fd;
    unsigned int msg_len = 0;
    uint8_t prev_byte = 0; // uint8_t is nice to store bytes
    uint16_t start_frame = 0;
    uint8_t* p;
    SerialFeedback msg, prev_msg;

    // Other variables
    float setpoint[2] = {0,0}; // Right , Left
};
