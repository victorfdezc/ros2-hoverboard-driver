#include "ros2-hoverboard-driver/hoverboard.hpp"


Hoverboard::Hoverboard() 
: Node("hoverboard_driver_node") // Member initialization for ROS2 node
{
    // Constructor implementation

    // These publishers are only for debugging purposes
    vel_pub_[0]    = create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/velocity", 10);
    vel_pub_[1]    = create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/velocity", 10);
    cmd_pub_[0]    = create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/cmd", 10);
    cmd_pub_[1]    = create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/cmd", 10);
    voltage_pub_   = create_publisher<std_msgs::msg::Float64>("hoverboard/battery_voltage", 10);
    temp_pub_      = create_publisher<std_msgs::msg::Float64>("hoverboard/temperature", 10);

    // Create the subscriber to receive speed setpoints
    speeds_sub_   = create_subscription<wheel_msgs::msg::WheelSpeeds>("wheel_vel_setpoints",
                    10, std::bind(&Hoverboard::setpoint_callback, this, std::placeholders::_1));
    
    // Convert m/s to rad/s
    // TODO : take into account the way we send references to the hoverboard controller
    //max_velocity /= wheel_radius;

    if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port to hoverboard");
        //exit(-1); // TODO : put this again
    }
    
    // CONFIGURE THE UART -- connecting to the board
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    // TODO : understand this shit
    struct termios options;
    tcgetattr(port_fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);
}

Hoverboard::~Hoverboard() { // Destructor implementation
    if (port_fd != -1) 
        close(port_fd);
}

void Hoverboard::setpoint_callback(wheel_msgs::msg::WheelSpeeds::UniquePtr msg)
{
    setpoint[0] = msg->right_wheel;
    setpoint[1] = msg->left_wheel;

    RCLCPP_INFO(this->get_logger(), "I heard something: %f, %f", setpoint[0], setpoint[1]);
}

void Hoverboard::read() {
    if (port_fd != -1) {
        unsigned char c;
        int i = 0, r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
            protocol_recv(c);

        // if (i > 0)
        // last_read = ros::Time::now();

        if (r < 0 && errno != EAGAIN)
            RCLCPP_ERROR(this->get_logger(), "Reading from serial %s failed: %d", PORT, r);
    }

    // if ((ros::Time::now() - last_read).toSec() > 1) {
    //     RCLCPP_ERROR(this->get_logger(), "Timeout reading from serial %s failed", PORT);
    // }
}

void Hoverboard::protocol_recv (char byte) {
    start_frame = ((uint16_t)(byte) << 8) | prev_byte;

    // Read the start frame
    if (start_frame == START_FRAME) {
        p = (char*)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
    }

    if (msg_len == sizeof(SerialFeedback)) {
        uint16_t checksum = (uint16_t)(
            msg.start ^
            msg.cmd1 ^
            msg.cmd2 ^
            msg.speedR_meas ^
            msg.speedL_meas ^
            msg.batVoltage ^
            msg.boardTemp ^
            msg.cmdLed);

        if (msg.start == START_FRAME && msg.checksum == checksum) {
            std_msgs::msg::Float64 f;

            f.data = (double)msg.batVoltage/100.0;
            voltage_pub_->publish(f);

            f.data = (double)msg.boardTemp/10.0;
            temp_pub_->publish(f);

            f.data = (double)msg.speedL_meas;
            vel_pub_[0]->publish(f);
            f.data = (double)msg.speedR_meas;
            vel_pub_[1]->publish(f);

            f.data = (double)msg.cmd1;
            cmd_pub_[0]->publish(f);
            f.data = (double)msg.cmd2;
            cmd_pub_[1]->publish(f);
        } else {
            RCLCPP_INFO(this->get_logger(), "Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        msg_len = 0;
    }
    prev_byte = byte;
}

void Hoverboard::write() {
    if (port_fd == -1) {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }
    // Calculate steering from difference of left and right
    const double speed = (setpoint[0] + setpoint[1])/2.0;
    const double steer = (setpoint[0] - setpoint[1])*2.0;

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)steer;
    command.speed = (int16_t)speed;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    int rc = ::write(port_fd, (const void*)&command, sizeof(command));
    if (rc < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to hoverboard serial port");
    }
}

