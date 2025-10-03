#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CmdvelPublisher : public rclcpp::Node {
public:
    CmdvelPublisher()
        : Node("joystick_to_cmdvel") {

        init_parameters();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)),
            std::bind(&CmdvelPublisher::joy_callback, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((uint64_t) (1000 / cmd_vel_publish_rate_hz)),
            std::bind(&CmdvelPublisher::cmd_vel_publish_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    sensor_msgs::msg::Joy joy_msg {};

    std::string cmd_vel_topic { "/cmd_vel" };
    std::string joy_topic { "/joy" };

    int64_t linear_x_axis { 1 };
    int64_t angular_z_axis { 0 };

    int64_t throttle_axis { 5 };

    int64_t turbo_enable_button { 5 };
    int64_t joy_enable_button { 4 };

    double maximum_linear_x_velocity { 2.0 };
    double maximum_angular_z_velocity { 2.0 };

    double cmd_vel_publish_rate_hz { 100.0 };

    double joy_timeout_seconds { 0.5 };

    rclcpp::Time prev_joy_received_time { this->get_clock()->now() };

    void cmd_vel_publish_callback() {
        geometry_msgs::msg::Twist cmd_vel {};

        if (joy_msg.axes.empty() || joy_msg.buttons.empty()) {
            return;
        }

        if ((this->get_clock()->now() - prev_joy_received_time).seconds() < joy_timeout_seconds) {

            try {

                if (joy_msg.buttons.at(joy_enable_button)) {

                    double throttle = -(joy_msg.axes.at(throttle_axis) - 1.0) / 2.0;

                    double max_linear_x = maximum_linear_x_velocity;
                    double max_angular_z = maximum_angular_z_velocity;

                    if (!joy_msg.buttons[turbo_enable_button]) {
                        max_linear_x /= 2.0;
                        max_angular_z /= 2.0;
                    }

                    double linear_x_velocity = joy_msg.axes.at(linear_x_axis) * max_linear_x * throttle;
                    double angular_z_velocity = joy_msg.axes.at(angular_z_axis) * max_angular_z * throttle;

                    cmd_vel.linear.x = linear_x_velocity;
                    cmd_vel.angular.z = angular_z_velocity;

                }

            } catch (std::out_of_range& err) {
                RCLCPP_ERROR(this->get_logger(), "The axis or button does not exist!\n");
            }
        }
        publisher_->publish(cmd_vel);

    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg != nullptr) {
            prev_joy_received_time = this->get_clock()->now();
            joy_msg = *msg;
        }
    }

    void init_parameters() {
        linear_x_axis = this->declare_parameter("linear_x_joy_axis", linear_x_axis);
        angular_z_axis = this->declare_parameter("angular_z_joy_axis", angular_z_axis);

        throttle_axis = this->declare_parameter("throttle_joy_axis", throttle_axis);

        turbo_enable_button = this->declare_parameter("turbo_enable_button", turbo_enable_button);
        joy_enable_button = this->declare_parameter("joy_enable_button", joy_enable_button);

        maximum_linear_x_velocity = this->declare_parameter("maximum_linear_x_velocity", maximum_linear_x_velocity);
        maximum_angular_z_velocity = this->declare_parameter("maximum_angular_z_velocity", maximum_angular_z_velocity);

        cmd_vel_publish_rate_hz = this->declare_parameter("cmd_vel_publish_rate", cmd_vel_publish_rate_hz);

        joy_timeout_seconds = this->declare_parameter("joy_timeout", joy_timeout_seconds);
    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdvelPublisher>());
    rclcpp::shutdown();
    return 0;
}