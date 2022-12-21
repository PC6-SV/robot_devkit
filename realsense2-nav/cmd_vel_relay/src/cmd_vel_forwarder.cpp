#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

#define ZERO_VEL_COMMAND      geometry_msgs::msg::Twist()

using std::placeholders::_1;

class Relay : public rclcpp::Node
{
  public:
    Relay()
    : Node("relay")
    {
      this->declare_parameter("input_topic", "cmd_vel");
      this->declare_parameter("output_topic", "cmd_vel_relayed");
      this->declare_parameter("timeout", 0.5);
      std::string input_topic = this->get_parameter("input_topic").get_parameter_value().get<std::string>();
      std::string output_topic = this->get_parameter("output_topic").get_parameter_value().get<std::string>();
      double timeout = this->get_parameter("timeout").get_parameter_value().get<double>();
      subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(input_topic, 10, std::bind(&Relay::topic_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(
            timeout)), std::bind(&Relay::timer_callback, this));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      this->timer_->reset();
      publisher_->publish(*msg);
    }
    void timer_callback()
    {
      publisher_->publish(ZERO_VEL_COMMAND);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>());
  rclcpp::shutdown();
  return 0;
}
