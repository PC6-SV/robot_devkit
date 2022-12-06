#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class Relay : public rclcpp::Node
{
  public:
    Relay()
    : Node("relay")
    {
      this->declare_parameter("input_topic", "cmd_vel");
      this->declare_parameter("output_topic", "cmd_vel_relayed");
      std::string input_topic = this->get_parameter("input_topic").get_parameter_value().get<std::string>();
      std::string output_topic = this->get_parameter("output_topic").get_parameter_value().get<std::string>();
      subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(input_topic, 10, std::bind(&Relay::topic_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      publisher_->publish(*msg);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>());
  rclcpp::shutdown();
  return 0;
}
