#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistPublisher : public rclcpp::Node
{
public:
    TwistPublisher() : Node("twist_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TwistPublisher::publish_twist, this));
    }

private:
    void publish_twist()
    {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.twist.linear.x = 1.0;
        msg.twist.angular.z = 0.5;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing TwistStamped message");
    }
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}