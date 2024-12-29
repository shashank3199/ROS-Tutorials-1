#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistSubscriber : public rclcpp::Node
{
public:
    TwistSubscriber() : Node("twist_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", 10,
            std::bind(&TwistSubscriber::twist_callback, this, std::placeholders::_1));
    }

private:
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received twist - linear.x: %f, angular.z: %f",
                    msg->twist.linear.x, msg->twist.angular.z);
    }
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}