#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class CmdVelBridge : public rclcpp::Node
{
public:
    CmdVelBridge()
        : Node("cmd_vel_bridge")
    {
        // Subscriber to /cmd_vel (Twist)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelBridge::cmdVelCallback, this, std::placeholders::_1));

        // Publisher to /robot_controller/cmd_vel (TwistStamped)
        // /robot_diff_controller/cmd_vel ----> for, diff drive topic
        // /robot_controller/cmd_vel -----> for, simple controller topic 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/robot_controller/cmd_vel", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Convert Twist to TwistStamped
        auto twist_stamped = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_stamped->header.stamp = this->now();
        twist_stamped->header.frame_id = "base_link";  
        twist_stamped->twist = *msg;

        // Publish
        cmd_vel_pub_->publish(std::move(twist_stamped));
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}