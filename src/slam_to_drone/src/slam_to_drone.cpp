#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class SlamToDrone : public rclcpp::Node
{
public:
    SlamToDrone() : Node("slam_to_drone")
    {
        // Subscriber: Pose do SLAM
        slam_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose_slam", 10,
            std::bind(&SlamToDrone::slam_callback, this, std::placeholders::_1));

        // Publisher: odometria para o drone
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/fmu/in/vehicle_visual_odometry", 10);

        RCLCPP_INFO(get_logger(), "SlamToDrone node initialized");
    }

private:
    void slam_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom_msg;
        
        // Header
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "map"; // mesmo frame do SLAM
        odom_msg.child_frame_id = "base_link"; // frame do drone

        // Posição
        odom_msg.pose.pose = msg->pose;

        // Covariances (opcional, se quiser indicar confiança)
        for (int i = 0; i < 36; ++i)
            odom_msg.pose.covariance[i] = 0.0; // ou valores estimados do SLAM

        // Publica para o drone
        odom_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamToDrone>());
    rclcpp::shutdown();
    return 0;
}

