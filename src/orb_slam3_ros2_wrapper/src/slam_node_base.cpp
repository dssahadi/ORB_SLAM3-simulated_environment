/**
 * @file slam_node_base.cpp
 * @brief Implementation of the common ROS 2 wrapper base class (POSE-ONLY).
 */

#include "orb_slam3_ros2_wrapper/slam_node_base.hpp"

#include <Eigen/Core>

namespace ORB_SLAM3_Wrapper
{
    using namespace WrapperTypeConversions;

    SlamNodeBase::SlamNodeBase(const std::string &node_name,
                               const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor)
        : rclcpp::Node(node_name)
    {
#ifdef ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA
        KernelController::setGPURunMode(true, true, true, true, false);
#endif

        // ---------------- Publishers (POSE ONLY) ----------------
        robotPoseMapFrame_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "robot_pose_slam", 10);

        // ---------------- TF ----------------
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        // ---------------- Parameters ----------------
        bool bUseViewer;
        this->declare_parameter("visualization", true);
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("robot_base_frame", "base_footprint");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("odometry_mode", false);
        this->get_parameter("odometry_mode", odometry_mode_);

        this->declare_parameter("publish_tf", true);
        this->get_parameter("publish_tf", publish_tf_);

        // Initial pose parameters
        this->declare_parameter("robot_x", 0.0);
        this->declare_parameter("robot_y", 0.0);
        this->declare_parameter("robot_z", 0.0);
        this->declare_parameter("robot_qx", 0.0);
        this->declare_parameter("robot_qy", 0.0);
        this->declare_parameter("robot_qz", 0.0);
        this->declare_parameter("robot_qw", 1.0);

        this->get_parameter("robot_x", robot_x_);
        this->get_parameter("robot_y", robot_y_);
        this->get_parameter("robot_z", robot_z_);
        this->get_parameter("robot_qx", robot_qx_);
        this->get_parameter("robot_qy", robot_qy_);
        this->get_parameter("robot_qz", robot_qz_);
        this->get_parameter("robot_qw", robot_qw_);

        geometry_msgs::msg::Pose initial_pose;
        initial_pose.position.x = robot_x_;
        initial_pose.position.y = robot_y_;
        initial_pose.position.z = robot_z_;
        initial_pose.orientation.x = robot_qx_;
        initial_pose.orientation.y = robot_qy_;
        initial_pose.orientation.z = robot_qz_;
        initial_pose.orientation.w = robot_qw_;

        // ---------------- ORB-SLAM3 Interface ----------------
        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(
            strVocFile,
            strSettingsFile,
            sensor,
            bUseViewer,
            false,               // loop closing disabled
            initial_pose,
            global_frame_,
            odom_frame_id_,
            robot_base_frame_id_);

        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();
    }

    SlamNodeBase::~SlamNodeBase()
    {
        interface_.reset();
    }

    void SlamNodeBase::onTracked(const std_msgs::msg::Header &stamp_source_header)
    {
        isTracked_ = true;

        if (publish_tf_)
        {
            if (!odometry_mode_)
            {
                geometry_msgs::msg::TransformStamped tfMapRobot;
                tfMapRobot.header.stamp = stamp_source_header.stamp;
                tfMapRobot.header.frame_id = global_frame_;
                tfMapRobot.child_frame_id = odom_frame_id_;
                interface_->getDirectMapToRobotTF(stamp_source_header, tfMapRobot);
                tfBroadcaster_->sendTransform(tfMapRobot);
            }
            else
            {
                try
                {
                    auto msgOdom =
                        tfBuffer_->lookupTransform(
                            odom_frame_id_,
                            robot_base_frame_id_,
                            tf2::TimePointZero);

                    interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
                    tfBroadcaster_->sendTransform(tfMapOdom_);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "TF error: %s", ex.what());
                    return;
                }
            }
        }

        geometry_msgs::msg::PoseStamped pose;
        interface_->getRobotPose(pose);
        pose.header.stamp = stamp_source_header.stamp;
        robotPoseMapFrame_->publish(pose);

        ++frequency_tracker_count_;
    }

} // namespace ORB_SLAM3_Wrapper

