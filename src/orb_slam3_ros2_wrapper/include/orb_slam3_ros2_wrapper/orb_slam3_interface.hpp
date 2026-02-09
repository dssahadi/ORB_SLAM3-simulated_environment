#ifndef ORBSLAM3_INTERFACE_HPP
#define ORBSLAM3_INTERFACE_HPP

#include <iostream>
#include <mutex>
#include <string>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "sophus/se3.hpp"
#include "System.h"

#include "orb_slam3_ros2_wrapper/type_conversion.hpp"
#include "orb_slam3_ros2_wrapper/time_profiler.hpp"

namespace ORB_SLAM3_Wrapper
{
class ORBSLAM3Interface
{
public:
    ORBSLAM3Interface(const std::string &voc,
                      const std::string &settings,
                      ORB_SLAM3::System::eSensor sensor,
                      bool useViewer,
                      bool /* loopClosing */,
                      geometry_msgs::msg::Pose initialRobotPose,
                      std::string globalFrame,
                      std::string odomFrame,
                      std::string robotFrame);

    ~ORBSLAM3Interface();

    bool processTrackedPose(const Sophus::SE3f& Tcw);

    void getDirectMapToRobotTF(std_msgs::msg::Header header,
                              geometry_msgs::msg::TransformStamped &tf);

    void getMapToOdomTF(const geometry_msgs::msg::TransformStamped& odomToBaseTf,
                        geometry_msgs::msg::TransformStamped &tf);

    void getRobotPose(geometry_msgs::msg::PoseStamped& pose);

    ORB_SLAM3::System* slam() { return mSLAM_.get(); }

private:
    void correctTrackedPose(const Sophus::SE3f &Tcw);

    std::shared_ptr<ORB_SLAM3::System> mSLAM_;
    ORB_SLAM3::System::eSensor sensor_;

    std::mutex poseMutex_;

    Eigen::Affine3f latestTrackedPose_;
    Eigen::Affine3f robotBase_to_cameraLink_;

    bool hasTracked_ = false;

    std::string globalFrame_;
    std::string odomFrame_;
    std::string robotFrame_;

    TimeProfiler* time_profiler_;
};
}

#endif

