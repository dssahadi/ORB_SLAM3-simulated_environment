#include "orb_slam3_ros2_wrapper/orb_slam3_interface.hpp"

namespace ORB_SLAM3_Wrapper
{
using namespace WrapperTypeConversions;

ORBSLAM3Interface::ORBSLAM3Interface(
    const std::string &voc,
    const std::string &settings,
    ORB_SLAM3::System::eSensor sensor,
    bool useViewer,
    bool,
    geometry_msgs::msg::Pose initialRobotPose,
    std::string globalFrame,
    std::string odomFrame,
    std::string robotFrame)
    : sensor_(sensor),
      globalFrame_(globalFrame),
      odomFrame_(odomFrame),
      robotFrame_(robotFrame)
{
    mSLAM_ = std::make_shared<ORB_SLAM3::System>(
        voc, settings, sensor, useViewer);

    robotBase_to_cameraLink_ =
        Eigen::Translation3f(
            initialRobotPose.position.x,
            initialRobotPose.position.y,
            initialRobotPose.position.z) *
        Eigen::Quaternionf(
            initialRobotPose.orientation.w,
            initialRobotPose.orientation.x,
            initialRobotPose.orientation.y,
            initialRobotPose.orientation.z);

    time_profiler_ = TimeProfiler::getInstance();
}

ORBSLAM3Interface::~ORBSLAM3Interface()
{
    mSLAM_->Shutdown();
}

bool ORBSLAM3Interface::processTrackedPose(const Sophus::SE3f& Tcw)
{
    if (mSLAM_->GetTrackingState() != ORB_SLAM3::Tracking::OK)
    {
        hasTracked_ = false;
        return false;
    }

    correctTrackedPose(Tcw);
    hasTracked_ = true;
    return true;
}

void ORBSLAM3Interface::correctTrackedPose(const Sophus::SE3f &Tcw)
{
    std::lock_guard<std::mutex> lock(poseMutex_);

    auto Twc = Tcw.inverse();
    Eigen::Affine3f poseORB = se3ToAffine(Twc);

    latestTrackedPose_ =
        robotBase_to_cameraLink_ *
        poseORB *
        robotBase_to_cameraLink_.inverse();
}

void ORBSLAM3Interface::getRobotPose(geometry_msgs::msg::PoseStamped& pose)
{
    std::lock_guard<std::mutex> lock(poseMutex_);
    pose.header.frame_id = globalFrame_;
    pose.pose = affine3fToPose(latestTrackedPose_);
}

void ORBSLAM3Interface::getDirectMapToRobotTF(
    std_msgs::msg::Header header,
    geometry_msgs::msg::TransformStamped &tf)
{
    if (!hasTracked_) return;

    std::lock_guard<std::mutex> lock(poseMutex_);

    auto pose = affine3fToPose(latestTrackedPose_);
    tf.header = header;
    tf.header.frame_id = globalFrame_;
    tf.child_frame_id = robotFrame_;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
}

void ORBSLAM3Interface::getMapToOdomTF(
    const geometry_msgs::msg::TransformStamped& odomToBaseTf,
    geometry_msgs::msg::TransformStamped &tf)
{
    if (!hasTracked_) return;

    Eigen::Affine3f odomPose =
        Eigen::Translation3f(
            odomToBaseTf.transform.translation.x,
            odomToBaseTf.transform.translation.y,
            odomToBaseTf.transform.translation.z) *
        Eigen::Quaternionf(
            odomToBaseTf.transform.rotation.w,
            odomToBaseTf.transform.rotation.x,
            odomToBaseTf.transform.rotation.y,
            odomToBaseTf.transform.rotation.z);

    std::lock_guard<std::mutex> lock(poseMutex_);

    auto mapToOdom = latestTrackedPose_ * odomPose.inverse();
    auto pose = affine3fToPose(mapToOdom);

    tf.header.stamp = odomToBaseTf.header.stamp;
    tf.header.frame_id = globalFrame_;
    tf.child_frame_id = odomFrame_;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
}

}

