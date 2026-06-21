#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <cmath>
#include <csignal>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "lsdc_tools.hpp"
#include "odom_struct.hpp"

using namespace std;

string kDroneId{"2"};
string kStableCloudTopic, kStableOdomTopic, kDiffOdomTopic, kInitMatchSuccessTopic;
OdomStructf diff_odom, calibrated_odom;
ros::Publisher pub_fusion_odom, pub_fusion_pcl, pub_stable_odom, pub_stable_pcl,
    pub_mavros_pose, pub_companion_status;
bool first_calibrated_odom_received{false}, first_diff_odom_received{false},
    init_match_success{false};
std::mutex mutex_state;

#define MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY 197

sensor_msgs::PointCloud2 transformCloudWithOdom(const sensor_msgs::PointCloud2ConstPtr& msg_in,
                                                OdomStructf transform_odom) {
    sensor_msgs::PointCloud2 msg_out = *msg_in;
    pcl::PointCloud<pcl::PointXYZI> cloud_in;
    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    pcl::fromROSMsg(*msg_in, cloud_in);
    cloud_out.points.reserve(cloud_in.points.size());

    Eigen::Matrix4f transform = transform_odom.M();
    for (const auto& point : cloud_in.points) {
        pcl::PointXYZI point_out = point;
        Eigen::Vector4f p(point.x, point.y, point.z, 1.f);
        p = transform * p;
        point_out.x = p.x();
        point_out.y = p.y();
        point_out.z = p.z();
        cloud_out.push_back(point_out);
    }

    pcl::toROSMsg(cloud_out, msg_out);
    msg_out.header.frame_id = "world";
    msg_out.header.stamp = msg_in->header.stamp;
    return msg_out;
}

void publishMavrosPose(const OdomStructf& odom, const ros::Time& stamp) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = stamp;
    pose_msg.pose.position.x = odom.T.x();
    pose_msg.pose.position.y = odom.T.y();
    pose_msg.pose.position.z = odom.T.z();
    pose_msg.pose.orientation.x = odom.Q.x();
    pose_msg.pose.orientation.y = odom.Q.y();
    pose_msg.pose.orientation.z = odom.Q.z();
    pose_msg.pose.orientation.w = odom.Q.w();
    pub_mavros_pose.publish(pose_msg);
}

void publishCompanionStatus(const ros::Time& stamp) {
    mavros_msgs::CompanionProcessStatus status_msg;
    status_msg.header.frame_id = "world";
    status_msg.header.stamp = stamp;
    status_msg.component = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    status_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;
    pub_companion_status.publish(status_msg);
}

void pubOdom(const OdomStructf& odom, ros::Publisher& pub) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "body";
    odom_msg.header.stamp = odom.timestamp;
    setPoseStamp(odom_msg.pose, odom.T, odom.Q);
    pub.publish(odom_msg);
}

void calibratedOdomCbk(const nav_msgs::Odometry::ConstPtr& msg) {
    OdomStructf next_calibrated_odom;
    next_calibrated_odom.T =
        Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    next_calibrated_odom.Q =
        Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    next_calibrated_odom.timestamp = msg->header.stamp;

    OdomStructf output_odom;
    bool use_localization{false};
    {
        std::lock_guard<std::mutex> lock(mutex_state);
        calibrated_odom = next_calibrated_odom;
        first_calibrated_odom_received = true;
        use_localization = init_match_success && first_diff_odom_received;
        if (use_localization) {
            // Localization success: stable outputs are aligned to the loaded WayPoint map world.
            output_odom.timestamp = calibrated_odom.timestamp;
            output_odom.setFromSE3(diff_odom.M() * calibrated_odom.M());
        } else {
            // Fallback before localization success: stable outputs remain in startup world.
            output_odom = calibrated_odom;
        }
    }

    pubOdom(output_odom, pub_stable_odom);
    pubOdom(output_odom, pub_fusion_odom);
    const ros::Time mavros_stamp = ros::Time::now();
    publishMavrosPose(output_odom, mavros_stamp);
    publishCompanionStatus(mavros_stamp);

}

void diffOdomCbk(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_state);
    diff_odom.T = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                  msg->pose.pose.position.z);
    diff_odom.Q = Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                     msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    diff_odom.timestamp = msg->header.stamp;
    first_diff_odom_received = true;
}

void initSuccessCallback(const std_msgs::BoolConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_state);
    if (!msg->data) {
        init_match_success = false;
        first_diff_odom_received = false;
        return;
    }
    init_match_success = true;
}

void pclCbk(const sensor_msgs::PointCloud2ConstPtr& msg_in) {
    OdomStructf correction_odom;
    bool use_localization{false};
    {
        std::lock_guard<std::mutex> lock(mutex_state);
        if (!first_calibrated_odom_received) {
            return;
        }
        if (init_match_success && first_diff_odom_received) {
            correction_odom = diff_odom;
            use_localization = true;
        }
    }

    sensor_msgs::PointCloud2 stable_cloud;
    if (use_localization) {
        // Localization success: transform calibrated cloud into the loaded WayPoint map world.
        stable_cloud = transformCloudWithOdom(msg_in, correction_odom);
    } else {
        // Fallback before localization success: publish calibrated startup-world cloud as-is.
        stable_cloud = *msg_in;
        stable_cloud.header.frame_id = "world";
    }
    pub_fusion_pcl.publish(stable_cloud);
    pub_stable_pcl.publish(stable_cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_repub_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<string>("drone_id", kDroneId, "2");
    private_nh.param<string>("stable_cloud_topic", kStableCloudTopic,
                             "/drone_" + kDroneId + "_cloud_registered");
    private_nh.param<string>("stable_odom_topic", kStableOdomTopic,
                             "/drone_" + kDroneId + "_visual_slam/odom");
    private_nh.param<string>("diff_odom_topic", kDiffOdomTopic,
                             "/drone_" + kDroneId + "_diff_odom");
    private_nh.param<string>("init_match_success_topic", kInitMatchSuccessTopic,
                             "/drone_" + kDroneId + "_init_match_success");

    pub_fusion_pcl = nh.advertise<sensor_msgs::PointCloud2>("/localization_cloud_registered", 1);
    pub_fusion_odom = nh.advertise<nav_msgs::Odometry>("/localization_odom", 10);
    pub_stable_pcl = nh.advertise<sensor_msgs::PointCloud2>(kStableCloudTopic, 1);
    pub_stable_odom = nh.advertise<nav_msgs::Odometry>(kStableOdomTopic, 10);
    pub_mavros_pose = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    pub_companion_status = nh.advertise<mavros_msgs::CompanionProcessStatus>(
        "/mavros/companion_process/status", 10);
    ros::Subscriber sub_diff_odom = nh.subscribe(kDiffOdomTopic, 1, diffOdomCbk);
    ros::Subscriber sub_odom = nh.subscribe("/Odometry_trans", 10, calibratedOdomCbk);
    ros::Subscriber sub_scan = nh.subscribe("/cloud_registered_trans", 1, pclCbk);
    ros::Subscriber sub_init_match_success =
        nh.subscribe(kInitMatchSuccessTopic, 10, initSuccessCallback);

    ROS_INFO("Localization fusion_repub stable outputs: cloud=%s odom=%s diff=%s success=%s",
             kStableCloudTopic.c_str(), kStableOdomTopic.c_str(), kDiffOdomTopic.c_str(),
             kInitMatchSuccessTopic.c_str());
    ros::spin();

    return 0;
}
