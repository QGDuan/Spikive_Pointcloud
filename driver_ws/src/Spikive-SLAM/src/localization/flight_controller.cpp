#include <Python.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include <livox_ros_driver/CustomMsg.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <cmath>
#include <csignal>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>

#include "lsdc_geo.hpp"
#include "lsdc_tools.hpp"
#include "odom_struct.hpp"

ros::Publisher pub_trans_odom;
ros::Publisher pub_trans_pcl;

Eigen::Matrix3f rotationMatrix;
Eigen::Vector3f translationVector;
Eigen::Vector3f init_translation;  // 新增初始化平移参数

Eigen::Matrix4f odomTransMatrix;

OdomStructf diff_odom, lio_odom, global_odom, trans_odom, init_diff_odom;

bool allFinite(const std::vector<double>& values) {
    for (double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

bool validateExtrinsic(const std::vector<double>& R, const std::vector<double>& T) {
    if (R.size() != 9 || T.size() != 3) {
        ROS_ERROR("Invalid R or T size: R=%zu T=%zu", R.size(), T.size());
        return false;
    }
    if (!allFinite(R) || !allFinite(T)) {
        ROS_ERROR("Invalid R or T: contains non-finite value");
        return false;
    }

    Eigen::Matrix3f rotation;
    rotation << R[0], R[1], R[2],
                R[3], R[4], R[5],
                R[6], R[7], R[8];
    const float det = rotation.determinant();
    const float orth_error =
        (rotation * rotation.transpose() - Eigen::Matrix3f::Identity()).norm();
    if (!std::isfinite(det) || std::fabs(det - 1.0f) > 1e-2f || orth_error > 1e-2f) {
        ROS_ERROR("Invalid R: det=%f orth_error=%f", det, orth_error);
        return false;
    }
    return true;
}

// 注释全局变量
// Eigen::Quaternionf init_orientation(1, 0, 0, 0);  // 初始四元数 (w,x,y,z)
// ros::Publisher pub_landing_target;

void lioOdomCbk(const nav_msgs::Odometry::ConstPtr& msg)  {

    lio_odom.T = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
    lio_odom.Q = Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    diff_odom.T = translationVector;

    diff_odom.Q = Eigen::Quaternionf (rotationMatrix);

    // R/T is the LiDAR-to-motion-center extrinsic; odom uses SE3 conjugation.
    Eigen::Matrix4f trans_odom_ = diff_odom.M() * lio_odom.M() * diff_odom.M().inverse();

    OdomStructf trans_odom;
    trans_odom.setFromSE3(trans_odom_);

    // 应用初始化平移
    trans_odom.T += init_translation;

    nav_msgs::Odometry odom_msg;
    // odom_msg.header = msg->header;

    odom_msg.header.frame_id = "world";
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.child_frame_id = "body";
    setPoseStamp(odom_msg.pose, trans_odom.T, trans_odom.Q);
    pub_trans_odom.publish(odom_msg);

    // 注释landing_target发布代码
    /*
    geometry_msgs::PoseStamped landing_pose;
    landing_pose.header.frame_id = "world";
    landing_pose.header.stamp = ros::Time::now();
    landing_pose.pose.position.x = init_translation[0];
    landing_pose.pose.position.y = init_translation[1];
    landing_pose.pose.position.z = init_translation[2];
    landing_pose.pose.orientation.x = init_orientation.x();
    landing_pose.pose.orientation.y = init_orientation.y();
    landing_pose.pose.orientation.z = init_orientation.z();
    landing_pose.pose.orientation.w = init_orientation.w();
    pub_landing_target.publish(landing_pose);
    */

        // std::cout<<"counter_:"<<counter_<<std::endl;
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // 将 sensor_msgs::PointCloud2 消息转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // 发布当前帧点云
    pcl::PointCloud<pcl::PointXYZI> curr_correct_cloud;

    // 遍历点云中的每个点
    for (auto& point : cloud.points) {
        pcl::PointXYZI pi = point;
        Eigen::Vector3f pv(pi.x, pi.y, pi.z);
        // Point cloud is already in LIO startup world; apply LiDAR-to-motion-center extrinsic.
        pv = rotationMatrix * pv + translationVector + init_translation;
        pi.x = pv[0];
        pi.y = pv[1];
        pi.z = pv[2];
        curr_correct_cloud.push_back(pi);
    }

    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(curr_correct_cloud, pub_cloud);
    // pub_cloud.header = cloud_msg->header;
    // pub_cloud.header.frame_id = "base_link";
    pub_cloud.header.stamp = cloud_msg->header.stamp;
    pub_cloud.header.frame_id = "world";
    pub_trans_pcl.publish(pub_cloud);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_to_uav_transform");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 从参数服务器获取R和T
    std::vector<double> R, T;
    nh.getParam("R", R);
    nh.getParam("T", T);

    if (!validateExtrinsic(R, T)) {
        return -1;
    }

    // 初始化旋转矩阵和平移向量
    rotationMatrix << R[0], R[1], R[2],
                      R[3], R[4], R[5],
                      R[6], R[7], R[8];
    translationVector << T[0], T[1], T[2];

    // 修改后的参数读取方式
    private_nh.param<float>("init_x", init_translation[0], 0.0f);
    private_nh.param<float>("init_y", init_translation[1], 0.0f);
    private_nh.param<float>("init_z", init_translation[2], 0.0f);

    // 注释参数读取
    /*
    private_nh.param<float>("init_qx", init_orientation.x(), 0.0f);
    private_nh.param<float>("init_qy", init_orientation.y(), 0.0f);
    private_nh.param<float>("init_qz", init_orientation.z(), 0.0f);
    private_nh.param<float>("init_qw", init_orientation.w(), 1.0f);
    */

    // 注释发布者初始化
    // pub_landing_target = nh.advertise<geometry_msgs::PoseStamped>("/mavros/landing_target/pose", 10);

    // 订阅里程计和点云数据
    ros::Subscriber sub_odom = nh.subscribe("/Odometry", 10, lioOdomCbk);
    ros::Subscriber pointcloud_sub = nh.subscribe("/cloud_registered", 1, pointcloudCallback);

    // 发布变换后的里程计和点云数据
    pub_trans_odom = nh.advertise<nav_msgs::Odometry>("/Odometry_trans", 10);
    pub_trans_pcl = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_trans", 1);

    ROS_INFO("slam_to_uav_transform publishes fallback topics only: /Odometry_trans, /cloud_registered_trans");


    ros::spin();

    return 0;
}
