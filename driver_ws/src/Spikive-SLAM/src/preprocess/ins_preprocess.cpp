#include <Python.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include <livox_ros_driver/CustomMsg.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <csignal>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include "lsdc_slam/bywire_chassis_state.h"

#include <GeographicLib/LocalCartesian.hpp>
#include "Commons/WGS84toCartesian.hpp"
#include "Commons/transfer.hpp"
#include "lsdc_geo.hpp"
#include "lsdc_math.hpp"

using namespace std;

class InsMsgHandle {
   private:
    ros::NodeHandle nh;
    ros::Subscriber sub_imu, sub_gps;
    ros::Publisher pub_rtk_lla;
    queue<sensor_msgs::Imu::ConstPtr> imu_msg_que;
    queue<sensor_msgs::NavSatFix::ConstPtr> gps_msg_que;

    double init_time{-1};

    bool gpsStatusOk(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
        return (gps_msg->status.status == 48 || gps_msg->status.status == 49 ||
                gps_msg->status.status == 50);
    }
    void pubRtkMsg(const sensor_msgs::Imu::ConstPtr& imu_msg,
                   const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
        nav_msgs::Odometry rtk_msg;
        rtk_msg.header.stamp = gps_msg->header.stamp;
        rtk_msg.header.frame_id = "world";
        rtk_msg.child_frame_id = gpsStatusOk(gps_msg) ? "OK" : "-";
        rtk_msg.pose.pose.position.x = gps_msg->latitude;
        rtk_msg.pose.pose.position.y = gps_msg->longitude;
        rtk_msg.pose.pose.position.z = gps_msg->altitude;
        rtk_msg.pose.pose.orientation.w = imu_msg->orientation.w;
        rtk_msg.pose.pose.orientation.x = imu_msg->orientation.x;
        rtk_msg.pose.pose.orientation.y = imu_msg->orientation.y;
        rtk_msg.pose.pose.orientation.z = imu_msg->orientation.z;
        pub_rtk_lla.publish(rtk_msg);
    }

    bool msgEmpty() { return imu_msg_que.empty() || gps_msg_que.empty(); }
    void synMsgAndPub() {
        while (!msgEmpty()) {
            if (imu_msg_que.front()->header.stamp.toSec() <
                gps_msg_que.front()->header.stamp.toSec() - 0.05)
                imu_msg_que.pop();
            else if (imu_msg_que.front()->header.stamp.toSec() >
                     gps_msg_que.front()->header.stamp.toSec() + 0.05) {
                gps_msg_que.pop();
            } else {
                sensor_msgs::ImuConstPtr imu_msg;
                sensor_msgs::NavSatFixConstPtr gps_msg;
                double d_time{1.0};
                while (!imu_msg_que.empty()) {
                    if (std::abs(imu_msg_que.front()->header.stamp.toSec() -
                                 gps_msg_que.front()->header.stamp.toSec()) < d_time) {
                        d_time = std::abs(imu_msg_que.front()->header.stamp.toSec() -
                                          gps_msg_que.front()->header.stamp.toSec());
                        imu_msg = imu_msg_que.front();
                        imu_msg_que.pop();
                    } else
                        break;
                }
                gps_msg = gps_msg_que.front();
                gps_msg_que.pop();

                // std::cout << imu_msg->header.stamp.toSec() - init_time << "  "
                //           << gps_msg->header.stamp.toSec() - init_time << "  "
                //           << gps_msg->header.stamp.toSec() - imu_msg->header.stamp.toSec() <<
                //           "\n";
                pubRtkMsg(imu_msg, gps_msg);
                break;
            }
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_in) {
        if (init_time < 0)
            init_time = msg_in->header.stamp.toSec();

        imu_msg_que.push(msg_in);
        synMsgAndPub();
        // cout << "imu stamp: " << msg_in->header.stamp.toSec() - init_time << "\n";
    }
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg_in) {
        if (init_time < 0)
            init_time = msg_in->header.stamp.toSec();

        gps_msg_que.push(msg_in);
        synMsgAndPub();
        // cout << "gps stamp: " << msg_in->header.stamp.toSec() - init_time << "\n";
    }

   public:
    InsMsgHandle() {
        ROS_INFO("[Ins Preprocess]: Start");

        string imu_topic, gps_topic;
        nh.param<string>("ins/gps_topic", gps_topic, "/rtk_gps");
        nh.param<string>("ins/imu_topic", imu_topic, "/rtk_imu");

        sub_gps = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 100000,
                                                       &InsMsgHandle::gpsCallback, this);
        sub_imu = nh.subscribe(imu_topic, 100000, &InsMsgHandle::imuCallback, this);
        pub_rtk_lla = nh.advertise<nav_msgs::Odometry>("/lsdc_rtk", 100000);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ins_preprocess");
    InsMsgHandle imh;

    std::cout << "[SLAM]: " << "Init ins_preprocess Success" << std::endl;
    ros::spin();
    return 0;
}