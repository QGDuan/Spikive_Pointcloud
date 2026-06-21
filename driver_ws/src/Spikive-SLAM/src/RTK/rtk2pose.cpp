#include <Python.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
#include <Eigen/Eigen>
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
#include "Commons/convert_coordinates.hpp"
#include "Commons/transfer.hpp"
#include "lsdc_slam/bywire_chassis_state.h"

#include <GeographicLib/LocalCartesian.hpp>
#include "Commons/WGS84toCartesian.hpp"
#include "Commons/transfer.hpp"
#include "lsdc_geo.hpp"
#include "lsdc_math.hpp"
#include "lsdc_tools.hpp"

using namespace std;

ros::Subscriber sub_rtk;
ros::Publisher pub_initial_pose, pub_rtk_odom, pub_rtk_path;
nav_msgs::Odometry odom_msg;
nav_msgs::Path path_msg;
string kSavePath;
string kLastPoseFile = string(string(ROOT_DIR)) + "/Lsdc_Repub/last_pose.txt";
bool init_pose{false};
bool kUseMapOrigin{false};
string kFrameId{"camera_init"};

Eigen::Vector3d init_dT, init_rtk_L;
Eigen::Quaterniond init_dQ, init_rtk_Q;

lsdc::LsdcGeographicLib lgl;

ofstream init_pose_out;
ifstream init_pose_in;

template <typename T>
void printV(T& lsdc_out, Eigen::Vector3d v) {
    lsdc_out << fixed << setprecision(20) << "[" << v.x() << ", " << v.y() << ", " << v.z()
             << "]\n";
}
template <typename T>
void printQ(T& lsdc_out, Eigen::Quaterniond q) {
    lsdc_out << fixed << setprecision(20) << "[" << q.x() << ", " << q.y() << ", " << q.z() << ", "
             << q.w() << "]\n";
}

void printInitPose() {
    string kInitPoseFile = kSavePath + "/pose_init.txt";
    init_pose_out.open(kInitPoseFile, std::ios::out);

    cout << "[Init Pose]:\nLat, Lon, Alt  ";
    printV(cout, init_rtk_L);
    cout << "Orientation(x, y, z, w):  ";
    printQ(cout, init_rtk_Q);
    cout << "Eular_Angle(row, pitch, yaw):  ";
    printV(cout, 180 / M_PI * lsdc::Q_to_EulerAngle(init_rtk_Q));
    cout << "\n";

    init_pose_out << "Origin Pose:\nLat, Lon, Alt:  ";
    printV(init_pose_out, lgl.origin_L);
    init_pose_out << "Orientation(x, y, z, w):  ";
    printQ(init_pose_out, lgl.origin_Q);
    init_pose_out << "Eular_Angle(row, pitch, yaw):  ";
    printV(init_pose_out, 180 / M_PI * lsdc::Q_to_EulerAngle(lgl.origin_Q));
    init_pose_out << "\n";

    init_pose_out << "Init Pose:\nLat, Lon, Alt:  ";
    printV(init_pose_out, init_rtk_L);
    init_pose_out << "Orientation(x, y, z, w):  ";
    printQ(init_pose_out, init_rtk_Q);
    init_pose_out << "Eular_Angle(row, pitch, yaw):  ";
    printV(init_pose_out, 180 / M_PI * lsdc::Q_to_EulerAngle(init_rtk_Q));
    init_pose_out << "\n";

    init_pose_out << "Delta Pose:\nDisplacement(x, y, z):  ";
    printV(init_pose_out, init_dT);
    init_pose_out << "Orientation(x, y, z, w):  ";
    printQ(init_pose_out, init_dQ);
    init_pose_out << "Eular_Angle(row, pitch, yaw):  ";
    printV(init_pose_out, 180 / M_PI * lsdc::Q_to_EulerAngle(init_dQ));
    init_pose_out << "\n";

    init_pose_out.close();
}

void pubInitPose() {
    while (ros::ok()) {
        ros::Duration(0.5).sleep();
        if (init_pose) {
            geometry_msgs::PoseWithCovarianceStamped initial_pose;
            initial_pose.header.frame_id = "map";
            initial_pose.header.stamp = ros::Time::now();
            setPoseStamp(initial_pose.pose, init_dT, init_dQ);

            pub_initial_pose.publish(initial_pose);
        }
    }
}

void pubRtkPose(const nav_msgs::Odometry::Ptr& msg,
                Eigen::Vector3d curr_T,
                Eigen::Quaterniond curr_Q) {
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = kFrameId;
    odom_msg.child_frame_id = msg->child_frame_id;
    setPoseStamp(odom_msg.pose, curr_T, curr_Q);
    pub_rtk_odom.publish(odom_msg);

    static int jjj{0};
    if (jjj++ % 10 == 0) {
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = kFrameId;
        setPoseStamp(pose_stamp, curr_T, curr_Q);

        path_msg.poses.push_back(pose_stamp);
        pub_rtk_path.publish(path_msg);
    }
}

void rtkCallback(const nav_msgs::Odometry::Ptr& msg) {
    Eigen::Vector3d curr_L, dT;
    Eigen::Quaterniond curr_Q, dQ;

    curr_L = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    curr_Q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    if (!init_pose) {
        if (msg->child_frame_id == "OK") {
            ;
        } else if (msg->child_frame_id == "ERROR") {
            init_pose_in.open(kLastPoseFile, ios::in);
            if (init_pose_in.is_open()) {
                string str;
                if (getline(init_pose_in, str)) {
                    istringstream istr(str);
                    istr >> curr_L.x() >> curr_L.y() >> curr_L.z();
                }
                if (getline(init_pose_in, str)) {
                    istringstream istr(str);
                    istr >> curr_Q.x() >> curr_Q.y() >> curr_Q.z() >> curr_Q.w();
                }
                ROS_WARN("Read Rtk From File.");
                // cout << std::setprecision(10) << init_L.transpose() << "\n"
                //      << init_Q.coeffs().transpose() << "\n\n";
            }
            init_pose_in.close();
        } else {
            ROS_WARN("No Init RTK Signal!");
            return;
        }

        lgl.getOriginFromRtk(curr_L, curr_Q, init_rtk_L, init_rtk_Q);
        if (lgl.origin_L.x() == 0 && lgl.origin_L.y() == 0 && lgl.origin_L.z() == 0) {
            lgl.origin_L = init_rtk_L;
            lgl.origin_Q = init_rtk_Q;
        }
        if (!kUseMapOrigin) {
            lgl.origin_L = init_rtk_L;
            lgl.origin_Q = init_rtk_Q;
        }

        lgl.getOdomFromRtk(curr_L, curr_Q, dT, dQ);
        init_dT = dT;
        init_dQ = dQ;
        init_pose = true;

        printInitPose();
    }

    lgl.getOdomFromRtk(curr_L, curr_Q, dT, dQ);
    pubRtkPose(msg, dT, dQ);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtk2pose");
    ros::NodeHandle nh;

    lgl.setRtkParamFromCfg(nh);

    nh.param<bool>("rtk/use_map_origin", kUseMapOrigin, false);
    nh.param<string>("rtk/frame_id", kFrameId, "camera_init");

    nh.param<std::string>("input_path", kSavePath, std::string(ROOT_DIR) + "Lsdc_Repub/");
    sub_rtk = nh.subscribe("/lsdc_rtk", 100, rtkCallback);
    pub_initial_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initial_pose", 1);
    pub_rtk_odom = nh.advertise<nav_msgs::Odometry>("/rtk_odom", 100000);
    pub_rtk_path = nh.advertise<nav_msgs::Path>("/rtk_path", 100000);
    path_msg.header.frame_id = kFrameId;

    std::cout << "[SLAM]: " << "Init rtk2pose Success" << std::endl;
    std::thread thd_pub_init_pose{pubInitPose};

    ros::spin();

    return 0;
}