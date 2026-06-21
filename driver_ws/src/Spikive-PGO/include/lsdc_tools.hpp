#ifndef LSDC_TOOLS_HPP
#define LSDC_TOOLS_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

// 四元数转欧拉角
template <typename T>
Eigen::Matrix<T, 3, 1> Q_to_EulerAngle(const Eigen::Quaternion<T>& q) {
    T roll, pitch, yaw;
    // roll (x-axis rotation)
    T sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    T cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    T sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    T siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    T cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);

    return Eigen::Matrix<T, 3, 1>(roll, pitch, yaw);
}

template <typename T, typename T1, typename T2>
void setPoseStamp(T& out, T1 t, T2 q) {
    out.pose.position.x = t.x();
    out.pose.position.y = t.y();
    out.pose.position.z = t.z();
    out.pose.orientation.x = q.x();
    out.pose.orientation.y = q.y();
    out.pose.orientation.z = q.z();
    out.pose.orientation.w = q.w();
}

struct PclOdomPair {
    sensor_msgs::PointCloud2ConstPtr pcl_msg;
    bool has_pcl{false};
    bool has_odom{false};

    Eigen::Vector3d T;
    Eigen::Matrix3d R;
    std::string signal;

    void addPclMsg(sensor_msgs::PointCloud2ConstPtr msg_in) {
        pcl_msg = msg_in;
        has_pcl = true;
    }
    void addOdomMsg(nav_msgs::Odometry::ConstPtr odom_msg) {
        has_odom = true;
        T = Eigen::Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                            odom_msg->pose.pose.position.z);
        R = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                               odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)
                .normalized()
                .toRotationMatrix();
        signal = odom_msg->child_frame_id;
    }
    void setOdom(Eigen::Vector3d T_in, Eigen::Matrix3d R_in, std::string signal_in) {
        has_odom = true;
        T = T_in;
        R = R_in;
        signal = signal_in;
    }
    bool is_completed() { return (has_pcl && has_odom); }
    ros::Time timestamp() { return pcl_msg->header.stamp; }
};

bool poseDiffThre(Eigen::Vector3d t, Eigen::Matrix3d r, double len_thre, double ang_thre) {
    if (t.norm() < len_thre && Q_to_EulerAngle(Eigen::Quaterniond(r)).norm() < ang_thre)
        return false;
    return true;
}

void split(const std::string& s,
           std::vector<std::string>& tokens,
           const std::string& delimiters = " ") {
    std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    std::string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos) {
        tokens.push_back(s.substr(lastPos, pos - lastPos));  // use emplace_back after C++11
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}

template <typename T>
double points_dist(T p1, T p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                     (p1.z - p2.z) * (p1.z - p2.z));
}

int findPoseIndexUsingTime(std::vector<double>& time_list, double& time) {
    double time_inc = 10000000000;
    int min_index = -1;
    for (size_t i = 0; i < time_list.size(); i++) {
        if (fabs(time_list[i] - time) < time_inc) {
            time_inc = fabs(time_list[i] - time);
            min_index = i;
        }
    }
    if (time_inc > 0.0001) {
        // std::string msg =
        //     "The timestamp between poses and point cloud is:" + std::to_string(time_inc) +
        //     "s. Please check it!";
        // ROS_ERROR_STREAM(msg.c_str());
        // std::cout << "Timestamp for point cloud:" << time << std::endl;
        // std::cout << "Timestamp for pose:" << time_list[min_index] << std::endl;
        // exit(-1);
        return -1;
    }
    return min_index;
}

void load_pose_with_time(const std::string& pose_file,
                         std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& poses_vec,
                         std::vector<double>& times_vec) {
    // times_vec.clear();
    // poses_vec.clear();
    std::ifstream fin(pose_file);
    std::string line;
    Eigen::Matrix<double, 1, 7> temp_matrix;
    while (getline(fin, line)) {
        std::istringstream sin(line);
        std::vector<std::string> Waypoints;
        std::string info;
        int number = 0;
        while (getline(sin, info, ' ')) {
            if (number == 0) {
                double time;
                std::stringstream data;
                data << info;
                data >> time;
                times_vec.push_back(time);
                number++;
            } else {
                double p;
                std::stringstream data;
                data << info;
                data >> p;
                temp_matrix[number - 1] = p;
                if (number == 7) {
                    Eigen::Vector3d translation(temp_matrix[0], temp_matrix[1], temp_matrix[2]);
                    Eigen::Quaterniond q(temp_matrix[6], temp_matrix[3], temp_matrix[4],
                                         temp_matrix[5]);
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
                    single_pose.first = translation;
                    single_pose.second = q.toRotationMatrix();
                    poses_vec.push_back(single_pose);
                }
                number++;
            }
        }
    }
}

void load_loop_with_time(const std::string& pose_file,
                         std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& loop_odom_vec,
                         std::vector<std::pair<double, double>>& loop_time_vec) {
    std::ifstream fin(pose_file);
    std::string line;
    Eigen::Matrix<double, 1, 7> temp_matrix;
    std::pair<double, double> time_pair;
    while (getline(fin, line)) {
        std::istringstream sin(line);
        std::vector<std::string> Waypoints;
        std::string info;
        int number = 0;
        while (getline(sin, info, ' ')) {
            if (number == 0) {
                std::stringstream data;
                data << info;
                data >> time_pair.first;
                number++;
            } else if (number == 1) {
                std::stringstream data;
                data << info;
                data >> time_pair.second;
                loop_time_vec.push_back(time_pair);
                number++;
            } else {
                double p;
                std::stringstream data;
                data << info;
                data >> p;
                temp_matrix[number - 2] = p;
                if (number == 8) {
                    Eigen::Vector3d translation(temp_matrix[0], temp_matrix[1], temp_matrix[2]);
                    Eigen::Quaterniond q(temp_matrix[6], temp_matrix[3], temp_matrix[4],
                                         temp_matrix[5]);
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
                    single_pose.first = translation;
                    single_pose.second = q.toRotationMatrix();
                    loop_odom_vec.push_back(single_pose);
                }
                number++;
            }
        }
    }
}

// pcl::PointCloud<pcl::PointXYZI>::Ptr getEdgeCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl) {
//     down_sampling_voxel(*pcl, 0.1);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::KdTreeFLANN<pcl::PointXYZI> kd_tree;
//     kd_tree.setInputCloud(pcl);

//     int search_num{10};
//     for (size_t i = 0; i < pcl->size(); i++) {
//         pcl::PointXYZI search_point = pcl->points[i];
//         std::vector<int> point_search_idx;
//         std::vector<float> point_squared_distance;
//         if (kd_tree.nearestKSearch(search_point, search_num, point_search_idx,
//                                    point_squared_distance) == search_num &&
//             point_squared_distance.back() < 1.0) {
//             Eigen::Vector3d center(0, 0, 0);
//             Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
//             std::vector<Eigen::Vector3d> near_points;
//             for (int j = 0; j < search_num; j++) {
//                 Eigen::Vector3d tmp(pcl->at(point_search_idx[j]).x, pcl->at(point_search_idx[j]).y,
//                                     pcl->at(point_search_idx[j]).z);
//                 center += tmp;
//                 near_points.push_back(tmp);
//             }
//             center /= search_num;
//             for (int j = 0; j < search_num; j++) {
//                 Eigen::Vector3d tmp = near_points[j] - center;
//                 cov_mat += tmp * tmp.transpose();
//             }

//             Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);
//             if (saes.eigenvalues()[2] > 5 * saes.eigenvalues()[1])
//                 edge_cloud->push_back(search_point);
//         }
//     }
//     return edge_cloud;
// }

#endif