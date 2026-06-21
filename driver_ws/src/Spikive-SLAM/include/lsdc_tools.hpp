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

#include "lsdc_math.hpp"

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

class VOXEL_LOC {
   public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC& other) const {
        return (x == other.x && y == other.y && z == other.z);
    }
};

// for down sample function
struct M_POINT {
    float xyz[3];
    float intensity;
    int count = 0;
};

#define HASH_P 116101
#define MAX_N 10000000000
#define MAX_FRAME_N 20000
template <>
struct std::hash<VOXEL_LOC> {
    int64_t operator()(const VOXEL_LOC& s) const {
        using std::hash;
        using std::size_t;
        return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    }
};

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI>& pl_feat, double voxel_size) {
    int intensity = rand() % 255;
    if (voxel_size < 0.01) {
        return;
    }
    std::unordered_map<VOXEL_LOC, M_POINT> voxel_map;
    uint plsize = pl_feat.size();

    for (uint i = 0; i < plsize; i++) {
        pcl::PointXYZI& p_c = pl_feat[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_c.data[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            iter->second.xyz[0] += p_c.x;
            iter->second.xyz[1] += p_c.y;
            iter->second.xyz[2] += p_c.z;
            iter->second.intensity += p_c.intensity;
            iter->second.count++;
        } else {
            M_POINT anp;
            anp.xyz[0] = p_c.x;
            anp.xyz[1] = p_c.y;
            anp.xyz[2] = p_c.z;
            anp.intensity = p_c.intensity;
            anp.count = 1;
            voxel_map[position] = anp;
        }
    }
    plsize = voxel_map.size();
    pl_feat.clear();
    pl_feat.resize(plsize);

    uint i = 0;
    for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
        pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
        pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
        pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
        pl_feat[i].intensity = iter->second.intensity / iter->second.count;
        i++;
    }
}

#endif