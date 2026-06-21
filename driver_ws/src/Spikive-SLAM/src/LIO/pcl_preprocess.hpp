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
#include <algorithm>
#include <cmath>
#include <csignal>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include "common_lib.h"
#include "preprocess.h"

extern condition_variable sig_buffer;
extern mutex mtx_buffer;
extern deque<double> time_buffer;
extern deque<PointCloudXYZI::Ptr> lidar_buffer;
extern bool time_sync_en;
extern bool timediff_set_flg;
extern double last_timestamp_imu;
extern double timediff_lidar_wrt_imu;

namespace pcl_pre {

double init_stamp{-1};
vector<string> lidar_type_vec;
int lidar_num{1};

void fusePclMsg();

struct LidarFrameTimeInfo {
    int lidar_type{AVIA};
    bool has_point_time_range{false};
    double raw_header_time{0.0};
    double point_min_offset_ms{0.0};
    double point_max_offset_ms{0.0};
};

struct PclWithStamp {
    PointCloudXYZI::Ptr ptr;
    double stamp_bgn{0}, stamp_end{0};
    LidarFrameTimeInfo time_info;
};

class PclPreprocess {
   private:
    ros::NodeHandle nh;
    ros::Subscriber sub_pcl;
    string lid_topic;
    int kLidarNum{-1};
    string kLidarType;

    vector<double> extrinT;
    vector<double> extrinR;
    V3D T_wrt_MainLidar;
    M3D R_wrt_MainLidar;

    double last_timestamp_lidar{-1};
    shared_ptr<Preprocess> p_pre;

    void transPclToMainLidar(const PointCloudXYZI::Ptr& ptr) {
        for (int i = 0; i < ptr->points.size(); i++) {
            auto& pt = ptr->points[i];
            Eigen::Vector3d pt_vec(pt.x, pt.y, pt.z);
            pt_vec = R_wrt_MainLidar * pt_vec + T_wrt_MainLidar;
            pt.x = pt_vec.x();
            pt.y = pt_vec.y();
            pt.z = pt_vec.z();
        }
    }

    LidarFrameTimeInfo buildFrameTimeInfo(double raw_header_time,
                                          const PointCloudXYZI::Ptr& ptr) const {
        LidarFrameTimeInfo info;
        info.lidar_type = p_pre->lidar_type;
        info.raw_header_time = raw_header_time;

        if (!ptr || ptr->empty())
            return info;

        double min_offset_ms = std::numeric_limits<double>::infinity();
        double max_offset_ms = -std::numeric_limits<double>::infinity();
        for (const auto& pt : ptr->points) {
            if (!std::isfinite(pt.curvature))
                continue;
            min_offset_ms = std::min(min_offset_ms, static_cast<double>(pt.curvature));
            max_offset_ms = std::max(max_offset_ms, static_cast<double>(pt.curvature));
        }

        if (!std::isfinite(min_offset_ms) || !std::isfinite(max_offset_ms))
            return info;

        info.has_point_time_range = true;
        info.point_min_offset_ms = min_offset_ms;
        info.point_max_offset_ms = max_offset_ms;
        return info;
    }

    void updateLidarImuTimeDiff(double lidar_stamp, double lidar_scan_time) {
        if (!time_sync_en || timediff_set_flg || last_timestamp_imu <= 0.0)
            return;
        if (std::abs(lidar_stamp - last_timestamp_imu) <= 1.0)
            return;

        timediff_set_flg = true;
        timediff_lidar_wrt_imu = lidar_stamp + lidar_scan_time - last_timestamp_imu;
        ROS_WARN("Self sync IMU and LiDAR, time diff is %.10lf", timediff_lidar_wrt_imu);
    }

    void livoxPclCbk(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        if (init_stamp == -1)
            init_stamp = msg->header.stamp.toSec();
        // cout << kLidarType << ": " << msg->header.stamp.toSec() - init_stamp << "\n";

        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            if (msg->header.stamp.toSec() < last_timestamp_lidar) {
                ROS_ERROR("lidar loop back, clear buffer");
                // lidar_deq.clear();
                pws_deq.clear();
            }
            last_timestamp_lidar = msg->header.stamp.toSec();

            PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
            p_pre->process(msg, ptr);

            if (!ptr)
                return;
            transPclToMainLidar(ptr);
            const auto time_info = buildFrameTimeInfo(last_timestamp_lidar, ptr);
            const double max_offset_ms =
                time_info.has_point_time_range ? std::max(0.0, time_info.point_max_offset_ms) : 0.0;
            double curr_stamp_end = !ptr->points.empty()
                                        ? last_timestamp_lidar + max_offset_ms / 1000
                                        : last_timestamp_lidar;
            updateLidarImuTimeDiff(last_timestamp_lidar, curr_stamp_end - last_timestamp_lidar);
            PclWithStamp pws = {
                .ptr = ptr, .stamp_bgn = last_timestamp_lidar, .stamp_end = curr_stamp_end};
            pws.time_info = time_info;
            pws_deq.push_back(pws);
            // lidar_deq.push_back(ptr);
            // time_deq.push_back(last_timestamp_lidar);

            fusePclMsg();
        }

        sig_buffer.notify_all();
    }

    void standardPclCbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (init_stamp == -1)
            init_stamp = msg->header.stamp.toSec();

        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            if (msg->header.stamp.toSec() < last_timestamp_lidar) {
                ROS_ERROR("lidar loop back, clear buffer");
                pws_deq.clear();
            }
            last_timestamp_lidar = msg->header.stamp.toSec();

            PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
            p_pre->process(msg, ptr);

            if (!ptr)
                return;
            transPclToMainLidar(ptr);
            const auto time_info = buildFrameTimeInfo(last_timestamp_lidar, ptr);
            if (p_pre->lidar_type == S10U &&
                (!time_info.has_point_time_range || time_info.point_max_offset_ms <= 0.0)) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "Drop S10U cloud without valid positive point timestamps after preprocessing.");
                return;
            }
            const double max_offset_ms =
                time_info.has_point_time_range ? std::max(0.0, time_info.point_max_offset_ms) : 0.0;
            double curr_stamp_end = !ptr->points.empty()
                                        ? last_timestamp_lidar + max_offset_ms / 1000
                                        : last_timestamp_lidar;
            updateLidarImuTimeDiff(last_timestamp_lidar, curr_stamp_end - last_timestamp_lidar);
            PclWithStamp pws = {
                .ptr = ptr, .stamp_bgn = last_timestamp_lidar, .stamp_end = curr_stamp_end};
            pws.time_info = time_info;
            pws_deq.push_back(pws);

            fusePclMsg();
        }

        sig_buffer.notify_all();
    }

   public:
    deque<PclWithStamp> pws_deq;

    PclPreprocess(int num) {
        kLidarNum = num;
        kLidarType = lidar_type_vec[kLidarNum];
        cout << "pcl preprocess: [" << kLidarType << "]\n";

        p_pre = shared_ptr<Preprocess>(new Preprocess());

        string tmp = kLidarType + "_";
        nh.param<string>(tmp + "common/lid_topic", lid_topic, "/livox/lidar_360");
        nh.param<double>(tmp + "preprocess/blind", p_pre->blind, 2);
        nh.param<int>(tmp + "preprocess/lidar_type", p_pre->lidar_type, AVIA);
        nh.param<int>(tmp + "preprocess/scan_line", p_pre->N_SCANS, 6);
        nh.param<int>(tmp + "preprocess/timestamp_unit", p_pre->time_unit, US);
        nh.param<int>(tmp + "preprocess/scan_rate", p_pre->SCAN_RATE, 10);
        nh.param<int>(tmp + "preprocess/point_filter_num", p_pre->point_filter_num, 2);
        nh.param<bool>(tmp + "preprocess/feature_extract_enable", p_pre->feature_enabled, false);
        nh.param<bool>(tmp + "preprocess/fov_crop_enable", p_pre->s10u_fov_crop_enable, false);
        nh.param<double>(tmp + "preprocess/vertical_fov_degree",
                         p_pre->s10u_vertical_fov_degree, 80.0);
        nh.param<double>(tmp + "preprocess/horizontal_fov_degree",
                         p_pre->s10u_horizontal_fov_degree, 120.0);
        nh.param<vector<double>>(tmp + "mapping/extrinsic_T", extrinT, vector<double>({0, 0, 0}));
        nh.param<vector<double>>(tmp + "mapping/extrinsic_R", extrinR,
                                 vector<double>({1, 0, 0, 0, 1, 0, 0, 0, 1}));
        T_wrt_MainLidar << VEC_FROM_ARRAY(extrinT);
        R_wrt_MainLidar << MAT_FROM_ARRAY(extrinR);

        if (p_pre->lidar_type == AVIA) {
            sub_pcl = nh.subscribe(lid_topic, 200000, &PclPreprocess::livoxPclCbk, this);
        } else {
            sub_pcl = nh.subscribe(lid_topic, 200000, &PclPreprocess::standardPclCbk, this);
        }
    }

    ~PclPreprocess() {}
};
vector<PclPreprocess*> pcl_pre_vec;
deque<LidarFrameTimeInfo> frame_time_buffer;

PointCloudXYZI::Ptr ptr_fusion(new PointCloudXYZI());
void fusePclMsg() {
    if (pcl_pre_vec.empty())
        ROS_WARN("Pcl Msg Wrong!\n");
    for (int lidar_count = 0; lidar_count < lidar_num; lidar_count++)
        if ((pcl_pre_vec[lidar_count]->pws_deq).empty())
            return;

    auto& main_frame = (pcl_pre_vec[0]->pws_deq).front();
    double stamp_bgn = main_frame.stamp_bgn;
    double stamp_end = main_frame.stamp_end;
    // cout << stamp_bgn - init_stamp << "  " << stamp_end - init_stamp << "\n";
    for (int lidar_count = 1; lidar_count < lidar_num; lidar_count++)
        if (((pcl_pre_vec[lidar_count]->pws_deq).back()).stamp_end < stamp_end)
            return;

    ptr_fusion->clear();
    *ptr_fusion = *(main_frame.ptr);  // 注释掉后不使用主雷达点云
    double last_stamp_end{-1};
    for (int lidar_count = 1; lidar_count < lidar_num; lidar_count++) {
        while (!(pcl_pre_vec[lidar_count]->pws_deq).empty()) {
            auto& curr_frame = (pcl_pre_vec[lidar_count]->pws_deq).front();
            for (int i = 0; i < curr_frame.ptr->size(); i++) {
                auto& pt = curr_frame.ptr->points[i];
                if (curr_frame.stamp_bgn + pt.curvature / 1000 >= stamp_bgn &&
                    curr_frame.stamp_bgn + pt.curvature / 1000 <= stamp_end)
                    (ptr_fusion->points).push_back(pt);
            }
            if (curr_frame.stamp_end <= stamp_end)
                (pcl_pre_vec[lidar_count]->pws_deq).pop_front();
            else {
                last_stamp_end =
                    last_stamp_end > curr_frame.stamp_end ? last_stamp_end : curr_frame.stamp_end;
                break;
            }
        }
    }
    time_buffer.push_back(stamp_bgn);
    if (main_frame.time_info.lidar_type == S10U) {
        lidar_buffer.push_back(PointCloudXYZI::Ptr(new PointCloudXYZI(*ptr_fusion)));
    } else {
        lidar_buffer.push_back(ptr_fusion);
    }
    frame_time_buffer.push_back(main_frame.time_info);
    (pcl_pre_vec[0]->pws_deq).pop_front();

    // cout << "main_stamp_end: " << stamp_end - init_stamp
    //      << "    last_stamp_end: " << last_stamp_end - init_stamp
    //      << "    delay_time: " << last_stamp_end - stamp_end << "\n";
}

void split(const string& s, vector<string>& tokens, const string& delimiters = " ") {
    string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (string::npos != pos || string::npos != lastPos) {
        tokens.push_back(s.substr(lastPos, pos - lastPos));  // use emplace_back after C++11
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}
void initPclPrepreocess(const string& lidar_types) {
    split(lidar_types, lidar_type_vec, "-");
    lidar_num = lidar_type_vec.size();
    for (int i = 0; i < lidar_num; i++) {
        PclPreprocess* pp = new PclPreprocess(i);
        pcl_pre_vec.push_back(pp);
    }
}
};  // namespace pcl_pre
