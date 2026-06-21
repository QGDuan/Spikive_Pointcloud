#include <geometry_msgs/PoseStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <thread>
#include "file_preprocess.hpp"
#include "include/STDesc.h"
#include "include/lsdc_tools.hpp"

bool kRunPgo;
bool kUseLoop, kUseGps, kUseFile;
double kMapResolution;

ConfigSetting config_setting;
std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> key_poses_vec;
std::vector<double> times_vec;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vec;

STDescManager* std_manager;
gtsam::Values initial;
gtsam::NonlinearFactorGraph graph;
gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
gtsam::noiseModel::Base::shared_ptr robustLoopNoise, newLaunchNoise;

gtsam::ISAM2* isam{nullptr};

size_t cloudInd{0};
size_t key_frame_index{0};
size_t keyCloudInd{0};
size_t loop_factor_count{0};
size_t rejected_loop_factor_count{0};
int max_loop_factor_count{0};
pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr pgo_map_pcl(new pcl::PointCloud<pcl::PointXYZI>());
std::vector<double> descriptor_time;
std::vector<double> querying_time;
std::vector<double> update_time;
int triggle_loop_num{0};

ros::Publisher pubOdomAftMapped, pubCureentCloud, pubCurrentCorner, pubMatchedCloud,
    pubMatchedCorner, pubSTD, pubCorrectCloud, pubOdomCorreted, pubPgoMap, pubCurrCorrectCloud,
    pub_lio_path, pub_pgo_path, pub_gps_path, pub_gps_points, pub_gps_use_points;
nav_msgs::Path lio_path, pgo_path, gps_path;
nav_msgs::Odometry odom;

double kUpdateGraphFreq{1.0}, kKeyFrameLenThre{0.1}, kKeyFrameAngThre{0.1};
double kGpsWaitingTime{10.0}, kGpsSpacingDistance{5.0}, kGpsSpacingTime{2.0};
std::mutex mutex_graph, mutex_msg, mutex_data;
Eigen::Vector3d diff_T = Eigen::Vector3d(0, 0, 0);
Eigen::Matrix3d diff_R = Eigen::Matrix3d::Identity();
bool real_time_en, use_key_frame;
std::string save_pose_path, save_loop_path;
std::string input_path, input_bag_names, input_pose_names, input_loop_names;
std::string file_cloud_topic;
std::ofstream save_pose_file, save_loop_file;

std::map<double, std::shared_ptr<PclOdomPair>>
    time_to_popair_map;  // 时间戳->该时间戳对应的消息结构体的映射
std::queue<std::shared_ptr<PclOdomPair>> popair_que;  // 消息结构体队列
std::queue<nav_msgs::Odometry> gps_msg_que;

void pubSourceMsg(const PclOdomPair& popair) {
    Eigen::Vector3d translation = popair.T;
    Eigen::Matrix3d rotation = Eigen::Quaterniond(popair.R).normalized().toRotationMatrix();

    pcl::PointCloud<pcl::PointXYZI> curr_cloud;
    pcl::fromROSMsg(*(popair.pcl_msg), curr_cloud);

    // 发布原odom和path
    static int jjj{0};
    if (++jjj % 10 == 0) {
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = "camera_init";
        setPoseStamp(pose_stamp, translation, Eigen::Quaterniond(rotation));
        lio_path.poses.push_back(pose_stamp);
        pub_lio_path.publish(lio_path);
    }

    // 发布当前帧点云
    pcl::PointCloud<pcl::PointXYZI> curr_correct_cloud;
    Eigen::Vector3d est_T = translation + rotation * diff_T;
    Eigen::Matrix3d est_R = rotation * diff_R;
    for (size_t j = 0; j < curr_cloud.size(); j += 10) {
        pcl::PointXYZI pi = curr_cloud.points[j];
        Eigen::Vector3d pv(pi.x, pi.y, pi.z);
        pv = est_R * pv + est_T;
        pi.x = pv[0];
        pi.y = pv[1];
        pi.z = pv[2];
        curr_correct_cloud.push_back(pi);
    }

    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(curr_correct_cloud, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubCurrCorrectCloud.publish(pub_cloud);
}

std::mutex mutex_buf, mutex_gps;
std::shared_ptr<PclOdomPair> getMsgFromTime(const double& time_stamp) {
    auto iter = time_to_popair_map.find(time_stamp);
    if (iter == time_to_popair_map.end()) {
        std::shared_ptr<PclOdomPair> popair_ptr(new PclOdomPair());
        // PclOdomPair* popair_ptr = new PclOdomPair();
        time_to_popair_map.insert(std::make_pair(time_stamp, popair_ptr));
        return popair_ptr;
    }
    return iter->second;
}

void pclCbk(const sensor_msgs::PointCloud2ConstPtr& msg_in) {
    mutex_buf.lock();
    auto msgpair_ptr = getMsgFromTime(msg_in->header.stamp.toSec());
    msgpair_ptr->addPclMsg(msg_in);
    if (msgpair_ptr->is_completed()) {
        pubSourceMsg(*msgpair_ptr);
        mutex_msg.lock();
        popair_que.push(msgpair_ptr);
        mutex_msg.unlock();
    }
    mutex_buf.unlock();
}

void odomCbk(const nav_msgs::Odometry::ConstPtr& msg_in) {
    mutex_buf.lock();
    auto msgpair_ptr = getMsgFromTime(msg_in->header.stamp.toSec());
    msgpair_ptr->addOdomMsg(msg_in);
    if (msgpair_ptr->is_completed()) {
        pubSourceMsg(*msgpair_ptr);
        mutex_msg.lock();
        popair_que.push(msgpair_ptr);
        mutex_msg.unlock();
    }
    mutex_buf.unlock();
}

bool judge_gps_point(const nav_msgs::Odometry& msg_in) {
    static Eigen::Vector3d last_gps_pose_tmp;
    static double last_gps_stamp_tmp{-1};
    Eigen::Vector3d gps_pose_tmp(msg_in.pose.pose.position.x, msg_in.pose.pose.position.y,
                                 msg_in.pose.pose.position.z);
    double gps_stamp_tmp = msg_in.header.stamp.toSec();

    if (std::isnan(gps_pose_tmp.x()) || std::isnan(gps_pose_tmp.y()) ||
        std::isnan(gps_pose_tmp.z()))
        return false;

    if (std::isinf(gps_pose_tmp.x()) || std::isinf(gps_pose_tmp.y()) ||
        std::isinf(gps_pose_tmp.z()))
        return false;

    // 两gps位置算出速度大于30m/s,错误数据删除
    if (last_gps_stamp_tmp > 0)
        if ((gps_pose_tmp - last_gps_pose_tmp).norm() / (gps_stamp_tmp - last_gps_stamp_tmp) > 30.0)
            return false;

    last_gps_stamp_tmp = gps_stamp_tmp;
    last_gps_pose_tmp = gps_pose_tmp;
    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr gps_points(new pcl::PointCloud<pcl::PointXYZI>());
// 在状态ok之后一定时间后才认为是真的ok
bool curr_gps_status{true}, converting_to_ok{false};
double converting_to_ok_stamp{-1};

void gpsPreprocess(const nav_msgs::Odometry gps_in) {
    if (!judge_gps_point(gps_in))
        return;

    if (gps_in.child_frame_id == "OK") {
        pcl::PointXYZI pt;
        pt.x = gps_in.pose.pose.position.x;
        pt.y = gps_in.pose.pose.position.y;
        pt.z = gps_in.pose.pose.position.z;
        pt.intensity = 255;

        if (curr_gps_status) {
            mutex_gps.lock();
            gps_msg_que.push(gps_in);
            mutex_gps.unlock();
        } else {
            if (!converting_to_ok) {
                converting_to_ok = true;
                converting_to_ok_stamp = gps_in.header.stamp.toSec();
            } else {
                if (gps_in.header.stamp.toSec() - converting_to_ok_stamp > kGpsWaitingTime) {
                    curr_gps_status = true;
                    converting_to_ok = false;
                }
            }
            pt.intensity = 0;
        }

        if (gps_points->empty() || points_dist(pt, gps_points->back()) > 1.0) {
            gps_points->push_back(pt);

            sensor_msgs::PointCloud2 pub_cloud;
            pcl::toROSMsg(*gps_points, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pub_gps_points.publish(pub_cloud);
        }
    } else {
        curr_gps_status = false;
        converting_to_ok = false;
    }

    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = "camera_init";
    pose_stamp.pose.position = gps_in.pose.pose.position;
    pose_stamp.pose.orientation = gps_in.pose.pose.orientation;
    if (gps_path.poses.empty() ||
        points_dist(pose_stamp.pose.position, gps_path.poses.back().pose.position) > 1.0) {
        gps_path.poses.push_back(pose_stamp);
        pub_gps_path.publish(gps_path);
    }
}

void gpsCbk(const nav_msgs::Odometry::ConstPtr& msg_in) {
    gpsPreprocess(*msg_in);
}

Eigen::Vector3d last_translation = Eigen::Vector3d(0, 0, 0);
Eigen::Matrix3d last_rotation = Eigen::Matrix3d::Identity();
bool is_new_launch{false};
std::string last_signal{""};
int new_launch_count{0};
bool msgpairPreprocess(std::shared_ptr<PclOdomPair> popair) {
    double laser_time = popair->timestamp().toSec();

    Eigen::Vector3d translation = popair->T;
    Eigen::Matrix3d rotation = popair->R;

    // 若是新的线路slam，标记
    std::string curr_signal = popair->signal;
    if (last_signal != curr_signal) {
        is_new_launch = true;
        last_signal = curr_signal;
        diff_T = Eigen::Vector3d(0, 0, 0);
        diff_R = Eigen::Matrix3d::Identity();
        new_launch_count = 0;
        // std::cout << "[New Launch]: " << curr_signal << std::endl;
    } else
        is_new_launch = false;

    if (use_key_frame) {
        bool is_key_frame = (is_new_launch || poseDiffThre(translation - last_translation,
                                                           rotation * last_rotation.transpose(),
                                                           kKeyFrameLenThre, kKeyFrameAngThre));
        if (!is_key_frame)
            return false;
        last_translation = translation;
        last_rotation = rotation;
    }
    new_launch_count++;

    mutex_data.lock();
    poses_vec.push_back(std::make_pair(translation, rotation));
    times_vec.push_back(laser_time);

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*(popair->pcl_msg), cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (size_t i = 0; i < cloud.size(); i++) {
        current_cloud->push_back(cloud.points[i]);
        Eigen::Vector3d pv = point2vec(cloud.points[i]);
        pv = rotation * pv + translation;
        cloud.points[i] = vec2point(pv);
    }

    down_sampling_voxel(cloud, config_setting.ds_size_);
    cloud_vec.push_back(current_cloud);
    for (auto pv : cloud.points) {
        temp_cloud->points.push_back(pv);
    }

    mutex_data.unlock();
    return true;
}

void addInitFactor() {
    mutex_graph.lock();
    initial.insert(0, gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                   gtsam::Point3(poses_vec[cloudInd].first)));
    graph.add(
        gtsam::PriorFactor<gtsam::Pose3>(0,
                                         gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                                      gtsam::Point3(poses_vec[cloudInd].first)),
                                         odometryNoise));
    mutex_graph.unlock();
}

void addNearFactor(bool is_new_launch) {
    mutex_graph.lock();
    initial.insert(cloudInd, gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                          gtsam::Point3(poses_vec[cloudInd].first)));
    // add connection between near frame
    Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
    Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

    t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
    R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

    gtsam::Rot3 R_sam(R_ab);
    gtsam::Point3 t_sam(t_ab);

    if (is_new_launch) {
        gtsam::NonlinearFactor::shared_ptr near_factor(new gtsam::BetweenFactor<gtsam::Pose3>(
            cloudInd - 1, cloudInd, gtsam::Pose3(R_sam, t_sam), newLaunchNoise));
        graph.push_back(near_factor);
    } else {
        gtsam::NonlinearFactor::shared_ptr near_factor(new gtsam::BetweenFactor<gtsam::Pose3>(
            cloudInd - 1, cloudInd, gtsam::Pose3(R_sam, t_sam), odometryNoise));
        graph.push_back(near_factor);
    }
    mutex_graph.unlock();
}

void addLoopFactor() {
    // mutex_graph.lock();
    // use first frame's pose as key pose
    key_poses_vec.push_back(poses_vec[cloudInd - config_setting.sub_frame_num_]);
    // std::cout << "Key Frame id:" << keyCloudInd
    //           << ", cloud size: " << temp_cloud->size() << std::endl;
    // step1. Descriptor Extraction
    auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
    std::vector<STDesc> stds_vec;
    std_manager->GenerateSTDescs(temp_cloud, stds_vec);
    auto t_descriptor_end = std::chrono::high_resolution_clock::now();
    descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));
    if (keyCloudInd % 10 == 0) {
        ROS_INFO_STREAM("STD key group " << keyCloudInd << ": descriptors=" << stds_vec.size()
                                         << ", cloud_points=" << temp_cloud->size());
    }

    // step2. Searching Loop
    auto t_query_begin = std::chrono::high_resolution_clock::now();
    std::pair<int, double> search_result(-1, 0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    loop_transform.first << 0, 0, 0;
    loop_transform.second = Eigen::Matrix3d::Identity();
    std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
    if (keyCloudInd > config_setting.skip_near_num_) {
        std_manager->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair);
    }
    if (search_result.first > 0) {
        ROS_INFO_STREAM("[Loop Detection] trigger loop: " << keyCloudInd << "--"
                        << search_result.first << ", score=" << search_result.second);
    }
    auto t_query_end = std::chrono::high_resolution_clock::now();
    querying_time.push_back(time_inc(t_query_end, t_query_begin));

    // step3. Add descriptors to the database
    auto t_map_update_begin = std::chrono::high_resolution_clock::now();
    std_manager->AddSTDescs(stds_vec);
    auto t_map_update_end = std::chrono::high_resolution_clock::now();
    update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
    // std::cout << "[Time] descriptor extraction: "
    //           << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
    //           << "query: " << time_inc(t_query_end, t_query_begin) << "ms, "
    //           << "update map:" << time_inc(t_map_update_end,
    //           t_map_update_begin)
    //           << "ms" << std::endl;
    // std::cout << std::endl;

    pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
    save_key_cloud = *temp_cloud;
    down_sampling_voxel(save_key_cloud, 0.2);
    std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

    // publish
    if (kRunPgo) {
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(*temp_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCureentCloud.publish(pub_cloud);
        pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCurrentCorner.publish(pub_cloud);
    }

    if (search_result.first > 0) {
        triggle_loop_num++;
        int match_frame = search_result.first;
        // obtain optimal transform
        std_manager->PlaneGeomrtricIcp(std_manager->plane_cloud_vec_.back(),
                                       std_manager->plane_cloud_vec_[match_frame], loop_transform);

        // add connection between loop frame.
        // e.g. if src_key_frame_id 5 with sub frames 51~60 triggle loop with
        //     tar_key_frame_id 1 with sub frames 11~20, add connection between
        // each sub frame, 51-11, 52-12,...,60-20.

        mutex_graph.lock();
        int sub_frame_num = config_setting.sub_frame_num_;
        for (size_t j = 1; j <= sub_frame_num; j++) {
            int src_frame = cloudInd + j - sub_frame_num;
            int tar_frame = match_frame * sub_frame_num + j;
            if (src_frame < 0 || tar_frame < 0 || src_frame >= poses_vec.size() ||
                tar_frame >= poses_vec.size() || src_frame == tar_frame) {
                rejected_loop_factor_count++;
                ROS_WARN_STREAM("Reject loop factor with invalid indices: keyCloudInd="
                                << keyCloudInd << ", match_frame=" << match_frame
                                << ", src_frame=" << src_frame << ", tar_frame=" << tar_frame
                                << ", poses=" << poses_vec.size());
                continue;
            }
            Eigen::Matrix3d src_R = loop_transform.second * poses_vec[src_frame].second;
            Eigen::Vector3d src_t =
                loop_transform.second * poses_vec[src_frame].first + loop_transform.first;
            Eigen::Matrix3d tar_R = poses_vec[tar_frame].second;
            Eigen::Vector3d tar_t = poses_vec[tar_frame].first;

            Eigen::Vector3d tmp_T = tar_R.transpose() * (src_t - tar_t);
            Eigen::Matrix3d tmp_R = tar_R.transpose() * src_R;
            if (!tmp_T.allFinite() || !tmp_R.allFinite() ||
                fabs(tmp_R.determinant() - 1.0) > 1e-3) {
                rejected_loop_factor_count++;
                ROS_WARN_STREAM("Reject loop factor with invalid transform: keyCloudInd="
                                << keyCloudInd << ", match_frame=" << match_frame
                                << ", src_frame=" << src_frame << ", tar_frame=" << tar_frame
                                << ", det=" << tmp_R.determinant() << ", t_norm="
                                << tmp_T.norm());
                continue;
            }
            if (max_loop_factor_count > 0 &&
                loop_factor_count >= static_cast<size_t>(max_loop_factor_count)) {
                rejected_loop_factor_count++;
                continue;
            }
            gtsam::Point3 ttem(tmp_T);
            gtsam::Rot3 Rtem(tmp_R);
            gtsam::NonlinearFactor::shared_ptr loop_factor(new gtsam::BetweenFactor<gtsam::Pose3>(
                tar_frame, src_frame, gtsam::Pose3(Rtem, ttem), robustLoopNoise));
            graph.push_back(loop_factor);
            loop_factor_count++;
            if (1) {
                Eigen::Quaterniond tmp_Q = Eigen::Quaterniond(tmp_R);
                save_loop_file << std::fixed << std::setprecision(20) << times_vec[tar_frame] << " "
                               << times_vec[src_frame] << " " << tmp_T.x() << " " << tmp_T.y()
                               << " " << tmp_T.z() << " " << tmp_Q.x() << " " << tmp_Q.y() << " "
                               << tmp_Q.z() << " " << tmp_Q.w() << std::endl;
            }
        }
        mutex_graph.unlock();

        // pcl::PointCloud<pcl::PointXYZI> correct_cloud;
        // pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first], pub_cloud);
        // pub_cloud.header.frame_id = "camera_init";
        // pubMatchedCloud.publish(pub_cloud);
        // pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
        // pub_cloud); pub_cloud.header.frame_id = "camera_init";
        // pubMatchedCorner.publish(pub_cloud);
        publish_std_pairs(loop_std_pair, pubSTD);
    }
    temp_cloud->clear();
    keyCloudInd++;
    // mutex_graph.unlock();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr gps_use_points(new pcl::PointCloud<pcl::PointXYZI>());
Eigen::Vector3d last_gps_pose;
double cumulative_distance{-1.0}, last_gps_stamp{-1.0};
void addGpsFactor(double curr_stamp) {
    mutex_gps.lock();
    while (!gps_msg_que.empty()) {
        if (gps_msg_que.front().header.stamp.toSec() < curr_stamp - 0.05)
            gps_msg_que.pop();
        else if (gps_msg_que.front().header.stamp.toSec() > curr_stamp + 0.05)
            break;
        else {
            nav_msgs::Odometry gps_msg = gps_msg_que.front();
            double d_time{1.0};
            while (!gps_msg_que.empty()) {
                if (std::abs(gps_msg_que.front().header.stamp.toSec() - curr_stamp) < d_time) {
                    d_time = std::abs(gps_msg_que.front().header.stamp.toSec() - curr_stamp);
                    gps_msg = gps_msg_que.front();
                    gps_msg_que.pop();
                } else
                    break;
            }
            Eigen::Vector3d gps_pose(gps_msg.pose.pose.position.x, gps_msg.pose.pose.position.y,
                                     gps_msg.pose.pose.position.z);

            // gps每累计走5m或者时间间隔大于2s，加入一次
            cumulative_distance += (gps_pose - last_gps_pose).norm();
            last_gps_pose = gps_pose;
            if (last_gps_stamp > 0 &&
                (cumulative_distance < kGpsSpacingDistance &&
                 gps_msg.header.stamp.toSec() - last_gps_stamp < kGpsSpacingTime))
                break;
            cumulative_distance = 0;
            last_gps_stamp = gps_msg.header.stamp.toSec();

            // std::cout << gps_pose.transpose() << "\n";

            // 可视化加入的gps因子
            pcl::PointXYZI pt;
            pt.x = gps_pose.x();
            pt.y = gps_pose.y();
            pt.z = gps_pose.z();
            gps_use_points->push_back(pt);
            sensor_msgs::PointCloud2 pub_cloud;
            pcl::toROSMsg(*gps_use_points, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pub_gps_use_points.publish(pub_cloud);

            // 加入gps因子
            mutex_graph.lock();
            gtsam::Vector Vector3(3);
            Vector3 << 1.0, 1.0, 1.0;
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
                gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor::shared_ptr gps_factor(new gtsam::GPSFactor(
                cloudInd, gtsam::Point3(gps_pose.x(), gps_pose.y(), gps_pose.z()), gps_noise));
            graph.push_back(gps_factor);
            mutex_graph.unlock();
            break;
        }
    }
    mutex_gps.unlock();
}

void saveGraphFactorMain(double curr_stamp) {
    // 更新odom和loop节点------------------------------------------------------------------------
    if (cloudInd == 0) {
        addInitFactor();
    } else {
        addNearFactor(is_new_launch);
        if (kUseLoop && cloudInd % config_setting.sub_frame_num_ == 0) {
            addLoopFactor();
        }
    }
    if (kUseGps)
        addGpsFactor(curr_stamp);

    cloudInd++;
}

void saveGraphFactor() {
    while (ros::ok()) {
        ros::Rate(5000).sleep();

        auto t_check_std_begin = std::chrono::high_resolution_clock::now();

        if (popair_que.empty())
            continue;

        // 读入odom和cloud信息---------------------------------------------------------------------
        mutex_msg.lock();
        auto popair = popair_que.front();
        popair_que.pop();
        mutex_msg.unlock();

        double curr_stamp = popair->timestamp().toSec();
        bool is_key_frame = msgpairPreprocess(popair);

        // 加入因子
        if (is_key_frame)
            saveGraphFactorMain(curr_stamp);

        auto t_check_std_end = std::chrono::high_resolution_clock::now();
        double d_time = time_inc(t_check_std_end, t_check_std_begin) / 1000.0;
        // if (d_time > 0.1 * config_setting.sub_frame_num_)
        //     ROS_WARN_STREAM("[Std Check Time " << std::setfill('0') << std::setw(5) <<
        //     cloudInd
        //                                        << "]: " << std::setprecision(3) << d_time <<
        //                                        "s");
        // else
        //     std::cout << "[Std Check Time " << std::setfill('0') << std::setw(5) << cloudInd
        //               << "]: " << std::setprecision(3) << d_time << "s" << std::endl;
    }
}

void pubUpdateResults(gtsam::Values& results, double map_resolution = 0.5) {
    mutex_data.lock();
    pgo_map_pcl->clear();
    pgo_path.poses.clear();
    save_pose_file.open(save_pose_path);
    int results_size = results.size();
    for (size_t i = 0; i < results_size; i++) {
        gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
        Eigen::Vector3d opt_translation = pose.translation();
        Eigen::Quaterniond opt_q(pose.rotation().matrix());

        if (1) {
            save_pose_file << std::fixed << std::setprecision(20) << times_vec[i] << " "
                           << opt_translation.x() << " " << opt_translation.y() << " "
                           << opt_translation.z() << " " << opt_q.x() << " " << opt_q.y() << " "
                           << opt_q.z() << " " << opt_q.w() << std::endl;
        }

        // 记录最后一帧odom准备发布
        if (i == results_size - 1) {
            setPoseStamp(odom.pose, opt_translation, opt_q);
            if (i < poses_vec.size()) {
                Eigen::Vector3d curr_T = poses_vec[i].first;
                Eigen::Matrix3d curr_R = poses_vec[i].second;
                diff_R = curr_R.transpose() * opt_q;
                diff_T = curr_R.transpose() * (opt_translation - curr_T);
            }
        }

        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = "camera_init";
        setPoseStamp(pose_stamp, opt_translation, opt_q);
        // 每行走相距1米加入path和点云
        if (kRunPgo || pgo_path.poses.empty() ||
            points_dist(pose_stamp.pose.position, pgo_path.poses.back().pose.position) > 5.0) {
            pgo_path.poses.push_back(pose_stamp);  // 加入path
            // 加入点云
            pcl::PointCloud<pcl::PointXYZI> correct_cloud;
            if (i < cloud_vec.size()) {
                for (size_t j = 0; j < cloud_vec[i]->size(); j++) {
                    pcl::PointXYZI pi = cloud_vec[i]->points[j];
                    Eigen::Vector3d pv(pi.x, pi.y, pi.z);
                    pv = opt_q * pv + opt_translation;
                    pi.x = pv[0];
                    pi.y = pv[1];
                    pi.z = pv[2];
                    correct_cloud.push_back(pi);
                }
            }

            *pgo_map_pcl += correct_cloud;
            // *pgo_map_pcl += *getEdgeCloud(correct_cloud.makeShared());
        }

        if (kRunPgo) {
            std::cout << "\r";  // 回到行首位置
            std::cout << "[Build Map]: " << std::setw(6) << std::fixed << std::setprecision(2)
                      << 100.0 * (i + 1) / results_size << "%";
        }
    }
    save_pose_file.close();
    pubOdomCorreted.publish(odom);
    pub_pgo_path.publish(pgo_path);

    // 发布当前地图
    sensor_msgs::PointCloud2 pub_cloud;
    down_sampling_voxel(*pgo_map_pcl, map_resolution);
    pcl::toROSMsg(*pgo_map_pcl, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubPgoMap.publish(pub_cloud);

    mutex_data.unlock();
}

void updateIsamGraph() {
    while (ros::ok()) {
        ros::Rate(kUpdateGraphFreq).sleep();
        if (cloudInd > 5) {
            // 计算更新graph------------------------------------------------------------------------
            auto t_pgo_begin = std::chrono::high_resolution_clock::now();

            mutex_graph.lock();
            isam->update(graph, initial);
            for (int i = 0; i < 20; i++)
                isam->update();
            // 清空因子图
            graph.resize(0);
            initial.clear();
            gtsam::Values results = isam->calculateEstimate();
            mutex_graph.unlock();

            auto t_pgo_end = std::chrono::high_resolution_clock::now();
            double d_time = time_inc(t_pgo_end, t_pgo_begin) / 1000.0;
            if (d_time > 1.0 / kUpdateGraphFreq)
                ROS_WARN_STREAM("[Pgo Update Time "
                                << std::setfill('0') << std::setw(5) << results.size()
                                << "]: " << std::setprecision(3) << d_time << "s");

            // 发布更新后消息odom、path和地图------------------------------------------------------------------------
            pubUpdateResults(results);
        }
    }
}

void preprocessStd() {
    auto t_pre_begin = std::chrono::high_resolution_clock::now();
    std::vector<std::string> input_bag_name_vec, input_pose_name_vec, input_loop_name_vec;
    split(input_bag_names, input_bag_name_vec, "-");
    split(input_pose_names, input_pose_name_vec, "-");
    split(input_loop_names, input_loop_name_vec, "-");
    if (input_bag_name_vec.empty() || input_pose_name_vec.empty())
        return;

    std::vector<sensor_msgs::PointCloud2::ConstPtr> cloud_msg_vec;
    std::vector<int> cloud_signal_vec;

    // 读入pose数据
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_file_vec;
    std::vector<double> times_file_vec;
    for (std::string& name : input_pose_name_vec) {
        std::string pose_path =
            input_path + (input_path.back() == '/' ? "" : "/") + name + "_pose.txt";
        ROS_INFO("Start to load the pose file %s", pose_path.c_str());
        load_pose_with_time(pose_path, poses_file_vec, times_file_vec);
    }
    std::cout << "Sucessfully load pose with number: " << poses_file_vec.size() << std::endl;

    // 读入bag点云存入cloud_msg_vec
    for (int i = 0; i < input_bag_name_vec.size(); i++) {
        std::fstream file_;
        std::string bag_path = input_path + (input_path.back() == '/' ? "" : "/") +
                               input_bag_name_vec[i] + "_cloud.bag";
        file_.open(bag_path, std::ios::in);
        if (!file_) {
            std::cout << "File " << bag_path << " does not exit" << std::endl;
        }
        ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
        rosbag::Bag bag;
        try {
            bag.open(bag_path, rosbag::bagmode::Read);
        } catch (rosbag::BagException e) {
            ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        }
        if (1) {
            std::vector<std::string> topics;
            topics.push_back(file_cloud_topic.empty() ? std::string("/cloud_registered_body")
                                                      : file_cloud_topic);
            rosbag::View view(bag, rosbag::TopicQuery(topics));
            BOOST_FOREACH (rosbag::MessageInstance const m, view) {
                cloud_msg_vec.push_back(m.instantiate<sensor_msgs::PointCloud2>());
                cloud_signal_vec.push_back(i);  // 记录是第几个包
            }
        }
        if (1) {
            std::vector<std::string> topics;
            topics.push_back(std::string("/rtk_odom"));
            rosbag::View view(bag, rosbag::TopicQuery(topics));
            BOOST_FOREACH (rosbag::MessageInstance const m, view) {
                auto msg = m.instantiate<nav_msgs::Odometry>();
                gpsPreprocess(*msg);
            }
        }
    }

    // 配对cloud和pose-------------------------------------------------------------------
    for (int i = 0; i < cloud_msg_vec.size(); i++) {
        auto cloud_ptr = cloud_msg_vec[i];
        if (cloud_ptr != NULL) {
            double laser_time = cloud_ptr->header.stamp.toSec();
            int pose_index = findPoseIndexUsingTime(times_file_vec, laser_time);
            if (pose_index == -1)
                continue;

            std::shared_ptr<PclOdomPair> popair_ptr(new PclOdomPair());
            popair_ptr->addPclMsg(cloud_ptr);
            popair_ptr->setOdom(poses_file_vec[pose_index].first, poses_file_vec[pose_index].second,
                                std::to_string(cloud_signal_vec[i]));

            if (popair_ptr->is_completed()) {
                mutex_msg.lock();
                popair_que.push(popair_ptr);
                mutex_msg.unlock();
            }
        }
    }
    int pre_size = popair_que.size(), per_cnt{0};
    std::cout << "[Preprocess] Read frame num: " << pre_size << std::endl;

    // 加入bag点云的std描述子-------------------------------------------------------------------
    while (!popair_que.empty()) {
        per_cnt++;
        auto popair = popair_que.front();
        double curr_stamp = popair->timestamp().toSec();

        // 发布path
        Eigen::Vector3d translation = popair->T;
        Eigen::Matrix3d rotation = Eigen::Quaterniond(popair->R).normalized().toRotationMatrix();
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = "camera_init";
        setPoseStamp(pose_stamp, translation, Eigen::Quaterniond(rotation));
        lio_path.poses.push_back(pose_stamp);
        if (per_cnt % 10 == 0)
            pub_lio_path.publish(lio_path);

        // 预处理
        bool is_key_frame = msgpairPreprocess(popair);
        mutex_msg.lock();
        popair_que.pop();
        mutex_msg.unlock();

        if (!is_key_frame)
            continue;

        if (cloudInd == 0)
            addInitFactor();
        else {
            addNearFactor(is_new_launch);

            if (cloudInd % config_setting.sub_frame_num_ == 0) {
                // addLoopFactor();
                key_poses_vec.push_back(poses_vec[cloudInd - config_setting.sub_frame_num_]);
                std::vector<STDesc> stds_vec;
                std_manager->GenerateSTDescs(temp_cloud, stds_vec);

                std_manager->AddSTDescs(stds_vec);

                pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
                save_key_cloud = *temp_cloud;
                std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

                temp_cloud->clear();
                keyCloudInd++;
            }
        }
        if (kUseGps)
            addGpsFactor(curr_stamp);

        cloudInd++;

        std::cout << "\r";  // 回到行首位置
        std::cout << "[Preprocess] Read Bag: " << std::setw(6) << std::fixed << std::setprecision(2)
                  << 100.0 * per_cnt / pre_size << "%  ";
    }
    std::cout << "\n";

    // 读入并处理loop数据-------------------------------------------------------------------
    if (!input_loop_name_vec.empty()) {
        std::vector<std::pair<double, double>> loop_time_vec;
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> loop_odom_vec;
        for (std::string& name : input_loop_name_vec) {
            std::string loop_path =
                input_path + (input_path.back() == '/' ? "" : "/") + name + "_loop.txt";
            ROS_INFO("Start to load the loop file %s", loop_path.c_str());
            load_loop_with_time(loop_path, loop_odom_vec, loop_time_vec);
            std::cout << "Sucessfully load loop with number: " << loop_odom_vec.size() << std::endl;
        }

        if (loop_odom_vec.size() != loop_time_vec.size())
            ROS_WARN("Read Loop File Warn!");
        else {
            for (int i = 0; i < loop_odom_vec.size(); i++) {
                int tar_frame = findPoseIndexUsingTime(times_vec, loop_time_vec[i].first);
                int src_frame = findPoseIndexUsingTime(times_vec, loop_time_vec[i].second);

                if (tar_frame == -1 || src_frame == -1)
                    continue;

                gtsam::Point3 ttem(loop_odom_vec[i].first);
                gtsam::Rot3 Rtem(loop_odom_vec[i].second);
                gtsam::NonlinearFactor::shared_ptr loop_factor(
                    new gtsam::BetweenFactor<gtsam::Pose3>(
                        tar_frame, src_frame, gtsam::Pose3(Rtem, ttem), robustLoopNoise));
                graph.push_back(loop_factor);

                if (1) {
                    Eigen::Vector3d tmp_T = loop_odom_vec[i].first;
                    Eigen::Quaterniond tmp_Q = Eigen::Quaterniond(loop_odom_vec[i].second);
                    save_loop_file << std::fixed << std::setprecision(20) << times_vec[tar_frame]
                                   << " " << times_vec[src_frame] << " " << tmp_T.x() << " "
                                   << tmp_T.y() << " " << tmp_T.z() << " " << tmp_Q.x() << " "
                                   << tmp_Q.y() << " " << tmp_Q.z() << " " << tmp_Q.w()
                                   << std::endl;
                }
            }
        }
    }

    // 计算graph-------------------------------------------------------------------
    if (cloudInd > 5) {
        mutex_graph.lock();
        isam->update(graph, initial);
        for (int i = 0; i < 10; i++)
            isam->update();
        // 清空因子图
        graph.resize(0);
        initial.clear();
        gtsam::Values results = isam->calculateEstimate();
        mutex_graph.unlock();

        // 发布更新后消息path和地图------------------------------------------------------------------------
        pubUpdateResults(results);
    }
    auto t_pre_end = std::chrono::high_resolution_clock::now();

    double d_time = time_inc(t_pre_end, t_pre_begin) / 1000.0;
    ROS_INFO_STREAM("[Preprocess] Use Time: " << std::setprecision(3) << d_time << "s");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "std_loop");
    ros::NodeHandle nh;

    nh.param<bool>("run_pgo", kRunPgo, false);
    nh.param<bool>("use_loop", kUseLoop, true);
    nh.param<bool>("use_gps", kUseGps, false);
    nh.param<bool>("use_file", kUseFile, false);

    std::string cloud_topic, odom_topic, gps_topic;
    std::string save_path, save_name, save_map_path, save_enu_map_path;
    nh.param<std::string>("cloud_topic", cloud_topic, "/cloud_registered_body");
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry_enu");
    nh.param<std::string>("gps_topic", gps_topic, "/rtk_odom_enu");
    nh.param<std::string>("save_path", save_path, std::string(ROOT_DIR) + "MAP");
    nh.param<std::string>("save_name", save_name, "pgo");
    save_pose_path = save_path + (save_path.back() == '/' ? "" : "/") + save_name + "_pose.txt";
    save_loop_path = save_path + (save_path.back() == '/' ? "" : "/") + save_name + "_loop.txt";
    save_map_path = save_path + (save_path.back() == '/' ? "" : "/") + save_name + "_map.pcd";
    save_enu_map_path =
        save_path + (save_path.back() == '/' ? "" : "/") + save_name + "_map_enu.pcd";
    save_loop_file.open(save_loop_path);
    //    std::cout << "Loop file save path: " << save_path << std::endl;

    nh.param<std::string>("input_path", input_path, "");
    nh.param<std::string>("input_bag_names", input_bag_names, "");
    nh.param<std::string>("input_pose_names", input_pose_names, "");
    nh.param<std::string>("input_loop_names", input_loop_names, "");
    file_cloud_topic = cloud_topic;
    //    std::cout << "Loop file input path: " << input_path << std::endl;

    nh.param<double>("loop/update_graph_freq", kUpdateGraphFreq, 1.0);
    nh.param<bool>("loop/use_key_frame", use_key_frame, true);
    nh.param<double>("loop/key_frame_len_thre", kKeyFrameLenThre, 0.1);
    nh.param<double>("loop/key_frame_ang_thre", kKeyFrameAngThre, 0.1);
    nh.param<int>("loop/max_loop_factor_count", max_loop_factor_count, 0);
    nh.param<double>("loop/gps_waiting_time", kGpsWaitingTime, 10.0);
    nh.param<double>("loop/gps_spacing_distance", kGpsSpacingDistance, 5.0);
    nh.param<double>("loop/gps_spacing_time", kGpsSpacingTime, 2.0);
    nh.param<double>("map_resolution", kMapResolution, 0.1);

    ros::Subscriber subCloud = nh.subscribe(cloud_topic, 100000, pclCbk);
    ros::Subscriber subOdom = nh.subscribe(odom_topic, 100000, odomCbk);
    ros::Subscriber subGps = nh.subscribe(gps_topic, 10000000, gpsCbk);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    pubCureentCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
    pubCurrentCorner = nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
    pubMatchedCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
    pubMatchedCorner = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
    pubSTD = nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
    pubCorrectCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_correct", 10000);
    pubOdomCorreted = nh.advertise<nav_msgs::Odometry>("/odom_corrected", 10);
    pubPgoMap = nh.advertise<sensor_msgs::PointCloud2>("/pgo_map", 1);
    pubCurrCorrectCloud = nh.advertise<sensor_msgs::PointCloud2>("/curr_cloud_correct", 1);
    pub_lio_path = nh.advertise<nav_msgs::Path>("/lio_path", 100000);
    pub_pgo_path = nh.advertise<nav_msgs::Path>("/pgo_path", 100000);
    pub_gps_path = nh.advertise<nav_msgs::Path>("/gps_path", 100000);
    pub_gps_points = nh.advertise<sensor_msgs::PointCloud2>("/gps_points", 10000);
    pub_gps_use_points = nh.advertise<sensor_msgs::PointCloud2>("/gps_use_points", 10000);
    lio_path.header.frame_id = "camera_init";
    pgo_path.header.frame_id = "camera_init";
    gps_path.header.frame_id = "camera_init";
    odom.header.frame_id = "camera_init";

    read_parameters(nh, config_setting);
    std_manager = new STDescManager(config_setting);

    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    gtsam::Vector robustNoiseVector6(6);  // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    gtsam::Vector newLaunchNoiseVector6(6);
    newLaunchNoiseVector6 << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    newLaunchNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(newLaunchNoiseVector6));

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    if (kUseFile)
        preprocessStd();

    std::cout << "[PGO]: Init Success!" << std::endl;
    if (!kRunPgo) {  // 实时loop
        std::thread thd_0{saveGraphFactor};
        std::thread thd_1{updateIsamGraph};
        ros::spin();
    } else {  // 读取文件
        FilePreprocess fpp;
        std::vector<nav_msgs::Odometry> gps_que_;
        fpp.readFile(popair_que, gps_que_);
        for (auto& gps_ : gps_que_)
            gpsPreprocess(gps_);
        int frame_num = popair_que.size();
        if (frame_num < 5) {
            ROS_WARN("Read Data Wrong: Too Little Data!");
        } else {
            std::cout << "[PGO]: Data Read Completed! Frame Num: " << frame_num << std::endl;

            // 加入因子
            auto t_graph_begin = std::chrono::high_resolution_clock::now();
            int per_cnt{0};
            while (!popair_que.empty()) {
                auto popair = popair_que.front();
                popair_que.pop();
                double curr_stamp = popair->timestamp().toSec();
                bool is_key_frame = msgpairPreprocess(popair);
                pubSourceMsg(*popair);

                if (is_key_frame)
                    saveGraphFactorMain(curr_stamp);

                std::cout << "\r";  // 回到行首位置
                std::cout << "[Run Bag]: " << std::setw(6) << std::fixed << std::setprecision(2)
                          << 100.0 * ++per_cnt / frame_num << "%";
            }
            auto t_graph_end = std::chrono::high_resolution_clock::now();
            std::cout << "\nConstruct Graph Time: " << time_inc(t_graph_end, t_graph_begin) / 1000.0
                      << "s" << std::endl;
            std::cout << "Graph Factor Count: " << graph.size()
                      << ", Initial Value Count: " << initial.size()
                      << ", Loop Factor Count: " << loop_factor_count
                      << ", Rejected Loop Factor Count: " << rejected_loop_factor_count
                      << std::endl;

            // 求解graph
            auto t_pgo_begin = std::chrono::high_resolution_clock::now();
            isam->update(graph, initial);
            for (int i = 0; i < 100; i++)
                isam->update();
            graph.resize(0);
            initial.clear();
            gtsam::Values results = isam->calculateEstimate();
            auto t_pgo_end = std::chrono::high_resolution_clock::now();
            std::cout << "Result Factor Size: " << results.size() << std::endl;
            std::cout << "Solve Graph Time: " << time_inc(t_pgo_end, t_pgo_begin) / 1000.0 << "s"
                      << std::endl;

            // 发布结果
            auto t_pub_begin = std::chrono::high_resolution_clock::now();
            pcl::PointCloud<pcl::PointXYZI> empty_cloud;
            sensor_msgs::PointCloud2 pub_cloud;
            pcl::toROSMsg(empty_cloud, pub_cloud);
            pub_cloud.header.frame_id = "camera_init";
            pubCureentCloud.publish(pub_cloud);
            pubCurrentCorner.publish(pub_cloud);
            pubMatchedCloud.publish(pub_cloud);
            pubMatchedCorner.publish(pub_cloud);
            std::vector<std::pair<STDesc, STDesc>> empty_std_pair;
            publish_std_pairs(empty_std_pair, pubSTD);

            pubUpdateResults(results, kMapResolution);
            std::cout << "\n";
            if (pgo_map_pcl->size() > 5) {
                pcl::io::savePCDFileBinary(save_map_path, *pgo_map_pcl);
                std::cout << "Save Map Successfully! Map Size: " << pgo_map_pcl->size()
                          << std::endl;
            } else
                std::cout << "No Point To Build Map.";
            auto t_pub_end = std::chrono::high_resolution_clock::now();
            std::cout << "Save Results Time: " << time_inc(t_pub_end, t_pub_begin) / 1000.0 << "s"
                      << std::endl;
        }
    }

    save_loop_file.close();
    return 0;
}
