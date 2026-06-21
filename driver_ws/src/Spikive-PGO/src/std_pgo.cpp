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
#include "include/STDesc.h"
#include "include/lsdc_tools.hpp"

bool judge_gps_point(const nav_msgs::Odometry::ConstPtr& msg_in) {
    static Eigen::Vector3d last_gps_pose_tmp;
    static double last_gps_stamp_tmp{-1};
    Eigen::Vector3d gps_pose_tmp(msg_in->pose.pose.position.x, msg_in->pose.pose.position.y,
                                 msg_in->pose.pose.position.z);
    double gps_stamp_tmp = msg_in->header.stamp.toSec();

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

pcl::PointCloud<pcl::PointXYZI>::Ptr base_map_pcl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr pgo_map_pcl(new pcl::PointCloud<pcl::PointXYZI>());
std::vector<sensor_msgs::PointCloud2::ConstPtr> cloud_msg_vec;

int main(int argc, char** argv) {
    ros::init(argc, argv, "std_pgo");
    ros::NodeHandle nh;

    bool kUseGps;
    nh.param<bool>("use_gps", kUseGps, false);
    std::cout << "use_gps: " << kUseGps << "\n";

    std::string input_bag_names = "", input_pose_names = "", input_path = "", base_map_path = "";
    nh.param<std::string>("input_bag_names", input_bag_names, "");
    nh.param<std::string>("input_pose_names", input_pose_names, "");
    nh.param<std::string>("input_path", input_path, "");
    std::vector<std::string> input_bag_name_vec, input_pose_name_vec;
    split(input_bag_names, input_bag_name_vec, "-");
    split(input_pose_names, input_pose_name_vec, "-");
    std::ofstream save_pose_file, save_loop_file;
    save_pose_file.open(std::string(ROOT_DIR) + "MAP/pgo_pose.txt");
    save_loop_file.open(std::string(ROOT_DIR) + "MAP/pgo_loop.txt");

    bool use_key_frame{false};
    double kKeyFrameLenThre{0.1}, kKeyFrameAngThre{0.1};
    double kGpsWaitingTime{10.0}, kGpsSpacingDistance{5.0}, kGpsSpacingTime{2.0};
    nh.param<bool>("loop/use_key_frame", use_key_frame, false);
    nh.param<double>("loop/key_frame_len_thre", kKeyFrameLenThre, 0.1);
    nh.param<double>("loop/key_frame_ang_thre", kKeyFrameAngThre, 0.1);
    nh.param<double>("loop/key_frame_ang_thre", kKeyFrameAngThre, 0.1);
    nh.param<double>("loop/gps_waiting_time", kGpsWaitingTime, 10.0);
    nh.param<double>("loop/gps_spacing_distance", kGpsSpacingDistance, 5.0);
    nh.param<double>("loop/gps_spacing_time", kGpsSpacingTime, 2.0);

    ConfigSetting config_setting;
    read_parameters(nh, config_setting);

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    ros::Publisher pubCureentCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
    ros::Publisher pubCurrentCorner =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
    ros::Publisher pubMatchedCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
    ros::Publisher pubMatchedCorner =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
    ros::Publisher pubSTD = nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

    ros::Publisher pubCorrectCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_correct", 10000);
    ros::Publisher pubOdomCorreted = nh.advertise<nav_msgs::Odometry>("/odom_corrected", 10);
    ros::Publisher pubPgoMap = nh.advertise<sensor_msgs::PointCloud2>("/pgo_map", 10000);
    ros::Publisher pub_base_map = nh.advertise<sensor_msgs::PointCloud2>("/base_map", 10000);
    ros::Publisher pub_lio_path = nh.advertise<nav_msgs::Path>("/lio_path", 100000);
    ros::Publisher pub_pgo_path = nh.advertise<nav_msgs::Path>("/pgo_path", 100000);
    ros::Publisher pub_gps_path = nh.advertise<nav_msgs::Path>("/gps_path", 100000);
    ros::Publisher pub_gps_points = nh.advertise<sensor_msgs::PointCloud2>("/gps_points", 10000);
    ros::Publisher pub_gps_use_points =
        nh.advertise<sensor_msgs::PointCloud2>("/gps_use_points", 10000);
    nav_msgs::Path lio_path, pgo_path, gps_path;
    lio_path.header.frame_id = "camera_init";
    pgo_path.header.frame_id = "camera_init";
    gps_path.header.frame_id = "camera_init";

    ros::Rate loop(500);
    ros::Rate slow_loop(100);
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_file_vec, poses_vec;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> key_poses_vec;
    std::vector<double> times_file_vec, times_vec;
    std::vector<int> cloud_signal_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vec;

    STDescManager* std_manager = new STDescManager(config_setting);
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
        gtsam::noiseModel::Diagonal::Variances(Vector6);
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise, newLaunchNoise;
    double loopNoiseScore = 0.1;
    gtsam::Vector robustNoiseVector6(6);  // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore,
        loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    gtsam::Vector newLaunchNoiseVector6(6);
    newLaunchNoiseVector6 << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    newLaunchNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(newLaunchNoiseVector6));

    size_t cloudInd = 0;
    size_t keyCloudInd = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<double> descriptor_time;
    std::vector<double> querying_time;
    std::vector<double> update_time;
    int triggle_loop_num = 0;

    // 读入pose数据
    for (std::string& name : input_pose_name_vec) {
        std::string pose_path =
            input_path + (input_path.back() == '/' ? "" : "/") + name + "_pose.txt";
        load_pose_with_time(pose_path, poses_file_vec, times_file_vec);
    }
    std::cout << "Sucessfully load pose with number: " << poses_file_vec.size() << std::endl;

    // gps数据准备
    std::queue<nav_msgs::Odometry::ConstPtr> gps_msg_que;
    pcl::PointCloud<pcl::PointXYZI>::Ptr gps_points(new pcl::PointCloud<pcl::PointXYZI>());
    // 在状态ok之后一定时间后才认为是真的ok
    bool curr_gps_status{true}, converting_to_ok{false};
    double converting_to_ok_stamp{-1};

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
            std::vector<std::string> types, topics;
            // types.push_back(std::string("sensor_msgs/PointCloud2"));
            // rosbag::View view(bag, rosbag::TypeQuery(types));
            topics.push_back(std::string("/cloud_registered_body"));
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            BOOST_FOREACH (rosbag::MessageInstance const m, view) {
                cloud_msg_vec.push_back(m.instantiate<sensor_msgs::PointCloud2>());
                cloud_signal_vec.push_back(i);  // 记录是第几个包
            }
        }
        if (kUseGps) {
            std::vector<std::string> topics;
            topics.push_back(std::string("/rtk_odom"));
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            BOOST_FOREACH (rosbag::MessageInstance const m, view) {
                auto msg_in = m.instantiate<nav_msgs::Odometry>();
                if (!judge_gps_point(msg_in))
                    continue;

                if (msg_in->child_frame_id == "OK") {
                    pcl::PointXYZI pt;
                    pt.x = msg_in->pose.pose.position.x;
                    pt.y = msg_in->pose.pose.position.y;
                    pt.z = msg_in->pose.pose.position.z;
                    pt.intensity = 255;

                    if (curr_gps_status) {
                        gps_msg_que.push(msg_in);

                    } else {
                        if (!converting_to_ok) {
                            converting_to_ok = true;
                            converting_to_ok_stamp = msg_in->header.stamp.toSec();
                        } else {
                            if (msg_in->header.stamp.toSec() - converting_to_ok_stamp >
                                kGpsWaitingTime) {
                                curr_gps_status = true;
                                converting_to_ok = false;
                            }
                        }
                        pt.intensity = 0;
                    }

                    if (gps_points->empty() || points_dist(pt, gps_points->back()) > 1.0) {
                        gps_points->push_back(pt);
                    }
                } else {
                    curr_gps_status = false;
                    converting_to_ok = false;
                }

                geometry_msgs::PoseStamped pose_stamp;
                pose_stamp.header.frame_id = "camera_init";
                pose_stamp.pose.position = msg_in->pose.pose.position;
                pose_stamp.pose.orientation = msg_in->pose.pose.orientation;
                if (gps_path.poses.empty() ||
                    points_dist(pose_stamp.pose.position, gps_path.poses.back().pose.position) >
                        1.0) {
                    gps_path.poses.push_back(pose_stamp);
                }
            }
        }
    }

    // 将每帧pose及loop信息加入因子图
    auto t_graph_begin = std::chrono::high_resolution_clock::now();
    Eigen::Vector3d last_translation;
    Eigen::Matrix3d last_rotation;
    // gps筛选
    pcl::PointCloud<pcl::PointXYZI>::Ptr gps_use_points(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Vector3d last_gps_pose;
    double cumulative_distance{-1.0}, last_gps_stamp{-1.0};
    if (1) {
        int per_cnt{0};
        std::string last_signal{""};
        bool is_new_launch{false};
        for (int i = 0; i < cloud_msg_vec.size(); i++) {
            ++per_cnt;
            auto cloud_ptr = cloud_msg_vec[i];
            if (cloud_ptr != NULL) {
                double laser_time = cloud_ptr->header.stamp.toSec();
                pcl::PCLPointCloud2 pcl_pc;
                pcl_conversions::toPCL(*cloud_ptr, pcl_pc);
                pcl::PointCloud<pcl::PointXYZI> cloud;
                pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(
                    new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromPCLPointCloud2(pcl_pc, cloud);
                int pose_index = findPoseIndexUsingTime(times_file_vec, laser_time);
                if (pose_index == -1)
                    continue;

                // 若是新的线路slam，标记
                Eigen::Vector3d translation = poses_file_vec[pose_index].first;
                Eigen::Matrix3d rotation = poses_file_vec[pose_index].second;
                std::string curr_signal = std::to_string(cloud_signal_vec[i]);
                if (last_signal != curr_signal) {
                    is_new_launch = true;
                    last_signal = curr_signal;
                    // std::cout << "[New Launch]: " << curr_signal << std::endl;
                } else
                    is_new_launch = false;

                if (use_key_frame) {
                    bool is_key_frame =
                        (is_new_launch || poseDiffThre(translation - last_translation,
                                                       rotation * last_rotation.transpose(),
                                                       kKeyFrameLenThre, kKeyFrameAngThre));
                    if (!is_key_frame)
                        continue;
                    last_translation = translation;
                    last_rotation = rotation;
                }
                poses_vec.push_back(std::make_pair(translation, rotation));
                times_vec.push_back(laser_time);

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

                // check if keyframe
                if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
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

                    // step2. Searching Loop
                    auto t_query_begin = std::chrono::high_resolution_clock::now();
                    std::pair<int, double> search_result(-1, 0);
                    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
                    loop_transform.first << 0, 0, 0;
                    loop_transform.second = Eigen::Matrix3d::Identity();
                    std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
                    if (keyCloudInd > config_setting.skip_near_num_) {
                        std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                                loop_std_pair);
                    }
                    if (search_result.first > 0) {
                        // std::cout << "[Loop Detection] triggle loop: " << keyCloudInd << "--"
                        //           << search_result.first << ", score:" <<
                        //           search_result.second
                        //           << std::endl;
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
                    down_sampling_voxel(save_key_cloud, 0.1);
                    std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

                    // publish
                    sensor_msgs::PointCloud2 pub_cloud;
                    pcl::toROSMsg(*temp_cloud, pub_cloud);
                    pub_cloud.header.frame_id = "camera_init";
                    pubCureentCloud.publish(pub_cloud);
                    pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
                    pub_cloud.header.frame_id = "camera_init";
                    pubCurrentCorner.publish(pub_cloud);

                    // std::cout << "std size: " << stds_vec.size() << "\n";
                    // while (1) {
                    //     ros::Duration(1).sleep();
                    // }

                    if (search_result.first > 0) {
                        triggle_loop_num++;
                        // add connection between near frame
                        initial.insert(cloudInd,
                                       gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                                    gtsam::Point3(poses_vec[cloudInd].first)));
                        // source i
                        // target i-1
                        Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
                        Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

                        t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
                        R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

                        gtsam::Rot3 R_sam(R_ab);
                        gtsam::Point3 t_sam(t_ab);

                        if (is_new_launch) {
                            gtsam::NonlinearFactor::shared_ptr near_factor(
                                new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                                       gtsam::Pose3(R_sam, t_sam),
                                                                       newLaunchNoise));
                            graph.push_back(near_factor);
                        } else {
                            gtsam::NonlinearFactor::shared_ptr near_factor(
                                new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                                       gtsam::Pose3(R_sam, t_sam),
                                                                       odometryNoise));
                            graph.push_back(near_factor);
                        }

                        int match_frame = search_result.first;
                        // obtain optimal transform
                        std_manager->PlaneGeomrtricIcp(std_manager->plane_cloud_vec_.back(),
                                                       std_manager->plane_cloud_vec_[match_frame],
                                                       loop_transform);

                        // add connection between loop frame.
                        // e.g. if src_key_frame_id 5 with sub frames 51~60 triggle loop with
                        //     tar_key_frame_id 1 with sub frames 11~20, add connection between
                        // each sub frame, 51-11, 52-12,...,60-20.

                        int sub_frame_num = config_setting.sub_frame_num_;
                        for (size_t j = 1; j <= sub_frame_num; j++) {
                            int src_frame = cloudInd + j - sub_frame_num;
                            Eigen::Matrix3d src_R =
                                loop_transform.second * poses_vec[src_frame].second;
                            Eigen::Vector3d src_t =
                                loop_transform.second * poses_vec[src_frame].first +
                                loop_transform.first;
                            int tar_frame = match_frame * sub_frame_num + j;
                            Eigen::Matrix3d tar_R = poses_vec[tar_frame].second;
                            Eigen::Vector3d tar_t = poses_vec[tar_frame].first;

                            gtsam::Point3 ttem(tar_R.transpose() * (src_t - tar_t));
                            gtsam::Rot3 Rtem(tar_R.transpose() * src_R);
                            gtsam::NonlinearFactor::shared_ptr loop_factor(
                                new gtsam::BetweenFactor<gtsam::Pose3>(tar_frame, src_frame,
                                                                       gtsam::Pose3(Rtem, ttem),
                                                                       robustLoopNoise));
                            graph.push_back(loop_factor);

                            if (1) {
                                Eigen::Vector3d tmp_T = tar_R.transpose() * (src_t - tar_t);
                                Eigen::Quaterniond tmp_Q =
                                    Eigen::Quaterniond(tar_R.transpose() * src_R);
                                save_loop_file
                                    << std::fixed << std::setprecision(20) << times_vec[tar_frame]
                                    << " " << times_vec[src_frame] << " " << tmp_T.x() << " "
                                    << tmp_T.y() << " " << tmp_T.z() << " " << tmp_Q.x() << " "
                                    << tmp_Q.y() << " " << tmp_Q.z() << " " << tmp_Q.w()
                                    << std::endl;
                            }
                        }

                        pcl::PointCloud<pcl::PointXYZI> correct_cloud;
                        pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first], pub_cloud);
                        pub_cloud.header.frame_id = "camera_init";
                        pubMatchedCloud.publish(pub_cloud);

                        pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                                      pub_cloud);
                        pub_cloud.header.frame_id = "camera_init";
                        pubMatchedCorner.publish(pub_cloud);
                        publish_std_pairs(loop_std_pair, pubSTD);

                    } else {
                        // add connection between near frame
                        initial.insert(cloudInd,
                                       gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                                    gtsam::Point3(poses_vec[cloudInd].first)));
                        Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
                        Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

                        t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
                        R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

                        gtsam::Rot3 R_sam(R_ab);
                        gtsam::Point3 t_sam(t_ab);

                        if (is_new_launch) {
                            gtsam::NonlinearFactor::shared_ptr near_factor(
                                new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                                       gtsam::Pose3(R_sam, t_sam),
                                                                       newLaunchNoise));
                            graph.push_back(near_factor);
                        } else {
                            gtsam::NonlinearFactor::shared_ptr near_factor(
                                new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                                       gtsam::Pose3(R_sam, t_sam),
                                                                       odometryNoise));
                            graph.push_back(near_factor);
                        }
                    }
                    temp_cloud->clear();
                    keyCloudInd++;
                    // loop.sleep();
                } else {
                    if (cloudInd == 0) {
                        initial.insert(0, gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                                       gtsam::Point3(poses_vec[cloudInd].first)));
                        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                            0,
                            gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                         gtsam::Point3(poses_vec[cloudInd].first)),
                            odometryNoise));
                    } else {
                        // add connection between near frame
                        initial.insert(cloudInd,
                                       gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),
                                                    gtsam::Point3(poses_vec[cloudInd].first)));
                        Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
                        Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

                        t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
                        R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

                        gtsam::Rot3 R_sam(R_ab);
                        gtsam::Point3 t_sam(t_ab);

                        if (is_new_launch) {
                            gtsam::NonlinearFactor::shared_ptr near_factor(
                                new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                                       gtsam::Pose3(R_sam, t_sam),
                                                                       newLaunchNoise));
                            graph.push_back(near_factor);
                        } else {
                            gtsam::NonlinearFactor::shared_ptr near_factor(
                                new gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd,
                                                                       gtsam::Pose3(R_sam, t_sam),
                                                                       odometryNoise));
                            graph.push_back(near_factor);
                        }
                    }
                }

                nav_msgs::Odometry odom;
                odom.header.frame_id = "camera_init";
                setPoseStamp(odom.pose, translation, Eigen::Quaterniond(rotation));
                pubOdomAftMapped.publish(odom);

                geometry_msgs::PoseStamped pose_stamp;
                pose_stamp.header.frame_id = "camera_init";
                setPoseStamp(pose_stamp, translation, Eigen::Quaterniond(rotation));
                if (cloudInd % 10 == 0) {
                    lio_path.poses.push_back(pose_stamp);
                    pub_lio_path.publish(lio_path);
                }

                // 加入gps因子
                if (kUseGps) {
                    double curr_stamp = laser_time;
                    while (!gps_msg_que.empty()) {
                        if (gps_msg_que.front()->header.stamp.toSec() < curr_stamp - 0.05)
                            gps_msg_que.pop();
                        else if (gps_msg_que.front()->header.stamp.toSec() > curr_stamp + 0.05)
                            break;
                        else {
                            nav_msgs::OdometryConstPtr gps_msg = gps_msg_que.front();
                            double d_time{1.0};
                            while (!gps_msg_que.empty()) {
                                if (std::abs(gps_msg_que.front()->header.stamp.toSec() -
                                             curr_stamp) < d_time) {
                                    d_time = std::abs(gps_msg_que.front()->header.stamp.toSec() -
                                                      curr_stamp);
                                    gps_msg = gps_msg_que.front();
                                    gps_msg_que.pop();
                                } else
                                    break;
                            }
                            Eigen::Vector3d gps_pose(gps_msg->pose.pose.position.x,
                                                     gps_msg->pose.pose.position.y,
                                                     gps_msg->pose.pose.position.z);

                            // gps每累计走5m或者时间间隔大于2s，加入一次
                            cumulative_distance += (gps_pose - last_gps_pose).norm();
                            last_gps_pose = gps_pose;
                            if (last_gps_stamp > 0 &&
                                (cumulative_distance < kGpsSpacingDistance &&
                                 gps_msg->header.stamp.toSec() - last_gps_stamp < kGpsSpacingTime))
                                break;
                            cumulative_distance = 0;
                            last_gps_stamp = gps_msg->header.stamp.toSec();

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
                            gtsam::Vector Vector3(3);
                            Vector3 << 1.0, 1.0, 1.0;
                            gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
                                gtsam::noiseModel::Diagonal::Variances(Vector3);
                            gtsam::GPSFactor::shared_ptr gps_factor(new gtsam::GPSFactor(
                                cloudInd, gtsam::Point3(gps_pose.x(), gps_pose.y(), gps_pose.z()),
                                gps_noise));
                            graph.push_back(gps_factor);
                            break;
                        }
                    }
                }

                // loop.sleep();
                cloudInd++;
            }
            std::cout << "\r";  // 回到行首位置
            std::cout << "[Run Bag]: " << std::setw(6) << std::fixed << std::setprecision(2)
                      << 100.0 * per_cnt / cloud_msg_vec.size() << "%";
        }
    }
    auto t_graph_end = std::chrono::high_resolution_clock::now();
    std::cout << std::endl;

    double mean_descriptor_time =
        std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) * 1.0 /
        descriptor_time.size();
    double mean_query_time =
        std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 / querying_time.size();
    double mean_update_time =
        std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 / update_time.size();
    std::cout << "Total key frame number:" << keyCloudInd << ", loop number:" << triggle_loop_num
              << std::endl;
    std::cout << "Construct Graph Time:" << time_inc(t_graph_end, t_graph_begin) / 1000.0 << "s"
              << std::endl;
    // std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
    //           << "ms, query: " << mean_query_time << "ms, update: " << mean_update_time
    //           << "ms, total: " << mean_descriptor_time + mean_query_time + mean_update_time <<
    //           "ms"
    //           << std::endl;

    auto t_pgo_begin = std::chrono::high_resolution_clock::now();
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);
    for (int i = 0; i < 100; i++)
        isam.update();
    graph.resize(0);
    initial.clear();
    auto t_pgo_end = std::chrono::high_resolution_clock::now();
    std::cout << "Solve Pgo Time:" << time_inc(t_pgo_end, t_pgo_begin) / 1000.0 << "s\n"
              << std::endl;

    // clear rviz
    pcl::PointCloud<pcl::PointXYZI> empty_cloud;
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(empty_cloud, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubCureentCloud.publish(pub_cloud);
    // loop.sleep();
    pubCurrentCorner.publish(pub_cloud);
    // loop.sleep();
    pubMatchedCloud.publish(pub_cloud);
    // loop.sleep();
    pubMatchedCorner.publish(pub_cloud);
    // loop.sleep();
    std::vector<std::pair<STDesc, STDesc>> empty_std_pair;
    publish_std_pairs(empty_std_pair, pubSTD);
    // slow_loop.sleep();

    gtsam::Values results = isam.calculateEstimate();

    for (size_t i = 0; i < results.size(); i++) {
        gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
        pcl::PointCloud<pcl::PointXYZI> correct_cloud;

        Eigen::Vector3d opt_translation = pose.translation();
        Eigen::Quaterniond opt_q(pose.rotation().matrix());
        for (size_t j = 0; j < cloud_vec[i]->size(); j++) {
            pcl::PointXYZI pi = cloud_vec[i]->points[j];
            Eigen::Vector3d pv(pi.x, pi.y, pi.z);
            // back transform to get point cloud in body frame
            // pv = poses_vec[i].second.transpose() * pv -
            //      poses_vec[i].second.transpose() * poses_vec[i].first;
            // transform with optimal poses
            pv = opt_q * pv + opt_translation;
            pi.x = pv[0];
            pi.y = pv[1];
            pi.z = pv[2];
            correct_cloud.push_back(pi);
        }

        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = "camera_init";
        setPoseStamp(pose_stamp, opt_translation, opt_q);
        if (pgo_path.poses.empty() ||
            points_dist(pose_stamp.pose.position, pgo_path.poses.back().pose.position) > 1.0) {
            pgo_path.poses.push_back(pose_stamp);
            pub_pgo_path.publish(pgo_path);
        }
        // if (i % 5 == 0) {
        //     pgo_path.poses.push_back(pose_stamp);
        //     pub_pgo_path.publish(pgo_path);
        // }

        // slow_loop.sleep();
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(correct_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCorrectCloud.publish(pub_cloud);

        *pgo_map_pcl += correct_cloud;

        std::cout << "\r";  // 回到行首位置
        std::cout << "[Build Map]: " << std::setw(6) << std::fixed << std::setprecision(2)
                  << 100.0 * (i + 1) / results.size() << "%";

        if (1) {
            save_pose_file << std::fixed << std::setprecision(20) << times_vec[i] << " "
                           << opt_translation.x() << " " << opt_translation.y() << " "
                           << opt_translation.z() << " " << opt_q.x() << " " << opt_q.y() << " "
                           << opt_q.z() << " " << opt_q.w() << std::endl;
        }

        // slow_loop.sleep();
    }
    std::cout << std::endl;
    save_loop_file.close();
    save_pose_file.close();
    std::cout << "Result Factor Size: " << results.size() << std::endl;

    pcl::toROSMsg(*gps_points, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pub_gps_points.publish(pub_cloud);
    pub_gps_path.publish(gps_path);

    down_sampling_voxel(*pgo_map_pcl, 0.1);
    pcl::toROSMsg(*pgo_map_pcl, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubPgoMap.publish(pub_cloud);
    if (pgo_map_pcl->size() > 5) {
        pcl::io::savePCDFileBinary(std::string(ROOT_DIR) + "MAP/pgo_map.pcd", *pgo_map_pcl);
        std::cout << "Save Map Successfully! Map Size: " << pgo_map_pcl->size() << std::endl;
    } else
        std::cout << "No Point To Build Map.";

    return 0;
}