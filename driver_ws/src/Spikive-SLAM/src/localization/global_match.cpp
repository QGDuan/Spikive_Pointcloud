#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include <livox_ros_driver/CustomMsg.h>
#include <lsdc_slam/Pose6D.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
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
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "lsdc_math.hpp"
#include "lsdc_tools.hpp"
#include "odom_struct.hpp"

using namespace std;

int kMaxIteration{20};
double kFreq{1.0}, kFilterSizeMap{0.2}, kFilterSizeSrc{0.1};
double kFovFar{300}, kFovAng{360}, kFovRad;
double kFitnessThreshold{0.3}, kMatchTimeoutSec{20.0}, kStatusPublishPeriodSec{1.0};
int kSuccessConfirmCount{10};
bool kDisplayMatchingTime{false}, kDebugDoNotMatch{false}, kContinuousMatch{false},
    kInitialPoseIsWorld{true};
string kDroneId{"2"};
string kInitialPoseTopic, kMapTopic, kScanTopic, kOdomTopic, kDiffOdomTopic,
    kInitMatchSuccessTopic, kMatchStatusTopic;
pcl::PointCloud<pcl::PointXYZ>::Ptr kGlobalMap(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pcl(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in_fov(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in_fov_ds(new pcl::PointCloud<pcl::PointXYZ>());
pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterMap;
pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterIcp;
bool global_map_init{false}, initial_pose_init{false}, matching_init{false};
bool has_published_match_status{false}, last_published_match_success{false};
int init_match_cnt{0};
OdomStructf curr_odom, diff_odom, pending_initial_world_pose;
bool pending_initial_pose{false};
uint64_t match_generation{0};
const size_t kMaxSyncedMsgPairs{2};
const size_t kMaxPendingMsgPairs{10};

ros::Publisher pub_diff_odom, pub_map_in_fov, pub_fov_sphere, pub_init_match_success,
    pub_global_map, pub_match_status;

string match_state{"idle"};
string match_status_message{"Localization idle"};
int match_attempt_count{0};
double latest_fitness_score{-1.0};
ros::Time match_start_time;
ros::Time last_match_status_publish_time;

// ---------------------------------------------------

std::mutex mutex_buf;
std::mutex mutex_map;
std::mutex mutex_status;

struct PclAndOdomMsg {
    sensor_msgs::PointCloud2ConstPtr pcl_msg;
    nav_msgs::Odometry::ConstPtr odom_msg;
    bool has_pcl{false};
    bool has_odom{false};

    void addPclMsg(sensor_msgs::PointCloud2ConstPtr msg_in) {
        pcl_msg = msg_in;
        has_pcl = true;
    }

    void addOdomMsg(nav_msgs::Odometry::ConstPtr msg_in) {
        odom_msg = msg_in;
        has_odom = true;
    }

    bool isCompleted() const { return has_pcl && has_odom; }

    ros::Time timestamp() const {
        if (odom_msg != nullptr) {
            return odom_msg->header.stamp;
        }
        if (pcl_msg != nullptr) {
            return pcl_msg->header.stamp;
        }
        return ros::Time(0);
    }
};

std::map<double, PclAndOdomMsg> time_to_msgpair_map;
std::deque<PclAndOdomMsg> msgpair_queue;

string jsonEscape(const string& value) {
    std::ostringstream out;
    for (const char c : value) {
        switch (c) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\b':
                out << "\\b";
                break;
            case '\f':
                out << "\\f";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(c));
                } else {
                    out << c;
                }
                break;
        }
    }
    return out.str();
}

void publishLocalizationMatchStatus(bool force) {
    std_msgs::String status_msg;
    const ros::Time now = ros::Time::now();
    {
        std::lock_guard<std::mutex> lock_status(mutex_status);
        if (!force && kStatusPublishPeriodSec > 0.0 && !last_match_status_publish_time.isZero() &&
            (now - last_match_status_publish_time).toSec() < kStatusPublishPeriodSec) {
            return;
        }

        int elapsed_ms = 0;
        if (!match_start_time.isZero() &&
            (match_state == "matching" || match_state == "localized" || match_state == "failed")) {
            elapsed_ms = static_cast<int>((now - match_start_time).toSec() * 1000.0);
            if (elapsed_ms < 0) {
                elapsed_ms = 0;
            }
        }
        const bool ok = match_state != "failed";
        std::ostringstream data;
        data << std::fixed << std::setprecision(6)
             << "{\"attempt_count\":" << match_attempt_count
             << ",\"drone_id\":\"" << jsonEscape(kDroneId) << "\""
             << ",\"elapsed_ms\":" << elapsed_ms
             << ",\"fitness_score\":" << latest_fitness_score
             << ",\"message\":\"" << jsonEscape(match_status_message) << "\""
             << ",\"ok\":" << (ok ? "true" : "false")
             << ",\"schema_version\":1"
             << ",\"state\":\"" << jsonEscape(match_state) << "\""
             << ",\"threshold\":" << kFitnessThreshold << "}";
        status_msg.data = data.str();
        last_match_status_publish_time = now;
    }
    pub_match_status.publish(status_msg);
}

void setLocalizationMatchStatus(const string& state, const string& message, bool force_publish) {
    {
        std::lock_guard<std::mutex> lock_status(mutex_status);
        match_state = state;
        match_status_message = message;
    }
    publishLocalizationMatchStatus(force_publish);
}

void beginLocalizationMatching() {
    {
        std::lock_guard<std::mutex> lock_status(mutex_status);
        match_state = "matching";
        match_status_message =
            "Matching current calibrated odometry to the loaded WayPoint map world";
        match_attempt_count = 0;
        latest_fitness_score = -1.0;
        match_start_time = ros::Time::now();
        last_match_status_publish_time = ros::Time(0);
    }
    publishLocalizationMatchStatus(true);
}

void updateLocalizationMatchMetrics(double fitness_score) {
    std::lock_guard<std::mutex> lock_status(mutex_status);
    ++match_attempt_count;
    latest_fitness_score = fitness_score;
}

void clearPendingSuccessConfirm(uint64_t generation_snapshot) {
    std::unique_lock<std::mutex> lock(mutex_buf);
    if (generation_snapshot == match_generation && !matching_init) {
        init_match_cnt = 0;
    }
}

bool localizationMatchTimedOut() {
    std::lock_guard<std::mutex> lock_status(mutex_status);
    return match_state == "matching" && kMatchTimeoutSec > 0.0 && !match_start_time.isZero() &&
           (ros::Time::now() - match_start_time).toSec() >= kMatchTimeoutSec;
}

bool localizationMatchInProgress() {
    std::lock_guard<std::mutex> lock_status(mutex_status);
    return match_state == "matching";
}

void publishInitMatchSuccess(bool success) {
    std::lock_guard<std::mutex> lock_status(mutex_status);
    if (has_published_match_status && last_published_match_success == success) {
        return;
    }
    std_msgs::Bool init_success_msg;
    init_success_msg.data = success;
    pub_init_match_success.publish(init_success_msg);
    has_published_match_status = true;
    last_published_match_success = success;
}

void resetMatchingState(bool clear_initial_pose) {
    matching_init = false;
    init_match_cnt = 0;
    msgpair_queue.clear();
    time_to_msgpair_map.clear();
    ++match_generation;
    if (clear_initial_pose) {
        initial_pose_init = false;
        pending_initial_pose = false;
    }
    publishInitMatchSuccess(false);
}

void prunePendingMsgPairs() {
    while (time_to_msgpair_map.size() > kMaxPendingMsgPairs) {
        time_to_msgpair_map.erase(time_to_msgpair_map.begin());
    }
}

void pclCbk(const sensor_msgs::PointCloud2ConstPtr& msg_in) {
    std::unique_lock<std::mutex> lock(mutex_buf);
    PclAndOdomMsg& msgpair = time_to_msgpair_map[msg_in->header.stamp.toSec()];
    msgpair.addPclMsg(msg_in);
    if (msgpair.isCompleted()) {
        msgpair_queue.push_back(msgpair);
        if (msgpair_queue.size() > kMaxSyncedMsgPairs) {
            msgpair_queue.pop_front();
        }
        time_to_msgpair_map.erase(msg_in->header.stamp.toSec());
    } else {
        prunePendingMsgPairs();
    }
}

void odomCbk(const nav_msgs::Odometry::ConstPtr& msg_in) {
    std::unique_lock<std::mutex> lock(mutex_buf);
    PclAndOdomMsg& msgpair = time_to_msgpair_map[msg_in->header.stamp.toSec()];
    msgpair.addOdomMsg(msg_in);
    if (msgpair.isCompleted()) {
        msgpair_queue.push_back(msgpair);
        if (msgpair_queue.size() > kMaxSyncedMsgPairs) {
            msgpair_queue.pop_front();
        }
        time_to_msgpair_map.erase(msg_in->header.stamp.toSec());
    } else {
        prunePendingMsgPairs();
    }
}

void mapCbk(const sensor_msgs::PointCloud2ConstPtr& map_msg_in) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*map_msg_in, *ptcld_in);

    if (ptcld_in->empty()) {
        {
            std::lock_guard<std::mutex> lock_map(mutex_map);
            kGlobalMap->clear();
            global_map_init = false;
        }
        {
            std::lock_guard<std::mutex> lock_state(mutex_buf);
            resetMatchingState(true);
        }
        sensor_msgs::PointCloud2 empty_map_msg = *map_msg_in;
        empty_map_msg.header.frame_id = "map";
        empty_map_msg.header.stamp = ros::Time::now();
        pub_global_map.publish(empty_map_msg);
        setLocalizationMatchStatus("idle", "Empty route map received; using fallback startup world",
                                   true);
        ROS_WARN("Localization: Empty route map received, fallback to calibrated odometry.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_global_map(new pcl::PointCloud<pcl::PointXYZ>());
    downSizeFilterMap.setInputCloud(ptcld_in);
    downSizeFilterMap.filter(*new_global_map);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*new_global_map, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "map";

    {
        std::lock_guard<std::mutex> lock_map(mutex_map);
        *kGlobalMap = *new_global_map;
        global_map_init = !kGlobalMap->empty();
    }
    {
        std::lock_guard<std::mutex> lock_state(mutex_buf);
        resetMatchingState(true);
    }

    pub_global_map.publish(map_msg);
    setLocalizationMatchStatus("waiting_initialpose",
                               "WayPoint map world loaded; waiting for initial pose estimate",
                               true);
    ROS_INFO("Localization: Route map loaded from %s", kMapTopic.c_str());
    cout << "Global_Map_Size: " << ptcld_in->size()
         << "    After_Down_Size: " << new_global_map->size() << "\n";
}

void initPoseCbk(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    OdomStructf initial_world_pose;
    initial_world_pose.T = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                           msg->pose.pose.position.z);
    initial_world_pose.Q =
        Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
            .normalized();
    initial_world_pose.timestamp = msg->header.stamp;

    bool has_map{false};
    {
        std::lock_guard<std::mutex> lock_map(mutex_map);
        has_map = global_map_init;
    }
    if (!has_map) {
        {
            std::unique_lock<std::mutex> lock(mutex_buf);
            resetMatchingState(true);
        }
        setLocalizationMatchStatus(
            "idle",
            "Initial pose ignored because no valid WayPoint localization map is loaded",
            true);
        ROS_WARN("Localization: Initial pose ignored, no localization map loaded.");
        return;
    }

    std::unique_lock<std::mutex> lock(mutex_buf);
    pending_initial_world_pose = initial_world_pose;
    pending_initial_pose = true;
    initial_pose_init = false;
    resetMatchingState(false);
    beginLocalizationMatching();
    ROS_INFO("Localization: Initial world pose received from %s", kInitialPoseTopic.c_str());
}
void getCurrInfo(const PclAndOdomMsg& pair) {
    const auto& odom_msg = pair.odom_msg;
    curr_odom.T = Eigen::Vector3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                                  odom_msg->pose.pose.position.z);
    curr_odom.Q =
        Eigen::Quaternionf(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                           odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    curr_odom.timestamp = pair.timestamp();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tmp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*(pair.pcl_msg), *pcl_tmp);
    curr_pcl->clear();
    curr_pcl->points.reserve(pcl_tmp->size());
    // 将lio当前帧点云（待匹配点云）按限定在视野距离内
    for (int i = 0; i < pcl_tmp->size(); i++) {
        auto& pt_tmp = pcl_tmp->points[i];
        Eigen::Vector3f pt(pt_tmp.x, pt_tmp.y, pt_tmp.z);
        pt = pt - curr_odom.T;
        if (pt.x() * pt.x() + pt.y() * pt.y() + pt.z() * pt.z() <= kFovFar * kFovFar)
            curr_pcl->points.push_back(pt_tmp);
    }
}

void applyPendingInitialPoseIfNeeded() {
    std::unique_lock<std::mutex> lock(mutex_buf);
    if (!pending_initial_pose) {
        return;
    }

    if (kInitialPoseIsWorld) {
        diff_odom.setFromSE3(pending_initial_world_pose.M() * lsdc::inverse_SE3(curr_odom.M()));
    } else {
        diff_odom = pending_initial_world_pose;
    }
    diff_odom.timestamp = curr_odom.timestamp;
    initial_pose_init = true;
    pending_initial_pose = false;
    ROS_INFO("Localization: Initial pose converted to diff_odom using current calibrated odometry.");
}

visualization_msgs::Marker marker_msg;
void findTargetMapInFov() {
    pcl_in_fov->clear();

    // 待搜索中心相对于地图原点的坐标变换
    Eigen::Matrix4f tmp = diff_odom.M() * curr_odom.M();
    Eigen::Matrix4f coor_trans_M = lsdc::inverse_SE3(tmp);
    std::lock_guard<std::mutex> lock_map(mutex_map);
    int map_size = kGlobalMap->size();

    for (int i = 0; i < map_size; i++) {
        Eigen::Vector4f pt(kGlobalMap->points[i].x, kGlobalMap->points[i].y,
                           kGlobalMap->points[i].z, 1.f);
        pt = coor_trans_M * pt;
        if ((kFovRad > M_PI ? true : pt.x() >= 0) &&
            (pt.x() * pt.x() + pt.y() * pt.y() + pt.z() * pt.z() <= kFovFar * kFovFar) &&
            (abs(atan2(pt.y(), pt.x())) <= kFovRad / 2.0))
            pcl_in_fov->points.push_back(kGlobalMap->points[i]);
    }

    // 发布当前将要icp的目标点云
    // sensor_msgs::PointCloud2 pcl_in_fov_msg;
    // pcl_in_fov_ds->clear();
    // for (int i = 0; i < pcl_in_fov->size(); i += 10)
    //     pcl_in_fov_ds->points.push_back(pcl_in_fov->points[i]);
    // pcl::toROSMsg(*pcl_in_fov_ds, pcl_in_fov_msg);
    // pcl_in_fov_msg.header.stamp = ros::Time().now();
    // pcl_in_fov_msg.header.frame_id = "map";
    // pub_map_in_fov.publish(pcl_in_fov_msg);

    // 发布当前将要icp的目标点云区域
    OdomStructf cur_ost;
    cur_ost.setFromSE3(tmp);
    setPoseStamp(marker_msg, cur_ost.T, cur_ost.Q);
    pub_fov_sphere.publish(marker_msg);
}

struct IcpResult {
    bool has_converged;
    double fitness_score;
    Eigen::Matrix4f M;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_target(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_unused(new pcl::PointCloud<pcl::PointXYZ>());
IcpResult runIcp(double scale, Eigen::Matrix4f guess) {
    pcl_source->clear();
    pcl_target->clear();
    pcl_unused->clear();

    downSizeFilterIcp.setLeafSize(kFilterSizeSrc * scale, kFilterSizeSrc * scale,
                                  kFilterSizeSrc * scale);
    downSizeFilterIcp.setInputCloud(curr_pcl);
    downSizeFilterIcp.filter(*pcl_source);
    downSizeFilterIcp.setLeafSize(kFilterSizeMap * scale, kFilterSizeMap * scale,
                                  kFilterSizeMap * scale);
    downSizeFilterIcp.setInputCloud(pcl_in_fov);
    downSizeFilterIcp.filter(*pcl_target);

    // ICP Settings
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(scale * 1.0);
    icp.setMaximumIterations(kMaxIteration);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    icp.setInputSource(pcl_source);
    icp.setInputTarget(pcl_target);

    // Align pointclouds
    icp.align(*pcl_unused, guess);

    return IcpResult{icp.hasConverged(), icp.getFitnessScore(), icp.getFinalTransformation()};
}

void publishDiffOdom(const OdomStructf& odom) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "body";
    odom_msg.header.stamp = odom.timestamp;
    setPoseStamp(odom_msg.pose, odom.T, odom.Q);
    pub_diff_odom.publish(odom_msg);
}

bool calculateDiffOdom(uint64_t generation_snapshot) {
    double time_bgn = ros::Time::now().toSec();
    OdomStructf next_diff_odom;
    bool current_matching_init{false};
    {
        std::unique_lock<std::mutex> lock(mutex_buf);
        next_diff_odom = diff_odom;
        current_matching_init = matching_init;
    }

    findTargetMapInFov();
    if (pcl_in_fov->size() < 5) {
        ROS_WARN("Off The Global Map!");
        updateLocalizationMatchMetrics(-1.0);
        clearPendingSuccessConfirm(generation_snapshot);
        return current_matching_init;
    }

    if (kDebugDoNotMatch) {
        next_diff_odom.timestamp = curr_odom.timestamp;
        {
            std::unique_lock<std::mutex> lock(mutex_buf);
            if (generation_snapshot != match_generation) {
                return matching_init;
            }
            diff_odom = next_diff_odom;
        }
        publishDiffOdom(next_diff_odom);
        updateLocalizationMatchMetrics(0.0);
        return current_matching_init;
    }

    IcpResult result;
    result = runIcp(10, next_diff_odom.M());
    result = runIcp(5, result.M);
    result = runIcp(1, result.M);

    double time_end = ros::Time::now().toSec();
    if (kDisplayMatchingTime)
        cout << "Matching Time: " << time_end - time_bgn << "s\n";

    updateLocalizationMatchMetrics(result.fitness_score);

    if (result.has_converged && result.fitness_score < kFitnessThreshold) {
        bool next_matching_init{false};
        int next_init_match_cnt{0};
        {
            std::unique_lock<std::mutex> lock(mutex_buf);
            next_matching_init = matching_init;
            next_init_match_cnt = init_match_cnt;
        }
        if (!current_matching_init) {
            if (++next_init_match_cnt >= kSuccessConfirmCount) {
                ROS_INFO("Localization: Initial Match Success!");
                next_matching_init = true;
            }
        }
        next_diff_odom.setFromSE3(result.M);
        next_diff_odom.timestamp = curr_odom.timestamp;
        {
            std::unique_lock<std::mutex> lock(mutex_buf);
            if (generation_snapshot != match_generation) {
                return matching_init;
            }
            matching_init = next_matching_init;
            init_match_cnt = next_init_match_cnt;
            diff_odom = next_diff_odom;
        }
        // diff_odom.print();
        publishDiffOdom(next_diff_odom);
        return next_matching_init;
    } else {
        ROS_WARN("Match Failure!");
        clearPendingSuccessConfirm(generation_snapshot);
    }
    return current_matching_init;
}

void globalMatch() {
    ros::Rate match_rate(kFreq);
    while (ros::ok()) {
        PclAndOdomMsg synced_msgpair;
        uint64_t generation_snapshot{0};
        bool has_synced_msgpair{false};
        bool has_pending_initial_pose{false};
        {
            std::unique_lock<std::mutex> lock(mutex_buf);
            has_pending_initial_pose = pending_initial_pose;
            generation_snapshot = match_generation;
            if (!msgpair_queue.empty()) {
                synced_msgpair = msgpair_queue.back();
                msgpair_queue.clear();
                has_synced_msgpair = true;
            }
        }

        bool has_map{false};
        {
            std::lock_guard<std::mutex> lock_map(mutex_map);
            has_map = global_map_init;
        }

        bool has_initial_pose{false};
        bool already_matched{false};
        {
            std::unique_lock<std::mutex> lock(mutex_buf);
            has_initial_pose = initial_pose_init;
            already_matched = matching_init;
        }

        bool should_process_cloud =
            has_synced_msgpair && synced_msgpair.pcl_msg != nullptr &&
            synced_msgpair.odom_msg != nullptr && has_map &&
            (has_pending_initial_pose ||
             (has_initial_pose && (kContinuousMatch || !already_matched)));
        if (should_process_cloud) {
            getCurrInfo(synced_msgpair);
            applyPendingInitialPoseIfNeeded();
            std::unique_lock<std::mutex> lock(mutex_buf);
            has_initial_pose = initial_pose_init;
            already_matched = matching_init;
        }

        if (has_synced_msgpair && has_map && has_initial_pose &&
            (kContinuousMatch || !already_matched) && synced_msgpair.pcl_msg != nullptr &&
            synced_msgpair.odom_msg != nullptr) {
            bool match_success = calculateDiffOdom(generation_snapshot);

            publishInitMatchSuccess(match_success);
            if (match_success) {
                setLocalizationMatchStatus(
                    "localized",
                    "Localization matched; stable outputs are aligned to WayPoint map world",
                    true);
            } else {
                publishLocalizationMatchStatus(false);
            }
        }
        if (localizationMatchTimedOut()) {
            {
                std::unique_lock<std::mutex> lock(mutex_buf);
                initial_pose_init = false;
                pending_initial_pose = false;
                matching_init = false;
                init_match_cnt = 0;
                msgpair_queue.clear();
                time_to_msgpair_map.clear();
                ++match_generation;
            }
            publishInitMatchSuccess(false);
            setLocalizationMatchStatus(
                "failed",
                "Localization match timeout; waiting for a new initial pose estimate",
                true);
        } else if (localizationMatchInProgress()) {
            publishLocalizationMatchStatus(false);
        }
        match_rate.sleep();
    }
}

void setFovSphere() {
    marker_msg.header.frame_id = "/map";
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "fov_sphere";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.scale.x = kFovFar * 2;
    marker_msg.scale.y = kFovFar * 2;
    marker_msg.scale.z = kFovFar * 2;
    marker_msg.color.r = 0.1f;
    marker_msg.color.g = 1.f;
    marker_msg.color.b = 1.f;
    marker_msg.color.a = 0.3f;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_match_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<string>("drone_id", kDroneId, "2");
    private_nh.param<string>("initial_pose_topic", kInitialPoseTopic,
                             "/drone_" + kDroneId + "_initialpose");
    private_nh.param<string>("map_topic", kMapTopic, "/drone_" + kDroneId + "_localization_pcl");
    private_nh.param<string>("scan_topic", kScanTopic, "/cloud_registered_trans");
    private_nh.param<string>("odom_topic", kOdomTopic, "/Odometry_trans");
    private_nh.param<string>("diff_odom_topic", kDiffOdomTopic,
                             "/drone_" + kDroneId + "_diff_odom");
    private_nh.param<string>("init_match_success_topic", kInitMatchSuccessTopic,
                             "/drone_" + kDroneId + "_init_match_success");
    private_nh.param<string>("match_status_topic", kMatchStatusTopic,
                             "/drone_" + kDroneId + "_localization_match_status");
    nh.param<double>("localization/match_freq", kFreq, 1.0);
    nh.param<int>("localization/max_iteration", kMaxIteration, 20);
    nh.param<double>("localization/filter_size_map", kFilterSizeMap, 0.2);
    nh.param<double>("localization/filter_size_src", kFilterSizeSrc, 0.1);
    nh.param<double>("localization/fov_far", kFovFar, 100);
    nh.param<double>("localization/fov_ang", kFovAng, 360);
    nh.param<bool>("localization/display_matching_time", kDisplayMatchingTime, false);
    nh.param<bool>("localization/debug_do_not_match", kDebugDoNotMatch, false);
    nh.param<double>("localization/fitness_threshold", kFitnessThreshold, 0.3);
    nh.param<int>("localization/success_confirm_count", kSuccessConfirmCount, 10);
    nh.param<double>("localization/match_timeout_sec", kMatchTimeoutSec, 20.0);
    nh.param<double>("localization/status_publish_period_sec", kStatusPublishPeriodSec, 1.0);
    nh.param<bool>("localization/continuous_match", kContinuousMatch, false);
    nh.param<bool>("localization/initial_pose_is_world", kInitialPoseIsWorld, true);
    if (kSuccessConfirmCount < 1) {
        kSuccessConfirmCount = 1;
    }
    if (kFitnessThreshold <= 0.0 || !std::isfinite(kFitnessThreshold)) {
        kFitnessThreshold = 0.3;
    }
    if (kStatusPublishPeriodSec < 0.0 || !std::isfinite(kStatusPublishPeriodSec)) {
        kStatusPublishPeriodSec = 1.0;
    }
    if (kMatchTimeoutSec < 0.0 || !std::isfinite(kMatchTimeoutSec)) {
        kMatchTimeoutSec = 20.0;
    }
    kFovRad = kFovAng * M_PI / 180;
    downSizeFilterMap.setLeafSize(kFilterSizeMap, kFilterSizeMap, kFilterSizeMap);

    pub_global_map = nh.advertise<sensor_msgs::PointCloud2>("/map", 1, true);
    pub_map_in_fov = nh.advertise<sensor_msgs::PointCloud2>("/submap", 1);
    pub_diff_odom = nh.advertise<nav_msgs::Odometry>(kDiffOdomTopic, 1, true);
    pub_init_match_success = nh.advertise<std_msgs::Bool>(kInitMatchSuccessTopic, 1, true);
    pub_match_status = nh.advertise<std_msgs::String>(kMatchStatusTopic, 1, true);
    pub_fov_sphere = nh.advertise<visualization_msgs::Marker>("/fov_sphere_marker", 1);
    ros::Subscriber sub_global_map = nh.subscribe(kMapTopic, 1, mapCbk);
    ros::Subscriber sub_initial_pose = nh.subscribe(kInitialPoseTopic, 1, initPoseCbk);
    ros::Subscriber sub_scan = nh.subscribe(kScanTopic, 1, pclCbk);
    ros::Subscriber sub_odom = nh.subscribe(kOdomTopic, 10, odomCbk);
    setFovSphere();

    setLocalizationMatchStatus("idle", "Localization idle; waiting for WayPoint map world", true);
    ROS_INFO("Localization global_match topics: map=%s initial_pose=%s cloud=%s odom=%s diff=%s success=%s status=%s",
             kMapTopic.c_str(), kInitialPoseTopic.c_str(), kScanTopic.c_str(), kOdomTopic.c_str(),
             kDiffOdomTopic.c_str(), kInitMatchSuccessTopic.c_str(), kMatchStatusTopic.c_str());
    thread thread_global_match{globalMatch};
    ros::spin();
    if (thread_global_match.joinable()) {
        thread_global_match.join();
    }

    return 0;
}
