#include <geometry_msgs/PoseStamped.h>
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
#include "include/lsdc_tools.hpp"

class FilePreprocess {
   private:
    ros::NodeHandle nh;
    std::string input_bag_names = "", input_pose_names = "", input_path = "";
    std::string cloud_topic, gps_topic;
    std::vector<std::string> input_bag_name_vec, input_pose_name_vec;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_file_vec;
    std::vector<double> times_file_vec;

   public:
    void readPose() {
        for (std::string& name : input_pose_name_vec) {
            std::string pose_path =
                input_path + (input_path.back() == '/' ? "" : "/") + name + "_pose.txt";
            load_pose_with_time(pose_path, poses_file_vec, times_file_vec);
        }
        std::cout << "Sucessfully load pose with number: " << poses_file_vec.size() << std::endl;
    }

    void readFile(std::queue<std::shared_ptr<PclOdomPair>>& popair_que,
                  std::vector<nav_msgs::Odometry>& gps_que) {
        readPose();

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
                topics.push_back(cloud_topic);
                rosbag::View view(bag, rosbag::TopicQuery(topics));

                BOOST_FOREACH (rosbag::MessageInstance const m, view) {
                    auto msg = m.instantiate<sensor_msgs::PointCloud2>();
                    double laser_time = msg->header.stamp.toSec();
                    int pose_index = findPoseIndexUsingTime(times_file_vec, laser_time);
                    if (pose_index == -1)
                        continue;

                    std::shared_ptr<PclOdomPair> popair_ptr(new PclOdomPair());
                    popair_ptr->addPclMsg(msg);
                    popair_ptr->setOdom(poses_file_vec[pose_index].first,
                                        poses_file_vec[pose_index].second, std::to_string(i));
                    if (popair_ptr->is_completed())
                        popair_que.push(popair_ptr);
                }
            }
            if (1) {
                std::vector<std::string> topics;
                topics.push_back(gps_topic);
                rosbag::View view(bag, rosbag::TopicQuery(topics));

                BOOST_FOREACH (rosbag::MessageInstance const m, view) {
                    auto msg = m.instantiate<nav_msgs::Odometry>();
                    gps_que.push_back(*msg);
                }
            }
        }
    }

    FilePreprocess() {
        nh.param<std::string>("input_bag_names", input_bag_names, "");
        nh.param<std::string>("input_pose_names", input_pose_names, "");
        nh.param<std::string>("input_path", input_path, "");
        split(input_bag_names, input_bag_name_vec, "-");
        split(input_pose_names, input_pose_name_vec, "-");

        nh.param<std::string>("cloud_topic", cloud_topic, "/cloud_registered_body");
        nh.param<std::string>("gps_topic", gps_topic, "/rtk_odom_enu");
    }
    ~FilePreprocess() {}
};