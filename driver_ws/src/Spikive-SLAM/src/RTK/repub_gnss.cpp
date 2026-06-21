#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "Commons/WGS84toCartesian.hpp"
#include "Commons/convert_coordinates.hpp"
#include "Commons/transfer.hpp"

#include "lsdc_geo.hpp"
#include "lsdc_math.hpp"
#include "lsdc_slam/bywire_chassis_state.h"

using namespace std;

class RepubGnssMsg {
   private:
    ros::NodeHandle nh;
    ros::Subscriber sub_slam_odom, sub_ins, sub_gnss, sub_init_match_success, sub_lio_odom;
    ros::Publisher pub_global_odom, pub_gnss_odom;
    string ins_topic = "/ins_lla";
    ofstream lat_lon_out;
    string kLastGnssFile = string(string(ROOT_DIR)) + "Lsdc_Repub/last_pose.txt";

    Eigen::Vector3d curr_T;  // x,y,z
    Eigen::Quaterniond curr_Q;

    Eigen::Vector3d ins_L;  // L: lat, lon, alt
    Eigen::Quaterniond ins_Q;

    Eigen::Vector3d curr_lio_T, last_lio_T, lio_velo;
    int gear;
    double vehicle_speed;

    bool init_match_success{false};

    lsdc::LsdcGeographicLib lgl;

    bool judgeState() {
        // cout << "gear: " << gear << "    vehi_speed: " << vehicle_speed
        //      << "    lio_speed: " << lio_velo.norm()
        //      << "    diff: " << abs(vehicle_speed - lio_velo.norm()) << "\n";
        // if (abs(vehicle_speed - lio_velo.norm()) > 0.3)
        //     return false;
        return true;
    }

    void slamOdomCbk(const nav_msgs::OdometryConstPtr& odom_msg) {
        if (init_match_success) {
            // 判断是否明显漂移
            geometry_msgs::PoseStamped msg_global;
            curr_T = Eigen::Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
            curr_Q = Eigen::Quaterniond(
                odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

            if (judgeState()) {
                msg_global.header.frame_id = "NORMAL";
            } else {
                msg_global.header.frame_id = "ERROR";
                ROS_ERROR("ERROR STATE");
            }

            // repub经纬度
            Eigen::Vector3d geo_L;     // L: lat, lon, alt
            Eigen::Quaterniond geo_Q;  // Q: w, x, y, z
            lgl.getRtkFromOdom(curr_T, curr_Q, geo_L, geo_Q);
            msg_global.header.stamp = odom_msg->header.stamp;
            msg_global.pose.position.x = geo_L.x();
            msg_global.pose.position.y = geo_L.y();
            msg_global.pose.position.z = geo_L.z();
            msg_global.pose.orientation.w = geo_Q.w();
            msg_global.pose.orientation.x = geo_Q.x();
            msg_global.pose.orientation.y = geo_Q.y();
            msg_global.pose.orientation.z = geo_Q.z();
            pub_global_odom.publish(msg_global);

            if (msg_global.header.frame_id == "ERROR")
                return;

            // 打印当前odom到文件
            lat_lon_out.open(kLastGnssFile, std::ios::out);
            lat_lon_out << std::setprecision(15) << geo_L.x() << " " << geo_L.y() << " "
                        << geo_L.z() << std::endl;
            lat_lon_out << std::setprecision(15) << geo_Q.x() << " " << geo_Q.y() << " "
                        << geo_Q.z() << " " << geo_Q.w() << std::endl;
            lat_lon_out.close();

            if (0) {  // 打印ins和localization定位的经纬度、转角，及他们的差。
                double ins_yaw = lsdc::Q_to_EulerAngle(ins_Q).z() * 180 / M_PI,
                       lsdc_yaw = lsdc::Q_to_EulerAngle(geo_Q).z() * 180 / M_PI;
                cout << "ins :  " << setprecision(10) << "  " << ins_L.x() << "  " << ins_L.y()
                     << "  " << ins_yaw << "\n";
                cout << "lsdc:  " << setprecision(10) << "  " << geo_L.x() << "  " << geo_L.y()
                     << "  " << lsdc_yaw << "  "
                     << "\n";

                double dx{0}, dy{0}, dz{0}, da{0};
                convert_coordinates::latlon_diff_to_meters(ins_L.x(), ins_L.y(), geo_L.x(),
                                                           geo_L.y(), dx, dy);
                GeographicLib::LocalCartesian geo_converter(ins_L.x(), ins_L.y(), ins_L.z());
                geo_converter.Forward(geo_L.x(), geo_L.y(), geo_L.z(), dx, dy, dz);

                da = lsdc_yaw - ins_yaw;
                cout << "   d:  " << dx << "  " << dy << "  " << da << "\n" << std::endl;
            }
        }
    }

    void insCallback(const nav_msgs::Odometry::Ptr& msg) {
        ins_L = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
        ins_Q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                   msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    }
    void gnssCallback(const lsdc_slam::bywire_chassis_stateConstPtr& msg) {
        ins_L = Eigen::Vector3d(msg->latitude, msg->longitude, 0);
        ins_Q = Eigen::Quaterniond(lsdc::EulerAngle_to_R(-(msg->azimuth) * M_PI / 180, 0.0, 0.0));

        gear = (int)msg->gear;
        vehicle_speed = (msg->vehicle_speed) / 3.6;
    }

    void initSuccessCallback(const std_msgs::BoolConstPtr& msg) {
        if (!init_match_success && bool(msg->data))
            init_match_success = true;
    }

    void lioOdomCbk(const nav_msgs::OdometryConstPtr& odom_msg) {
        curr_lio_T = Eigen::Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
        lio_velo = (curr_lio_T - last_lio_T) / 0.1;
        last_lio_T = curr_lio_T;
    }

   public:
    RepubGnssMsg() {
        ROS_INFO("Start GNSS_Repub");

        lgl.setRtkParamFromCfg(nh);

        // ros::Duration(10).sleep();

        sub_slam_odom = nh.subscribe<nav_msgs::Odometry>("/localization_odom", 100,
                                                         &RepubGnssMsg::slamOdomCbk, this);
        sub_lio_odom =
            nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, &RepubGnssMsg::lioOdomCbk, this);
        sub_ins = nh.subscribe(ins_topic, 100, &RepubGnssMsg::insCallback, this);
        sub_gnss = nh.subscribe("/bywire_chassis", 100, &RepubGnssMsg::gnssCallback, this);
        sub_init_match_success =
            nh.subscribe("/init_match_success", 100, &RepubGnssMsg::initSuccessCallback, this);
        pub_global_odom = nh.advertise<geometry_msgs::PoseStamped>("/global_pose", 100);
        pub_gnss_odom = nh.advertise<sensor_msgs::NavSatFix>("/gnss", 100);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "repub_gnss_node");

    RepubGnssMsg rpm;
    ros::spin();

    return 0;
}