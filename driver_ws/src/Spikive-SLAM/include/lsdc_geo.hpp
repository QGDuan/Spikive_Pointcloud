#ifndef LSDC_GEO_HPP
#define LSDC_GEO_HPP

#include <Python.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
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

#include <GeographicLib/LocalCartesian.hpp>
#include "Commons/WGS84toCartesian.hpp"
#include "Commons/transfer.hpp"
#include "lsdc_math.hpp"
#include "odom_struct.hpp"

namespace lsdc {
class LsdcGeographicLib {
   private:
   public:
    Eigen::Vector3d origin_L, T_rtk_wrt_body, E_rtk_wrt_body, T_car_wrt_rtk, E_car_wrt_rtk;
    Eigen::Quaterniond origin_Q, Q_rtk_wrt_body, Q_car_wrt_rtk;

    LsdcGeographicLib() {}
    ~LsdcGeographicLib() {}

    void getRtkFromOdom(Eigen::Vector3d odom_T,
                        Eigen::Quaterniond odom_Q,
                        Eigen::Vector3d& geo_L,
                        Eigen::Quaterniond& geo_Q) {
        OdomStructd origin_wrt_world(Eigen::Vector3d(0, 0, 0), lsdc::kRot90 * origin_Q);
        OdomStructd body_wrt_origin(odom_T, odom_Q);
        OdomStructd rtk_wrt_body(T_rtk_wrt_body, Q_rtk_wrt_body);
        OdomStructd car_wrt_rtk(T_car_wrt_rtk, Q_car_wrt_rtk);
        OdomStructd car_wrt_world(origin_wrt_world.M() * body_wrt_origin.M() * rtk_wrt_body.M() *
                                  car_wrt_rtk.M());

        // ins的朝向为x轴和正北方向的正向角
        double lat{0}, lon{0}, alt{0};
        GeographicLib::LocalCartesian geo_converter(origin_L.x(), origin_L.y(), origin_L.z());
        geo_converter.Reverse(car_wrt_world.T.x(), car_wrt_world.T.y(), car_wrt_world.T.z(), lat,
                              lon, alt);

        geo_L = Eigen::Vector3d(lat, lon, alt);
        geo_Q = lsdc::kRot90.inverse() * car_wrt_world.Q;
    }

    void getOdomFromRtk(Eigen::Vector3d geo_L,
                        Eigen::Quaterniond geo_Q,
                        Eigen::Vector3d& odom_T,
                        Eigen::Quaterniond& odom_Q) {
        // ins的朝向为x轴和正北方向的正向角
        double dx{0}, dy{0}, dz{0};
        GeographicLib::LocalCartesian geo_converter(origin_L.x(), origin_L.y(), origin_L.z());
        geo_converter.Forward(geo_L.x(), geo_L.y(), geo_L.z(), dx, dy, dz);
        Eigen::Vector3d t = Eigen::Vector3d(dx, dy, dz);

        OdomStructd origin_wrt_world(Eigen::Vector3d(0, 0, 0), lsdc::kRot90 * origin_Q);
        OdomStructd rtk_wrt_world(t, lsdc::kRot90 * geo_Q);
        OdomStructd rtk_wrt_body(T_rtk_wrt_body, Q_rtk_wrt_body);
        OdomStructd body_wrt_origin(origin_wrt_world.M().inverse() * rtk_wrt_world.M() *
                                    rtk_wrt_body.M().inverse());

        odom_T = body_wrt_origin.T;
        odom_Q = body_wrt_origin.Q;
    }

    void getOriginFromRtk(Eigen::Vector3d geo_L,
                          Eigen::Quaterniond geo_Q,
                          Eigen::Vector3d& L,
                          Eigen::Quaterniond& Q) {
        OdomStructd rtk_wrt_world(Eigen::Vector3d(0, 0, 0), lsdc::kRot90 * geo_Q);
        OdomStructd rtk_wrt_body(T_rtk_wrt_body, Q_rtk_wrt_body);
        OdomStructd body_wrt_world(rtk_wrt_world.M() * rtk_wrt_body.M().inverse());

        double lat{0}, lon{0}, alt{0};
        GeographicLib::LocalCartesian geo_converter(geo_L.x(), geo_L.y(), geo_L.z());
        geo_converter.Reverse(body_wrt_world.T.x(), body_wrt_world.T.y(), body_wrt_world.T.z(), lat,
                              lon, alt);

        L = Eigen::Vector3d(lat, lon, alt);
        Q = lsdc::kRot90.inverse() * body_wrt_world.Q;
    }

    void setRtkParamFromCfg(ros::NodeHandle& nh) {
        std::vector<double> origin_L_vec, origin_Q_vec, T_rtk_wrt_body_vct, E_rtk_wrt_body_vct,
            T_car_wrt_rtk_vct, E_car_wrt_rtk_vct;

        nh.param<std::vector<double>>("localization/origin_L", origin_L_vec,
                                      std::vector<double>({0, 0, 0}));
        nh.param<std::vector<double>>("localization/origin_Q", origin_Q_vec,
                                      std::vector<double>({0, 0, 0, 0}));
        origin_L = Eigen::Vector3d(origin_L_vec[0], origin_L_vec[1], origin_L_vec[2]);
        origin_Q = origin_Q_vec == std::vector<double>({0, 0, 0, 0})
                       ? Eigen::Quaterniond(kRot90.inverse())
                       : Eigen::Quaterniond(origin_Q_vec[3], origin_Q_vec[0], origin_Q_vec[1],
                                            origin_Q_vec[2]);

        nh.param<std::vector<double>>("rtk/T_rtk_wrt_body", T_rtk_wrt_body_vct,
                                      std::vector<double>({0, 0, 0}));
        nh.param<std::vector<double>>("rtk/E_rtk_wrt_body", E_rtk_wrt_body_vct,
                                      std::vector<double>({0, 0, 0}));
        T_rtk_wrt_body << T_rtk_wrt_body_vct[0], T_rtk_wrt_body_vct[1], T_rtk_wrt_body_vct[2];
        E_rtk_wrt_body << E_rtk_wrt_body_vct[0] * M_PI / 180, E_rtk_wrt_body_vct[1] * M_PI / 180,
            E_rtk_wrt_body_vct[2] * M_PI / 180;
        Q_rtk_wrt_body =
            Eigen::Quaterniond(Eigen::AngleAxisd(E_rtk_wrt_body(0), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(E_rtk_wrt_body(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(E_rtk_wrt_body(2), Eigen::Vector3d::UnitX()));

        nh.param<std::vector<double>>("rtk/T_car_wrt_rtk", T_car_wrt_rtk_vct,
                                      std::vector<double>({0, 0, 0}));
        nh.param<std::vector<double>>("rtk/E_car_wrt_rtk", E_car_wrt_rtk_vct,
                                      std::vector<double>({0, 0, 0}));
        T_car_wrt_rtk << T_car_wrt_rtk_vct[0], T_car_wrt_rtk_vct[1], T_car_wrt_rtk_vct[2];
        E_car_wrt_rtk << E_car_wrt_rtk_vct[0] * M_PI / 180, E_car_wrt_rtk_vct[1] * M_PI / 180,
            E_car_wrt_rtk_vct[2] * M_PI / 180;
        Q_car_wrt_rtk =
            Eigen::Quaterniond(Eigen::AngleAxisd(E_car_wrt_rtk(0), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(E_car_wrt_rtk(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(E_car_wrt_rtk(2), Eigen::Vector3d::UnitX()));
    }
};
};  // namespace lsdc

// namespace lsdc
#endif