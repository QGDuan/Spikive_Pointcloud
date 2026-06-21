#ifndef ODOM_STRUCT_HPP
#define ODOM_STRUCT_HPP

#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <iostream>

#include "lsdc_math.hpp"

struct OdomStructf {
    ros::Time timestamp;
    Eigen::Vector3f T;  // translation
    Eigen::Quaternionf Q;
    Eigen::Matrix3f R() { return Q.normalized().toRotationMatrix(); }  // rotation
    Eigen::Matrix4f M() { return lsdc::RT_to_SE3(R(), T); }            // se3
    OdomStructf(Eigen::Vector3f T, Eigen::Quaternionf Q) {
        this->T = T;
        this->Q = Q;
    }
    OdomStructf(Eigen::Matrix4f m) {
        this->Q = Eigen::Quaternionf(lsdc::R_from_SE3(m));
        this->T = lsdc::T_from_SE3(m);
    }
    OdomStructf() {
        this->Q = Eigen::Quaternionf::Identity();
        this->T = Eigen::Vector3f::Zero();
    }
    void setFromSE3(Eigen::Matrix4f m) {
        Q = Eigen::Quaternionf(lsdc::R_from_SE3(m));
        T = lsdc::T_from_SE3(m);
    }
    void print() {
        std::cout << "T: " << T.transpose() << "\n"
                  << "Q: " << Q.coeffs().transpose() << "\n\n";
    }
};

struct OdomStructd {
    ros::Time timestamp;
    Eigen::Vector3d T;  // translation
    Eigen::Quaterniond Q;
    Eigen::Matrix3d R() { return Q.normalized().toRotationMatrix(); }  // rotation
    Eigen::Matrix4d M() { return lsdc::RT_to_SE3(R(), T); }            // se3
    OdomStructd(Eigen::Vector3d T, Eigen::Quaterniond Q) {
        this->T = T;
        this->Q = Q;
    }
    OdomStructd(Eigen::Matrix4d m) {
        this->Q = Eigen::Quaterniond(lsdc::R_from_SE3(m));
        this->T = lsdc::T_from_SE3(m);
    }
    OdomStructd() {
        this->Q = Eigen::Quaterniond::Identity();
        this->T = Eigen::Vector3d::Zero();
    }
    void setFromSE3(Eigen::Matrix4d m) {
        Q = Eigen::Quaterniond(lsdc::R_from_SE3(m));
        T = lsdc::T_from_SE3(m);
    }
    void print() {
        std::cout << "T: " << T.transpose() << "\n"
                  << "Q: " << Q.coeffs().transpose() << "\n\n";
    }
};

#endif