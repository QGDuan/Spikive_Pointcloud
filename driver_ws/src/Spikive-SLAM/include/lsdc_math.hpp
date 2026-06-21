#ifndef LSDC_MATH_HPP
#define LSDC_MATH_HPP

namespace lsdc {

Eigen::AngleAxisd kRot90 = Eigen::AngleAxisd((90 * M_PI / 180), Eigen::Vector3d::UnitZ());

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

template <typename T>
Eigen::Matrix<T, 3, 3> EulerAngle_to_R(T yaw, T pitch, T roll) {
    return (Eigen::AngleAxis<T>(yaw, Eigen::Matrix<T, 3, 1>::UnitZ()) *
            Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY()) *
            Eigen::AngleAxis<T>(roll, Eigen::Matrix<T, 3, 1>::UnitX()))
        .toRotationMatrix();
}

template <typename T>
Eigen::Matrix<T, 4, 4> RT_to_SE3(Eigen::Matrix<T, 3, 3> R_in, Eigen::Matrix<T, 3, 1> T_in) {
    Eigen::Matrix<T, 1, 4> v4t;
    v4t << 0, 0, 0, 1;
    Eigen::Matrix<T, 4, 4> se3;
    se3 << R_in, T_in, v4t;
    return se3;
}

template <typename T>
Eigen::Matrix<T, 3, 3> R_from_SE3(Eigen::Matrix<T, 4, 4> se3) {
    return se3.topLeftCorner(3, 3);
}

template <typename T>
Eigen::Matrix<T, 3, 1> T_from_SE3(Eigen::Matrix<T, 4, 4> se3) {
    return se3.topRightCorner(3, 1);
}

template <typename T>
Eigen::Matrix<T, 4, 4> inverse_SE3(Eigen::Matrix<T, 4, 4> se3) {
    Eigen::Matrix<T, 1, 4> v4t;
    v4t << 0, 0, 0, 1;
    Eigen::Matrix<T, 4, 4> inv;
    inv << (se3.block(0, 0, 3, 3)).transpose(),
        -(se3.block(0, 0, 3, 3)).transpose() * (se3.block(0, 3, 3, 1)), v4t;
    return inv;
}

};  // namespace lsdc

#endif