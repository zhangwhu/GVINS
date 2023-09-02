/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ROTATION_H
#define ROTATION_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rotation {

public:
    static Quaterniond matrix2quaternion(const Matrix3d &matrix) {
        return Quaterniond(matrix);
    }

    static Matrix3d quaternion2matrix(const Quaterniond &quaternion) {
        return quaternion.toRotationMatrix();
    }

    // ZYX旋转顺序, 前右下的IMU, 输出RPY
    static Vector3d matrix2euler(const Eigen::Matrix3d &dcm) {
        Vector3d euler;

        euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

        if (dcm(2, 0) <= -0.999) {
            euler[0] = atan2(dcm(2, 1), dcm(2, 2));
            euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
        } else if (dcm(2, 0) >= 0.999) {
            euler[0] = atan2(dcm(2, 1), dcm(2, 2));
            euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
        } else {
            euler[0] = atan2(dcm(2, 1), dcm(2, 2));
            euler[2] = atan2(dcm(1, 0), dcm(0, 0));
        }

        // heading 0~2PI
        if (euler[2] < 0) {
            euler[2] = M_PI * 2 + euler[2];
        }

        return euler;
    }

    static Vector3d quaternion2euler(const Quaterniond &quaternion) {
        return matrix2euler(quaternion.toRotationMatrix());
    }

    static Quaterniond rotvec2quaternion(const Vector3d &rotvec) {
        double angle = rotvec.norm();
        Vector3d vec = rotvec.normalized();
        return Quaterniond(Eigen::AngleAxisd(angle, vec));
    }

    static Vector3d quaternion2vector(const Quaterniond &quaternion) {
        Eigen::AngleAxisd axisd(quaternion);
        return axisd.angle() * axisd.axis();
    }

    // RPY --> C_b^n, 旋转不可交换, ZYX顺序
    static Matrix3d euler2matrix(const Vector3d &euler) {
        return Matrix3d(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                        Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    static Quaterniond euler2quaternion(const Vector3d &euler) {
        return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    // 反对称矩阵
    static Matrix3d skewSymmetric(const Vector3d &vector) {
        Matrix3d mat;
        mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
        return mat;
    }

    static Eigen::Matrix4d quaternionleft(const Quaterniond &q) {
        Eigen::Matrix4d ans;
        ans(0, 0)             = q.w();
        ans.block<1, 3>(0, 1) = -q.vec().transpose();
        ans.block<3, 1>(1, 0) = q.vec();
        ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + skewSymmetric(q.vec());
        return ans;
    }

    static Eigen::Matrix4d quaternionright(const Quaterniond &p) {
        Eigen::Matrix4d ans;
        ans(0, 0)             = p.w();
        ans.block<1, 3>(0, 1) = -p.vec().transpose();
        ans.block<3, 1>(1, 0) = p.vec();
        ans.block<3, 3>(1, 1) = p.w() * Eigen::Matrix3d::Identity() - skewSymmetric(p.vec());
        return ans;
    }

    static Eigen::MatrixXd quat2eulererror(const Quaterniond &p) {
        double q0 = p.w(), q1 = p.x(), q2 = p.y(), q3 = p.z();
        double t2,  t3,  t4,  t5,  t6,  t7,  t8,  t9,  t10, t11, t12, t13, t14, t15, t16, t17, 
               t18, t19, t20, t21, t22, t23, t24, t25, t26, t27, t28, t29, t30, t31; 
               
        Eigen::MatrixXd ans = Eigen::MatrixXd::Zero(3, 4);
        t8 = q0 * q1 * 2.0;         t9 = q2 * q3 * 2.0;
        t2 = t8 + t9;               t4 = q0 * q0;
        t5 = q1 * q1;               t6 = q2 * q2;
        t7 = q3 * q3;               t3 = t4 - t5 - t6 + t7;
        t10 = t3 * t3;              t11 = t2 * t2;
        t12 = t10 + t11;            t13 = 1.0 / t12;
        t14 = 1.0 / t3;             t15 = 1.0 / (t3 * t3);
        t17 = q0 * q2 * 2.0;        t18 = q1 * q3 * 2.0;
        t16 = t17 - t18;            t19 = t16 * t16;
        t20 = -t19 + 1.0;           t21 = 1.0 / sqrt(t20);
        t24 = q0 * q3 * 2.0;        t25 = q1 * q2 * 2.0;
        t22 = t24 + t25;            t23 = t4 + t5 - t6 - t7;
        t26 = t23 * t23;            t27 = t22 * t22;
        t28 = t26 + t27;            t29 = 1.0 / t28;
        t30 = 1.0 / t23;            t31 = 1.0 / (t23 * t23);

        ans(0, 3) = t10 * t13 * (q1 * t14 * 2.0 - q0 * t2 * t15 * 2.0);
        ans(1, 3) = q2 * t21 *  2.0;
        ans(2, 3) = t26 * t29 * (q3 * t30 * 2.0 - q0 * t22 * t31 * 2.0);

        ans(0, 0) = t10 * t13 * (q0 * t14 * 2.0 + q1 * t2 * t15 * 2.0);
        ans(1, 0) = q3 * t21 * -2.0;
        ans(2, 0) = t26 * t29 * (q2 * t30 * 2.0 - q1 * t22 * t31 * 2.0);

        ans(0, 1) = t10 * t13 * (q3 * t14 * 2.0 + q2 * t2 * t15 * 2.0);
        ans(1, 1) = q0 * t21 *  2.0;
        ans(2, 1) = t26 * t29 * (q1 * t30 * 2.0 + q2 * t22 * t31 * 2.0);

        ans(0, 2) = t10 * t13 * (q2 * t14 * 2.0 - q3 * t2 * t15 * 2.0);
        ans(1, 2) = q1 * t21 * -2.0;
        ans(2, 2) = t26 * t29 * (q0 * t30 * 2.0 + q3 * t22 * t31 * 2.0);
        
        return ans;
    }
};

#endif // ROTATION_H
