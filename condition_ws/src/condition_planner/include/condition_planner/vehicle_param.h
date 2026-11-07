#pragma once
#include <cmath>

struct VehicleParam {
    // 几何参数
    double rear_hang = 0.032;
    double front_hang = 0.036;
    double wheel_base = 0.143;
    double length = 0.211;
    double width = 0.191;

    // 动力学约束
    double max_velocity = 0.25;
    double max_acceleration = 0.1;
    double max_deceleration = -0.1;

    // 转向约束
    double max_phi = 0.3491;     // rad (~20°)
    double min_phi = -0.3491;
    double max_omega = 0.1;      // 前轮转角变化率 rad/s

    // 碰撞检测 (两个版本按需选用)
    double vehicle_buffer = 0.0;      // 精准碰撞检测
    double vehicle_buffer_safe = 0.02; // 膨胀碰撞检测

    // 派生参数
    double min_turn_radius() const {
        return wheel_base / std::tan(max_phi);  // ≈0.393m
    }

    double c2x() const {
        return 0.5 * length - rear_hang;  // 几何中心相对后轴偏移
    }

    // 条件分支约束阈值
    double phi_threshold() const { return max_phi * 0.5; }   // 0.1746 rad
    double v_limit() const { return max_velocity * 0.5; }    // 0.125 m/s
};
