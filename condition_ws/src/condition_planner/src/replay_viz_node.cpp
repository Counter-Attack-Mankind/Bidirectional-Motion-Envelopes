/**
 * 回放可视化节点：画场景(边界+障碍物) + 轨迹起终点车身足迹+朝向
 * 帮助用户对齐实车位置
 */
#include <ros/ros.h>
#include <fstream>
#include <sstream>

#include "common/math/pose.h"
#include "common/math/polygon2d.h"
#include "common/math/box2d.h"
#include "visualization_plot.h"
#include "my_env.h"

using namespace common::math;

int main(int argc, char** argv) {
    ros::init(argc, argv, "replay_viz_node");
    ros::NodeHandle nh("~");

    VisualizationPlot::Init(nh, "world", "/condition_planner_markers");

    // 加载环境
    std::string env_file, csv_file;
    nh.param<std::string>("env_file", env_file, "");
    nh.param<std::string>("csv_file", csv_file, "");

    // 车辆参数
    double back_edge, wheel_base, length, width;
    nh.param("back_edge_to_center", back_edge, 0.032);
    nh.param("wheel_base", wheel_base, 0.143);
    nh.param("length", length, 0.211);
    nh.param("width", width, 0.191);

    // 等RViz连接
    ros::Duration(2.0).sleep();

    // 画场景
    MyEnvironment env;
    if (env.Read(env_file)) {
        // 边界
        std::vector<double> bx, by;
        for (const auto& p : env.points) {
            bx.push_back(p.x()); by.push_back(p.y());
        }
        VisualizationPlot::Plot(bx, by, 0.01, Color::White, 100, "Border");

        // 障碍物(实心)
        for (size_t i = 0; i < env.obstacles.size(); i++) {
            VisualizationPlot::PlotFilledPolygon(env.obstacles[i], Color::Red,
                                                  200 + i, "Obstacle_" + std::to_string(i));
        }
        ROS_INFO("Scene: %zu border pts, %zu obstacles", env.points.size(), env.obstacles.size());
    }

    // 读CSV轨迹的首末点
    if (!csv_file.empty()) {
        std::ifstream ifs(csv_file);
        if (ifs.is_open()) {
            std::string line;
            std::getline(ifs, line); // 表头

            double first_x=0, first_y=0, first_th=0;
            double last_x=0, last_y=0, last_th=0;
            bool has_first = false;
            while (std::getline(ifs, line)) {
                std::stringstream ss(line);
                std::string tok;
                std::vector<double> v;
                while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
                if (v.size() < 4) continue;
                if (!has_first) {
                    first_x = v[1]; first_y = v[2]; first_th = v[3];
                    has_first = true;
                }
                last_x = v[1]; last_y = v[2]; last_th = v[3];
            }

            if (has_first) {
                // 起点车身足迹 + 朝向
                Pose sp(first_x, first_y, first_th);
                double dist = length / 2.0 - back_edge;
                Box2d sbox(sp.extend(dist), sp.theta(), length, width);
                VisualizationPlot::PlotPolygon(Polygon2d(sbox), 0.005, Color::Blue, 1000, "Start");
                double arr = 0.15;
                VisualizationPlot::Plot(
                    {first_x, first_x + arr*cos(first_th)},
                    {first_y, first_y + arr*sin(first_th)},
                    0.008, Color::Blue, 1002, "StartHeading");

                // 终点车身足迹 + 朝向
                Pose gp(last_x, last_y, last_th);
                Box2d gbox(gp.extend(dist), gp.theta(), length, width);
                VisualizationPlot::PlotPolygon(Polygon2d(gbox), 0.005, Color::Cyan, 1001, "Goal");
                VisualizationPlot::Plot(
                    {last_x, last_x + arr*cos(last_th)},
                    {last_y, last_y + arr*sin(last_th)},
                    0.008, Color::Cyan, 1003, "GoalHeading");

                // 画轨迹线
                ifs.clear(); ifs.seekg(0);
                std::getline(ifs, line); // 跳过表头
                std::vector<double> tx, ty;
                while (std::getline(ifs, line)) {
                    std::stringstream ss(line);
                    std::string tok;
                    std::vector<double> v;
                    while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
                    if (v.size() >= 3) { tx.push_back(v[1]); ty.push_back(v[2]); }
                }
                VisualizationPlot::Plot(tx, ty, 0.008, Color::Green, 2, "Saved Trajectory");

                ROS_INFO("Trajectory: start(%.2f, %.2f, %.1f°) -> goal(%.2f, %.2f, %.1f°), %zu pts",
                         first_x, first_y, first_th*180/M_PI,
                         last_x, last_y, last_th*180/M_PI, tx.size());
            }
        } else {
            ROS_WARN("Cannot open CSV: %s", csv_file.c_str());
        }
    }

    VisualizationPlot::Trigger();
    ROS_INFO("Replay visualization ready. Align your car to the blue START box.");

    ros::spin();
    return 0;
}
