/**
 * 轨迹回放节点：读取保存的CSV轨迹 → 发布 sandbox_msgs/Trajectory → pure_pursuit跟踪
 * 用法：rosrun condition_planner trajectory_replay_node _csv_file:=path/to/trajectory_v1.csv _target:=0
 */
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <vector>

#include <sandbox_msgs/Trajectory.h>
#include <sandbox_msgs/TrajectoryPoint.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_replay_node");
    ros::NodeHandle nh("~");

    // 参数
    std::string csv_file;
    int target_id = 0;
    bool loop = false;

    nh.param<std::string>("csv_file", csv_file, "");
    nh.param("target", target_id, 0);
    nh.param("loop", loop, false);

    if (csv_file.empty()) {
        ROS_ERROR("No csv_file specified! Use _csv_file:=path/to/trajectory.csv");
        return 1;
    }

    // 读CSV
    std::ifstream ifs(csv_file);
    if (!ifs.is_open()) {
        ROS_ERROR("Cannot open: %s", csv_file.c_str());
        return 1;
    }

    sandbox_msgs::Trajectory traj_msg;
    traj_msg.target = target_id;
    traj_msg.header.frame_id = "world";

    std::string line;
    std::getline(ifs, line); // 跳过表头: t,x,y,theta,v,phi,a,omega,s,kappa
    int count = 0;
    while (std::getline(ifs, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> vals;
        while (std::getline(ss, token, ',')) {
            vals.push_back(std::stod(token));
        }
        if (vals.size() < 5) continue;

        sandbox_msgs::TrajectoryPoint pt;
        pt.time = vals[0];
        pt.x = vals[1];
        pt.y = vals[2];
        pt.yaw = vals[3];
        pt.velocity = vals[4];
        pt.acceleration = (vals.size() > 6) ? vals[6] : 0.0;
        traj_msg.points.push_back(pt);
        count++;
    }

    ROS_INFO("Loaded %d trajectory points from: %s", count, csv_file.c_str());
    if (count == 0) {
        ROS_ERROR("Empty trajectory!");
        return 1;
    }

    ROS_INFO("  Duration: %.2f s", traj_msg.points.back().time);
    ROS_INFO("  Target vehicle: %d", target_id);

    // 发布
    ros::Publisher traj_pub = nh.advertise<sandbox_msgs::Trajectory>("/traj", 1, true);

    // 等待订阅者连接
    ros::Duration(1.0).sleep();

    // 等待用户确认再发送轨迹
    printf("\n");
    printf("==========================================\n");
    printf("  Trajectory loaded, waiting to send...\n");
    printf("  Align car to START in RViz first.\n");
    printf("  Press [Enter] to publish trajectory.\n");
    printf("==========================================\n");
    std::cin.get();

    traj_msg.header.stamp = ros::Time::now();
    traj_pub.publish(traj_msg);
    printf(">>> Trajectory published to /traj <<<\n");

    if (loop) {
        ros::Rate rate(1.0);
        while (ros::ok()) {
            traj_msg.header.stamp = ros::Time::now();
            traj_pub.publish(traj_msg);
            rate.sleep();
        }
    } else {
        ros::spin(); // 保持节点存活(latched publisher)
    }

    return 0;
}
