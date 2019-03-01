#include <ros/ros.h>
#include <ros/package.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <map>

#define POINTS 200  // Per 7000
#define DTMIE 0.008
//std::vector<std::string> jointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
static const std::string PLANNING_GROUP = "manipulator";

typedef std::vector<std::string> Line;
typedef std::vector<double> DColumn;
typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ActionClent;

using namespace boost::filesystem;

path packPath(ros::package::getPath("moveit_collision_inspector"));

void splitLine(const std::string & str, const char delimeter, std::vector<std::string> & out)
{
    std::stringstream ss(str);

    std::string s;
    while (std::getline(ss, s, delimeter)) {
        boost::trim_right(s);
        boost::trim_left(s);
        out.push_back(s);
    }
}

void splitLine(const std::string & str, const char delimeter, std::vector<double> & out)
{
    std::stringstream ss(str);

    std::string s;
    while (std::getline(ss, s, delimeter)) {
        boost::trim_right(s);
        boost::trim_left(s);
        out.push_back(std::stod(s));
    }
}

void readCsv(path p, const char delimeter, std::map<std::string, DColumn> & out) {

    ifstream ifs{p};
    std::string line;
    Line header;
    size_t m = 0;

    // Read header of csv file
    std::getline(ifs, line);
    splitLine(line, delimeter, header);
    m = header.size();

    std::vector<DColumn> data(m);
    std::vector<double> dline;
    while(std::getline(ifs, line)) {
        splitLine(line, delimeter, dline);
        for (size_t j = 0; j < m; ++j)
            data[j].push_back(dline[j]);
        dline.clear();
    }

    for (size_t j = 0; j < m; ++j)
        out[header[j]] = data[j];
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "inspector");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string dataFilePath = "";
    std::vector<std::string> jointNames;
    ros::param::get("~file_path", dataFilePath);
    ros::param::get("~joint_names", jointNames);

    // MoveIt!
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    ActionClent trajectoryActionClient("/execute_trajectory", true);

    path relativeFilePath(dataFilePath.c_str());
    path filePath = packPath / relativeFilePath;
    std::cout << filePath << std::endl;
    std::map<std::string, DColumn> data;

    readCsv(filePath, ',', data);
    std::cout << data["shoulder_pan_joint"].size() << std::endl;

    moveit_msgs::ExecuteTrajectoryGoal goal;
    moveit_msgs::ExecuteTrajectoryAction tmp;
    trajectory_msgs::JointTrajectory traj;
    size_t n = data.begin()->second.size(), m = data.size();
    std::vector<double> initPos(m);
    size_t step = 7000/POINTS;

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize(m);

    for (size_t j = 0; j < m; ++j) {
        initPos[j] = data[jointNames[j]][0];
    }

    traj.joint_names = jointNames;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            p.positions[j] = data[jointNames[j]][i];
            p.time_from_start = ros::Duration(i*DTMIE);
        }
        traj.points.push_back(p);
    }

    // Move to initial poistion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setJointValueTarget(initPos);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ros::Duration(5).sleep();
    move_group.move();

    for (size_t i = 0; i < n; i += step) {
        move_group.setJointValueTarget(traj.points[i].positions);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Trajectory is no valid");
            break;
        }
        move_group.move();
    }



    // goal.trajectory.joint_trajectory = traj;

    // trajectoryActionClient.waitForServer();
    // trajectoryActionClient.sendGoal(goal);

    // // Wait for the action to return
    // bool finished_before_timeout = trajectoryActionClient.waitForResult();

    // if (finished_before_timeout) {
    //     actionlib::SimpleClientGoalState state = trajectoryActionClient.getState();
    //     ROS_INFO("Action finished: %s", state.toString().c_str());
    // } else ROS_ERROR("Action did not finish before the time out.");

    return 1;
}
