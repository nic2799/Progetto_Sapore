#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace SaveYaml {

void savePlanToYAML(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                    const geometry_msgs::msg::PoseStamped& target_pose,
                    const std::string& plan_name,
                    int& plan_counter_,
                    const std::string& plans_output_file_,
                    double gripper_state); // <--- NUOVO PARAMETRO
}