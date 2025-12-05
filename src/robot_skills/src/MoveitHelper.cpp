// MoveItHelper.cpp
#include "MoveitHelper.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <vector>

namespace moveit_utils {
void MoveItHelper::setStartStateFromJointPositions(const std::vector<std::string>& joint_names, const std::vector<double>& positions) {
    moveit::core::RobotState start_state(move_group_->getRobotModel());
    for (size_t i = 0; i<joint_names.size(); ++i)
    {

    std::cout << "  Giunto " << joint_names[i] << ": " << positions[i] << " rad (" <<positions[i] * 180.0 / M_PI << "°)" << std::endl;
    }
    start_state.setVariablePositions(joint_names, positions);
    start_state.update();
    move_group_->setStartState(start_state);
}

void MoveItHelper::setStartStateToCurrentState() {
    move_group_->setStartStateToCurrentState();
}

void MoveItHelper::clearPoseTargets() {
    move_group_->clearPoseTargets();
}

MoveItHelper::MoveItHelper(rclcpp::Node::SharedPtr node, const std::string& planning_group_name)
  : node_(node), planning_group_name_(planning_group_name) {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_name_);
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
}

bool MoveItHelper::init() {
    return move_group_ != nullptr && planning_scene_interface_ != nullptr;
}

std::string MoveItHelper::spawnCollisionObject(const geometry_msgs::msg::Pose& pose, double height, double radius) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();
    collision_object.id = "object_" + std::to_string(rand());

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions = {height, radius};

    collision_object.primitives.push_back(primitive);
   // collision_object.primitive_poses.push_back(pose);
  //  collision_object.operation = collision_object.ADD;

    tf2::Quaternion q;
    tf2::Quaternion q_old;
    tf2::Quaternion q_new;
    q.setRPY(0, M_PI/2, 0);
    q_new=q_old*q;


    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(q_new);
    
   // pose.orientation=q_new;
    geometry_msgs::msg::Pose oriented_pose = pose;
    oriented_pose.orientation=quat_msg;
    collision_object.primitive_poses.push_back(oriented_pose);
    collision_object.operation = collision_object.ADD;
    std::cout << "abbiamo inserito oggetto" << std::endl;

    planning_scene_interface_->applyCollisionObject(collision_object);
    return collision_object.id;
}

void MoveItHelper::removeCollisionObject(const std::string& object_id) {
    planning_scene_interface_->removeCollisionObjects({object_id});
}

moveit_msgs::msg::Constraints MoveItHelper::createNoRotationYZConstraint() {
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group_->getEndEffectorLink();
    ocm.header.frame_id = move_group_->getPlanningFrame();
    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
    ocm.orientation = current_pose.pose.orientation;
    ocm.absolute_x_axis_tolerance = 2 * M_PI;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = 1.0;
    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    return constraints;
}

void MoveItHelper::setPathConstraints(const moveit_msgs::msg::Constraints& constraints) {
    move_group_->setPathConstraints(constraints);
}

void MoveItHelper::clearPathConstraints() {
    move_group_->clearPathConstraints();
}

moveit::core::MoveItErrorCode MoveItHelper::planWithFallback(
    const geometry_msgs::msg::Pose& target_pose,
    moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& phase_name) {
    std::vector<std::string> planners = {"ESTkConfigDefault","ESTkConfigDefault", "RRTstarkConfigDefault", "PRMkConfigDefault", "RRTstarkConfigDefault", "PRMkConfigDefault", "PRMkConfigDefault"};
    for (const auto& planner : planners) {
        RCLCPP_INFO(node_->get_logger(), "🔧 Tentativo planning '%s' con %s...", phase_name.c_str(), planner.c_str());
        move_group_->setPoseTarget(target_pose);
        move_group_->setPlannerId(planner);
        move_group_->setPlanningTime(30.0);
        move_group_->setNumPlanningAttempts(100);
        move_group_->setGoalTolerance(0.02);
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), " Planning '%s' riuscito con %s", phase_name.c_str(), planner.c_str());
            return moveit::core::MoveItErrorCode::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "  Planning '%s' fallito con %s, provo planner alternativo...", phase_name.c_str(), planner.c_str());
        }
    }
    RCLCPP_ERROR(node_->get_logger(), " Planning '%s' fallito con tutti i planner", phase_name.c_str());
    return moveit::core::MoveItErrorCode::FAILURE;
}

bool MoveItHelper::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

double MoveItHelper::computeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                          moveit_msgs::msg::RobotTrajectory& trajectory) {
    const double eef_step = 0.005;
    const double jump_threshold = 0.0;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    return fraction;
}

moveit::core::MoveItErrorCode MoveItHelper::planPouringRotation(
    double final_z_rotation_rad,
    moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::vector<double>& joint_positions_start) {
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(100.0);
    move_group_->setGoalPositionTolerance(0.03);
    move_group_->setGoalOrientationTolerance(0.2);
    move_group_->setNumPlanningAttempts(60);
    //move_group_->setEndEffectorLink("right_tcp");//lo prende dal group del srdf
    move_group_->setGoalJointTolerance(0.05);  // Tolleranza stretta per rotazione precisa

    std::vector<double> joint_positions = joint_positions_start;
    int joint_a7_index = 6; // Settimo elemento (A7)
    double posizione_iniziale_a7 = joint_positions[joint_a7_index];
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        std::cout << "  Giunto " << i << ": " << joint_positions[i] << " rad (" << joint_positions[i] * 180.0 / M_PI << "°)" << std::endl;
    }
    joint_positions[joint_a7_index] += final_z_rotation_rad;
    move_group_->setJointValueTarget(joint_positions);

    moveit::core::MoveItErrorCode result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_->clearPoseTargets();
        return moveit::core::MoveItErrorCode::FAILURE;
    }
    move_group_->clearPoseTargets();
    return moveit::core::MoveItErrorCode::SUCCESS;
}

bool MoveItHelper::attachObject(const std::string& object_id) {
    std::vector<std::string> touch_links = {
        "right_finger_left", 
        "right_finger_right", 
        "right_hand",
        "left_finger_left", 
        "left_finger_right",
        "left_hand"
    };
return (move_group_->attachObject(object_id, "", touch_links) == moveit::core::MoveItErrorCode::SUCCESS);
}

void MoveItHelper::detachObject(const std::string& object_id) {
    move_group_->detachObject(object_id);
}


// MoveItHelper.cpp

bool MoveItHelper::planToNamedTarget(const std::string& target_name, 
                                     moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    
    // 1. Imposta il target usando il nome stringa
    move_group_->setNamedTarget(target_name);
    
    // 2. Opzionale: dai un po' più di tempo a MoveIt se il PC è lento
    move_group_->setPlanningTime(5.0); 

    // 3. Pianifica
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
        RCLCPP_INFO(node_->get_logger(), " Planning target '%s' riuscito per gruppo %s", 
                    target_name.c_str(), planning_group_name_.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), " Planning target '%s' fallito per gruppo %s", 
                    target_name.c_str(), planning_group_name_.c_str());
    }
    return success;
}




} // namespace moveit_utils
