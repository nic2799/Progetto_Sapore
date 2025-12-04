// MoveItHelper.hpp

#ifndef MOVEIT_HELPER_HPP
#define MOVEIT_HELPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/utils/moveit_error_code.h>

// Definizione di una struttura per i parametri specifici del Pick&Place
struct PickPlaceParams {
    double object_size;
    double approach_distance;
    double lift_distance;
};

// Namespace per organizzare i metodi MoveIt
namespace moveit_utils {

class MoveItHelper {
public:
    // ...existing code...
    // Imposta lo stato iniziale del robot da joint_names e positions
    void setStartStateFromJointPositions(const std::vector<std::string>& joint_names, const std::vector<double>& positions);
public:
    // Utility methods for state and pose targets
    void setStartStateToCurrentState();
    void clearPoseTargets();
    MoveItHelper(rclcpp::Node::SharedPtr node, 
                 const std::string& planning_group_name);
    
    // Inizializzazione MoveIt (come il tuo metodo init)
    bool init();

    // Spawn e Rimozione Collision Object
    std::string spawnCollisionObject(const geometry_msgs::msg::Pose& pose, 
                                     double height, double radius);
    void removeCollisionObject(const std::string& object_id);

    // Gestione vincoli (spostata qui)
    moveit_msgs::msg::Constraints createNoRotationYZConstraint();
    void setPathConstraints(const moveit_msgs::msg::Constraints& constraints);
    void clearPathConstraints();
    
    // Pianificazione con fallback (riutilizzata in tutte le fasi)
    moveit::core::MoveItErrorCode planWithFallback(
        const geometry_msgs::msg::Pose& target_pose, 
        moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const std::string& phase_name);
        
    // Esecuzione di un piano
    bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

    // Movimento cartesiano
    double computeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                moveit_msgs::msg::RobotTrajectory& trajectory);
    
    // Rotazione giunto specifica (per il Pouring)
    moveit::core::MoveItErrorCode planPouringRotation(
        double final_z_rotation_rad,
        moveit::planning_interface::MoveGroupInterface::Plan& plan,
        const std::vector<double>& joint_positions_start);
        
    // Attacca/Stacca oggetto
    bool attachObject(const std::string& object_id);
    void detachObject(const std::string& object_id);

    bool planToNamedTarget(const std::string& target_name, 
                                     moveit::planning_interface::MoveGroupInterface::Plan& plan);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::string planning_group_name_;
};

} // namespace moveit_utils

#endif // MOVEIT_HELPER_HPP