#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <vector>
#include <cmath>

class MultiIKNode : public rclcpp::Node
{
public:
    MultiIKNode() : Node("multi_ik_node")
    {
        RCLCPP_INFO(this->get_logger(), "Avvio nodo Multi-IK...");
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    }

    void init()
    {
        // Crea PlanningSceneMonitor
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            this->shared_from_this(), "robot_description");

        if (!psm_->getPlanningScene())
        {
            RCLCPP_ERROR(this->get_logger(), "Errore: impossibile creare PlanningSceneMonitor.");
            return;
        }

        // Aggiorna PlanningScene
        psm_->requestPlanningSceneState("get_planning_scene");

        // Salva come membri della classe
        planning_scene_ = psm_->getPlanningScene();
        robot_model_ = planning_scene_->getRobotModel();  // ATTENZIONE: RobotModelConstPtr

        if (!robot_model_)
        {
            RCLCPP_ERROR(this->get_logger(), "Errore: robot model non valido!");
            return;
        }

        group_name_ = "right_arm";
        jmg_ = robot_model_->getJointModelGroup(group_name_);
        if (!jmg_)
            throw std::runtime_error("❌ Gruppo non trovato nel modello SRDF");

        generateGridAndComputeIK();
    }

private:
    void generateGridAndComputeIK()
    {
        // Parametri griglia
        double xmin = -0.25, xmax = -0.24;
        double ymin = -1.0, ymax = -0.2;
        double zmin = 0.2, zmax = 1.8;
        double step = 0.1;

        std::vector<geometry_msgs::msg::Pose> grid_points;
        tf2::Quaternion q;
        q.setRPY(0.0, M_PI_2, 0.0);

        for (double x = xmin; x <= xmax; x += step)
            for (double y = ymin; y <= ymax; y += step)
                for (double z = zmin; z <= zmax; z += step)
                {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = x;
                    pose.position.y = y;
                    pose.position.z = z;
                    pose.orientation = tf2::toMsg(q);
                    grid_points.push_back(pose);
                }

        RCLCPP_INFO(this->get_logger(), "Generati %zu punti nella griglia.", grid_points.size());

        moveit::core::RobotState start_state(robot_model_);
        start_state.setToDefaultValues();

        visualization_msgs::msg::MarkerArray markers;
        int id = 0;

        kinematics::KinematicsQueryOptions options;
        options.return_approximate_solution = false;

        for (const auto &target : grid_points)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Target pos=(%.3f, %.3f, %.3f)",
                        target.position.x, target.position.y, target.position.z);

            bool found = false;
            collision_detection::CollisionResult collision_res;

            for (int i = 0; i < 20; ++i)
            {
                start_state.setToRandomPositions(jmg_);
                start_state.update();

                moveit::core::GroupStateValidityCallbackFn no_constraint;
 
                found = start_state.setFromIK(jmg_, target, 0.1, no_constraint, options);

                if (found)
                {
                    std::vector<double> joint_values;
                    start_state.copyJointGroupPositions(jmg_, joint_values);

                    sensor_msgs::msg::JointState candidate_js;
                    candidate_js.name = jmg_->getVariableNames();
                    candidate_js.position = joint_values;

                    collision_detection::CollisionRequest collision_request;
                    collision_request.contacts = true;
                    collision_request.max_contacts = 100;
                    collision_request.max_contacts_per_pair = 5;
                    collision_request.group_name = group_name_;
                    planning_scene_->checkCollision(collision_request, collision_res, start_state);

                    if (collision_res.collision)
                    {
                        RCLCPP_INFO(this->get_logger(), "Collisione rilevata!");

                        auto start = this->now();
                        rclcpp::Rate rate(50);
                       
                            joint_state_pub_->publish(candidate_js);
                     
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Nessuna collisione rilevata.");
                        break;
                    }
                }
            }

            // Marker visualization
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = target;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
            marker.ns = this->get_name();
            marker.color.r = found ? 0.0 : 1.0;
            marker.color.g = (!found && !collision_res.collision) ? 0.0 : 1.0;
            marker.color.b = collision_res.collision ? 1.0 : 0.0;
            marker.color.a = 0.8;

            markers.markers.push_back(marker);
                    marker_pub_->publish(markers);

        }

        marker_pub_->publish(markers);
        RCLCPP_INFO(this->get_logger(), "Pubblicati marker per la griglia IK.");
    }

private:
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    moveit::core::RobotModelConstPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    const moveit::core::JointModelGroup *jmg_;
    std::string group_name_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiIKNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
