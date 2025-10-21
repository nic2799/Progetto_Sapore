#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ReachabilityNodes : public rclcpp::Node
{
public:
    ReachabilityNodes()
        : Node("reachability_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo Reachability creato (inizializzazione differita).");
    }

    bool initialize(const std::string &planning_group = "left_arm", const std::string &reference_frame = "world")
    {
        try
        {
            // Crea MoveGroupInterface
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), planning_group);
            move_group_->setPoseReferenceFrame(reference_frame);
            move_group_->setEndEffectorLink("left_tcp");


            // Avvia il monitor dello stato del robot (importantissimo)
            move_group_->startStateMonitor();

            // Crea PlanningSceneMonitor
            planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                shared_from_this(), "robot_description");

            if (!planning_scene_monitor_->getPlanningScene())
            {
                RCLCPP_ERROR(this->get_logger(), "Errore: impossibile inizializzare il PlanningSceneMonitor (planning scene vuota).");
                return false;
            }

            // Avvia i monitor
            planning_scene_monitor_->startSceneMonitor();
            planning_scene_monitor_->startWorldGeometryMonitor();
            planning_scene_monitor_->startStateMonitor();

            planning_scene_ = planning_scene_monitor_->getPlanningScene();

            RCLCPP_INFO(this->get_logger(), "MoveGroup e PlanningSceneMonitor inizializzati correttamente.");
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Eccezione in initialize(): %s", e.what());
            return false;
        }
    }

   void runReachabilityTest()
{
    double x_min = -1.0, x_max = -0.25;
    double y_min = -1.0, y_max = 2.0;
    double z_min = 0.0, z_max = 1.78;
    double step  = 0.25;

    if (!move_group_)
    {
        RCLCPP_ERROR(this->get_logger(), "move_group_ non inizializzato. Chiamare initialize() prima.");
        return;
    }

    auto current_state_ptr = move_group_->getCurrentState();
    if (!current_state_ptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Impossibile ottenere current state dal MoveGroup.");
        return;
    }

    const moveit::core::JointModelGroup* joint_model_group =
        current_state_ptr->getJointModelGroup(move_group_->getName());

    if (!joint_model_group)
    {
        RCLCPP_ERROR(this->get_logger(), "Impossibile ottenere JointModelGroup per '%s'.",
                     move_group_->getName().c_str());
        return;
    }

    int marker_id = 0;

    for (double x = x_min; x <= x_max; x += step)
    {
        for (double y = y_min; y <= y_max; y += step)
        {
            for (double z = z_min; z <= z_max; z += step)
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = z;
                pose.orientation.w = 1.0; // placeholder

                moveit::core::RobotState robot_state = *move_group_->getCurrentState();

                bool found_ik = false;
                bool in_collision = false;
                bool success = false;

                // callback per fermare la ricerca IK appena trova una soluzione valida
                auto validity_callback = [&](moveit::core::RobotState* state,
                                             const moveit::core::JointModelGroup* jmg,
                                             const double* ik_solution) {
                    // aggiorna pianificazione per verificare collisioni
                    planning_scene_monitor_->updateFrameTransforms();
                    auto scene = planning_scene_monitor_->getPlanningScene();
                    if (!scene) return false;
                    return !scene->isStateColliding(*state, move_group_->getName());
                };

                // Tenta più volte orientamenti diversi (MoveIt genera orientazioni random)
                found_ik = robot_state.setFromIK(
                    joint_model_group, pose, 100, 0.1, validity_callback);

                if (!found_ik)
                {
                    RCLCPP_DEBUG(this->get_logger(),
                                 "IK fallita per punto x=%.2f y=%.2f z=%.2f", x, y, z);
                }
                else
                {
                    planning_scene_monitor_->updateFrameTransforms();
                    auto scene = planning_scene_monitor_->getPlanningScene();

                    if (!scene)
                    {
                        RCLCPP_ERROR(this->get_logger(), "PlanningScene non disponibile.");
                    }
                    else
                    {
                        in_collision = scene->isStateColliding(robot_state, move_group_->getName());
                        if (!in_collision)
                        {
                            move_group_->setPoseTarget(pose);
                            moveit::planning_interface::MoveGroupInterface::Plan plan;
                            success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                        }
                    }
                }

                // ROSSO: non raggiungibile
                if (!found_ik || in_collision)
                    addVisualMarker(pose, marker_id++, 1.0, 0.0, 0.0);
                // BLU: raggiungibile ma non pianificabile
                else if (!success)
                    addVisualMarker(pose, marker_id++, 0.0, 0.0, 1.0);
                // VERDE: tutto ok
                else
                    addVisualMarker(pose, marker_id++, 0.0, 1.0, 0.0);
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Test raggiungibilità completato!");
}

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    planning_scene::PlanningScenePtr planning_scene_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    void addVisualMarker(const geometry_msgs::msg::Pose& pose, int id,
                         double r, double g, double b)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "reachability";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = pose;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Crea il nodo
    auto node = std::make_shared<ReachabilityNodes>();

    // Crea un executor separato
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Lancia l’executor in un thread separato
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // Inizializza MoveGroup e PlanningSceneMonitor
    if (!node->initialize("left_arm", "world"))
    {
        RCLCPP_ERROR(rclcpp::get_logger("reachability_main"), "Inizializzazione fallita. Esco.");
        rclcpp::shutdown();
        executor_thread.join();
        return 1;
    }

    // Esegui il test di raggiungibilità
    node->runReachabilityTest();

    // Chiudi tutto
    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}
