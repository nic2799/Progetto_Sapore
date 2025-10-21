#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
class ReachabilityNodes : public rclcpp::Node
{
public:
    ReachabilityNodes()
        : Node("reachability_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);//publisher per i marker di visualizzazione che ci serviranno per visualizzare i punti raggiungibili e non raggiungibili  

    }

    bool initialize(const std::string &planning_group = "right_arm", const std::string &reference_frame = "world")
    {
        try
        {
            // Crea MoveGroupInterface
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), planning_group);
            move_group_->setPoseReferenceFrame(reference_frame);
            move_group_->setEndEffectorLink("right_tcp");


            // Avvia il monitor dello stato del robot (importantissimo)
            move_group_->startStateMonitor();

            // Crea PlanningSceneMonitor
            planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                shared_from_this(), "robot_description");
                /*Planning scene è un oggetto che permette di immagazzinare la rappresentazione del mondo circorstante e dello stato del robot
                lo stato interno dell'oggetto planning_scene è gestito dal planning scene monitor(PSM) consente lettura e scrittura dello stato
                */

            if (!planning_scene_monitor_->getPlanningScene())
            {
                RCLCPP_ERROR(this->get_logger(), "Errore: impossibile inizializzare il PlanningSceneMonitor (planning scene vuota).");
                return false;
            }

            // Avvia i monitor
            planning_scene_monitor_->startSceneMonitor();//fa  subscriber a /planning_scene(aggiorna la scena)  // tiene allineata la scena globale
            planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");//chiede esplicitamente l'aggiornamento della scena di pianificazione
            planning_scene_monitor_->startWorldGeometryMonitor();//si iscrive a topic come collision object per aggiornare ambiente
            planning_scene_monitor_->startStateMonitor();

            planning_scene_ = planning_scene_monitor_->getPlanningScene();//restituisce puntatore alla planning scene gestita dal PSM

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
        double x_min = -0.25, x_max = -0.24;
        double y_min = -1.0,  y_max = -0.2;
        double z_min = 0.0,   z_max = 1.8;
        double step  = 0.05;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 3.14/2, 0); // Rotazione di 90 gradi intorno all'asse Y
        geometry_msgs::msg::Quaternion ros_quaternion;
        tf2::convert(quaternion, ros_quaternion); // Conversio
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
                    pose.orientation = ros_quaternion;

                    moveit::core::RobotState robot_state = *move_group_->getCurrentState();
                    bool found_ik = robot_state.setFromIK(joint_model_group, pose, 0.05);

                    if (!found_ik)
                    {
                        //addVisualMarker(pose, marker_id++, 1.0, 0.0, 0.0);
                        continue;
                    }

                    planning_scene_monitor_->updateFrameTransforms();
                    auto scene = planning_scene_monitor_->getPlanningScene();

                    if (!scene) { RCLCPP_ERROR(get_logger(), "No planning scene"); return; }

                    const auto &world = scene->getWorld();
                    auto ids = world->getObjectIds();
                    RCLCPP_INFO(get_logger(), "Oggetti nel world: %zu", ids.size());
                             for (const auto &id : ids){
                                 RCLCPP_INFO(get_logger(), " - %s", id.c_str());}


                    bool in_collision = scene->isStateColliding(robot_state, move_group_->getName());
                    if (in_collision)
                    {
                        addVisualMarker(pose, marker_id++, 0.0, 0.0, 1.0);
                        continue;
                    }else if(!in_collision && found_ik)
                    {
                        addVisualMarker(pose, marker_id++, 0.0, 1.0, 0.0);
                        continue;
                    }

                    move_group_->setPoseTarget(pose);
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                   // bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

                    //if (success)
                      //  addVisualMarker(pose, marker_id++, 0.0, 1.0, 0.0);
                    //else
                      //  addVisualMarker(pose, marker_id++, 0.0, 0.0, 1.0);
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
    if (!node->initialize("right_arm", "world"))
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
