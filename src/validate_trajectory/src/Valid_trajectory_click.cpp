#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

struct PlanInfo
{
    int id;                                    // ID del piano
    std::string name;                          // Nome del piano
    geometry_msgs::msg::Point target_position; // Dove il robot vuole arrivare
   
};

class ValidateTrajectoryClickNode : public rclcpp::Node
{
public:
    ValidateTrajectoryClickNode() : Node("validate_trajectory_click_node")
    {
        this->declare_parameter<std::string>("yaml_file", "");
        this->declare_parameter<std::string>("group_name", "");
        this->declare_parameter<double>("max_click_distance", 0.5); // 50cm soglia
        
        group_name_ = this->get_parameter("group_name").as_string();
        yaml_file_ = this->get_parameter("yaml_file").as_string();
        max_click_distance_ = this->get_parameter("max_click_distance").as_double();
        
        if (yaml_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), " Parametro 'yaml_file' non specificato!");
            return;
        }
        
        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&ValidateTrajectoryClickNode::pointCallback, this, std::placeholders::_1));
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization_marker_array", 10);
        
        display_state_pub_ = this->create_publisher<moveit_msgs::msg::DisplayRobotState>(
            "/display_robot_state", 10);
        
        RCLCPP_INFO(this->get_logger(), "  Nodo avviato");
        RCLCPP_INFO(this->get_logger(), "   File YAML: %s", yaml_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "   Group: %s", group_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "   Max click distance: %.2f m", max_click_distance_);
        
        // Carica SOLO le info base (id + target_position)
        if (!loadPlanInfoFromYAML()) {
            RCLCPP_ERROR(this->get_logger(), "  Caricamento info piani fallito!");
            return;
        }
        
       publishTargetMarkers();
        RCLCPP_INFO(this->get_logger(), "\n  Usa 'Publish Point' in RViz per selezionare un piano!");
    }
    
    bool init()
    {
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(), "robot_description");
        
        if (!psm_->getPlanningScene()) {
            RCLCPP_ERROR(this->get_logger(), "Planning Scene non disponibile!");
            return false;
        }
        
        psm_->startSceneMonitor();
        psm_->startStateMonitor();
        psm_->startWorldGeometryMonitor();
        
        planning_scene_ = psm_->getPlanningScene();
        
        RCLCPP_INFO(this->get_logger(), "Planning Scene Monitor inizializzato");
        return true;
    }

private:
    // Carica SOLO id + target_position (veloce, leggero)
    bool loadPlanInfoFromYAML()
    {
        RCLCPP_INFO(this->get_logger(), "\n Caricamento info piani da: %s", yaml_file_.c_str());
        
        try {
            YAML::Node yaml_data = YAML::LoadFile(yaml_file_);
            
            if (!yaml_data["plans"]) {
                RCLCPP_ERROR(this->get_logger(), "File YAML non contiene 'plans'");
                return false;
            }
            
            plans_info_.clear();
            
            for (const auto& plan_node : yaml_data["plans"]) {
                PlanInfo info;
                
                info.id = plan_node["id"].as<int>();
                
                if (plan_node["name"]) {
                    info.name = plan_node["name"].as<std::string>();
                }
                
                // LEGGI SOLO target_position (leggero!)
                auto target_pos = plan_node["target_pose"]["position"];
                info.target_position.x = target_pos["x"].as<double>();
                info.target_position.y = target_pos["y"].as<double>();
                info.target_position.z = target_pos["z"].as<double>();
                
                plans_info_.push_back(info);
                
                RCLCPP_INFO(this->get_logger(), 
                           "   ✓ Piano %d: target = (%.3f, %.3f, %.3f)",
                           info.id,
                           info.target_position.x,
                           info.target_position.y,
                           info.target_position.z);
            }
            
            RCLCPP_INFO(this->get_logger(), " Caricati %zu piani\n", plans_info_.size());
            return true;
            
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), " Errore parsing YAML: %s", e.what());
            return false;
        }
    }
    
    // Carica traiettoria SOLO per il piano selezionato
    bool loadTrajectoryForPlan(int plan_id, trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        RCLCPP_INFO(this->get_logger(), " Caricamento traiettoria per piano %d...", plan_id);
        
        try {
            YAML::Node yaml_data = YAML::LoadFile(yaml_file_);
            
            // Cerca il piano specifico
            for (const auto& plan_node : yaml_data["plans"]) {
                if (plan_node["id"].as<int>() == plan_id) {
                    
                    if (!plan_node["trajectory"]) {
                        RCLCPP_ERROR(this->get_logger(), "❌ Piano %d non ha traiettoria", plan_id);
                        return false;
                    }
                    
                    auto traj_node = plan_node["trajectory"];
                    
                    // Joint names
                    trajectory.joint_names.clear();
                    for (const auto& name : traj_node["joint_names"]) {
                        trajectory.joint_names.push_back(name.as<std::string>());
                    }
                    
                    // Waypoints (points)
                    trajectory.points.clear();
                    for (const auto& point_node : traj_node["points"]) {
                        trajectory_msgs::msg::JointTrajectoryPoint point;
                        
                        // Positions
                        for (const auto& pos : point_node["positions"]) {
                            point.positions.push_back(pos.as<double>());
                        }
                        
                        // Velocities (opzionale)
                        if (point_node["velocities"]) {
                            for (const auto& vel : point_node["velocities"]) {
                                point.velocities.push_back(vel.as<double>());
                            }
                        }
                        
                        // Accelerations (opzionale)
                        if (point_node["accelerations"]) {
                            for (const auto& acc : point_node["accelerations"]) {
                                point.accelerations.push_back(acc.as<double>());
                            }
                        }
                        
                        // Time from start
                        point.time_from_start.sec = 0;
                        point.time_from_start.nanosec = 
                            static_cast<uint32_t>(point_node["time_from_start"].as<double>() * 1e9);
                        
                        trajectory.points.push_back(point);
                    }
                    
                    RCLCPP_INFO(this->get_logger(), 
                               " Traiettoria caricata: %zu waypoints", 
                               trajectory.points.size());
                    return true;
                }
            }
            
            RCLCPP_ERROR(this->get_logger(), " Piano %d non trovato nel file YAML", plan_id);
            return false;
            
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), " Errore caricamento traiettoria: %s", e.what());
            return false;
        }
    }
    
    void publishTargetMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (const auto& info : plans_info_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "group_" + group_name_;
            marker.id = info.id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position = info.target_position;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
            
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.8;
            
            marker.lifetime = rclcpp::Duration(0, 0);
            marker_array.markers.push_back(marker);
            
        
        }
        
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), " Pubblicati %zu marker", plans_info_.size());
    }
    
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                   "\n  Click: (%.3f, %.3f, %.3f) in '%s'",
                   msg->point.x, msg->point.y, msg->point.z, 
                   msg->header.frame_id.c_str());
        
        if (plans_info_.empty()) {
            RCLCPP_WARN(this->get_logger(), "  Nessun piano caricato!");
            return;
        }
        
        // Trova piano più vicino
        int closest_plan_id = -1;
        double min_distance = std::numeric_limits<double>::max();
        
        for (const auto& info : plans_info_) {
            double distance = computeDistance(msg->point, info.target_position);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_plan_id = info.id;
            }
        }
        
        if (closest_plan_id < 0) {
            RCLCPP_ERROR(this->get_logger(), "Nessun piano trovato!");
            return;
        }
        
        //  CONTROLLO SOGLIA: Ignora click troppo lontani da TUTTI i marker di questo nodo
        if (min_distance > max_click_distance_) {
            RCLCPP_DEBUG(this->get_logger(), 
                        "  Click troppo lontano (%.3f m > %.3f m), ignorato da questo nodo [%s]",
                        min_distance, max_click_distance_, group_name_.c_str());
            return;
        }
        
        // Trova info piano
        const PlanInfo* selected_info = nullptr;
        for (const auto& info : plans_info_) {
            if (info.id == closest_plan_id) {
                selected_info = &info;
                break;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "\n ========================================");
        RCLCPP_INFO(this->get_logger(), "   PIANO SELEZIONATO: %d", selected_info->id);
        RCLCPP_INFO(this->get_logger(), "   ========================================");
        RCLCPP_INFO(this->get_logger(), "   Nome: %s", selected_info->name.c_str());
        RCLCPP_INFO(this->get_logger(), "   Target: (%.3f, %.3f, %.3f)",
                   selected_info->target_position.x,
                   selected_info->target_position.y,
                   selected_info->target_position.z);
        RCLCPP_INFO(this->get_logger(), "   Distanza: %.3f m", min_distance);
        
        highlightSelectedPlan(selected_info->id);
        
        // Cerca se esiste un piano consecutivo con la stessa target_pose (per sequenze approach+pouring)
        int next_plan_id = selected_info->id + 1;//pouring  ha id consecutivo
        bool has_consecutive_plan = false;
        const PlanInfo* next_plan_info = nullptr;
        
        for (const auto& info : plans_info_) {
            if (info.id == next_plan_id) {
                // Verifica se la target_position è la stessa (tolleranza 1cm)
                double distance_to_next = computeDistance(selected_info->target_position, info.target_position);
                if (distance_to_next < 0.01) {  // 1cm tolleranza
                    has_consecutive_plan = true;
                    next_plan_info = &info;
                    RCLCPP_INFO(this->get_logger(), "    Rilevato piano consecutivo %d (stesso target)", next_plan_id);
                    RCLCPP_INFO(this->get_logger(), "      Nome: %s", info.name.c_str());
                    break;
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "   ========================================\n");
        
        // Carica e visualizza primo piano (approach)
        trajectory_msgs::msg::JointTrajectory trajectory1;
        if (loadTrajectoryForPlan(selected_info->id, trajectory1)) {
            RCLCPP_INFO(this->get_logger(), "   Piano %d - Waypoints: %zu", selected_info->id, trajectory1.points.size());
            visualizeTrajectory(trajectory1);
        } else {
            RCLCPP_ERROR(this->get_logger(), " Impossibile caricare traiettoria piano %d!", selected_info->id);
            return;
        }
        
        // Se esiste piano consecutivo, caricalo e visualizzalo automaticamente
        if (has_consecutive_plan && next_plan_info != nullptr) {
            RCLCPP_INFO(this->get_logger(), "\n Caricamento piano consecutivo %d...", next_plan_id);
            
            // Pausa breve tra le due visualizzazioni
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            
            trajectory_msgs::msg::JointTrajectory trajectory2;
            if (loadTrajectoryForPlan(next_plan_id, trajectory2)) {
                RCLCPP_INFO(this->get_logger(), "    Piano %d - Waypoints: %zu", next_plan_id, trajectory2.points.size());
                visualizeTrajectory(trajectory2);
                RCLCPP_INFO(this->get_logger(), "\n Sequenza completa visualizzata (piani %d + %d)\n", 
                           selected_info->id, next_plan_id);
            } else {
                RCLCPP_WARN(this->get_logger(), "  Impossibile caricare piano consecutivo %d", next_plan_id);
            }
        }
    }
    
    double computeDistance(const geometry_msgs::msg::Point& p1, 
                          const geometry_msgs::msg::Point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    void highlightSelectedPlan(int selected_id)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (const auto& info : plans_info_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "plan_targets";
            marker.id = info.id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position = info.target_position;
            marker.pose.orientation.w = 1.0;
            
            if (info.id == selected_id) {
                marker.scale.x = marker.scale.y = marker.scale.z = 0.08;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
            } else {
                marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.color.a = 0.4;
            }
            
            marker.lifetime = rclcpp::Duration(0, 0);//permanente
            marker_array.markers.push_back(marker);
        }
        
        marker_pub_->publish(marker_array);
    }
    
    void visualizeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        if (!planning_scene_) {
            RCLCPP_ERROR(this->get_logger(), " Planning Scene non disponibile!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "🎬 Visualizzazione traiettoria (%zu waypoints)...", 
                   trajectory.points.size());
        
        // Ottieni il robot model
        const moveit::core::RobotModelConstPtr& robot_model = planning_scene_->getRobotModel();
        
        // Ottieni il joint model group
        const moveit::core::JointModelGroup* joint_model_group = 
            robot_model->getJointModelGroup(group_name_);//cerca il gruppo estratto dal primo nome di joint (left o right)
        
        // if (!joint_model_group) {
        //     // Prova con nomi standard
        //     joint_model_group = robot_model->getJointModelGroup(group_name_);
        //     if (!joint_model_group) {
        //         joint_model_group = robot_model->getJointModelGroup(group_name_);
        //     }
        //     if (!joint_model_group) {
        //         joint_model_group = robot_model->getJointModelGroup("manipulator");
        //     }
        // }
        
        if (!joint_model_group) {
            RCLCPP_ERROR(this->get_logger(), " Joint Model Group non trovato!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "   Group: %s", joint_model_group->getName().c_str());
        
        // Pubblica ogni waypoint con un delay
        for (size_t i = 0; i < trajectory.points.size(); ++i) {
            const auto& point = trajectory.points[i];
            
            // Crea robot state
            moveit::core::RobotState robot_state(robot_model);
            robot_state.setToDefaultValues();//applica i valori di default ai joint
            
            // Imposta posizioni dei joint
            robot_state.setJointGroupPositions(joint_model_group, point.positions);//applica le posizioni dei joint al robot state
            robot_state.update();
            
            // Crea messaggio DisplayRobotState
            moveit_msgs::msg::DisplayRobotState display_msg;
            display_msg.state.is_diff = false;
            moveit::core::robotStateToRobotStateMsg(robot_state, display_msg.state);//trasforma il robot state in un messaggio
            
            // Pubblica
            display_state_pub_->publish(display_msg);
            
            // Log ogni 10 waypoints
            if (i % 10 == 0 || i == trajectory.points.size() - 1) {
                RCLCPP_INFO(this->get_logger(), "    Waypoint %zu/%zu", 
                           i + 1, trajectory.points.size());
            }
            
            // Pausa breve per visualizzazione
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        
        RCLCPP_INFO(this->get_logger(), "Visualizzazione completata!\n");
    }
    
    // Membri privati
    std::string yaml_file_;
    std::string group_name_;
    double max_click_distance_;  // Soglia massima per accettare un click
    std::vector<PlanInfo> plans_info_;  // SOLO info leggere
    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr display_state_pub_;
    
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    planning_scene::PlanningScenePtr planning_scene_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ValidateTrajectoryClickNode>();
    
    if (!node->init()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Inizializzazione fallita!");
        return 1;
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}