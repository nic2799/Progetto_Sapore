#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>


class BuildSceneNode : public rclcpp::Node
{
public:
    BuildSceneNode() : Node("build_scene_node")
    {
        // Publisher per il planning scene
        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", 10);
        
        // Timer per pubblicare la scena ogni secondo
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&BuildSceneNode::publishScene, this));
        
        // Carica la configurazione degli ostacoli
        loadObstacleConfiguration();
        
        RCLCPP_INFO(this->get_logger(), "Build Scene Node started");
    }
    shape_msgs::msg::Mesh loadMesh(const std::string& resource_path) {
    shapes::Shape* shape = shapes::createMeshFromResource(resource_path);
    if (!shape) {
        throw std::runtime_error("Errore nel caricamento del mesh: " + resource_path);
    }
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg mesh_msg_tmp;
    shapes::constructMsgFromShape(shape, mesh_msg_tmp);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);
    delete shape;
    return mesh_msg;
}


private:
    struct ObstacleInfo {
        std::string name;
        std::string mesh_path;
        geometry_msgs::msg::Pose pose;
    };

    std::vector<ObstacleInfo> obstacles_;

    void loadObstacleConfiguration()
    {
        try {
            // Percorso del file YAML
            std::string package_path = ament_index_cpp::get_package_share_directory("build_scene_package");
            std::string yaml_path = package_path + "/obstacle_descritpion/environment.yaml";
            
            // Carica il file YAML
            YAML::Node config = YAML::LoadFile(yaml_path);
            
            if (!config["obstacles"]) {
                RCLCPP_ERROR(this->get_logger(), "No 'obstacles' section found in YAML file");
                return;
            }

            for (const auto& obstacle_node : config["obstacles"]) {
                ObstacleInfo obstacle;
                
                // Nome dell'ostacolo
                obstacle.name = obstacle_node["name"].as<std::string>();
                
                // Percorso del mesh
                obstacle.mesh_path = obstacle_node["cad"].as<std::string>();
                
                // Posizione
                auto position = obstacle_node["pose"]["position"];
                obstacle.pose.position.x = position[0].as<double>();
                obstacle.pose.position.y = position[1].as<double>();
                obstacle.pose.position.z = position[2].as<double>();
                
                // Orientamento (rpy -> quaternion)
                auto rpy = obstacle_node["pose"]["rpy"];
                double roll = rpy[0].as<double>();
                double pitch = rpy[1].as<double>();
                double yaw = rpy[2].as<double>();
                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);
                obstacle.pose.orientation.x = q.x();
                obstacle.pose.orientation.y = q.y();
                obstacle.pose.orientation.z = q.z();
                obstacle.pose.orientation.w = q.w();
                
                obstacles_.push_back(obstacle);
                
                RCLCPP_INFO(this->get_logger(), "Loaded obstacle: %s at position [%.2f, %.2f, %.2f]", 
                           obstacle.name.c_str(), 
                           obstacle.pose.position.x, 
                           obstacle.pose.position.y, 
                           obstacle.pose.position.z);
            }
            
            RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu obstacles from YAML", obstacles_.size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading obstacle configuration: %s", e.what());
        }
    }

    std::string resolvePackagePath(const std::string& package_path) 
    {
        // Risolve i percorsi del tipo "package://package_name/path"
        if (package_path.substr(0, 10) == "package://") {
            size_t package_end = package_path.find('/', 10);
            std::string package_name = package_path.substr(10, package_end - 10);
            std::string relative_path = package_path.substr(package_end);
            
            try {
                std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
                return package_share_dir + relative_path;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Could not resolve package path: %s", e.what());
                return "";
            }
        }
        return package_path;
    }

    void publishScene()
    {
        // Crea il messaggio planning scene
        auto planning_scene_msg = moveit_msgs::msg::PlanningScene();
        planning_scene_msg.is_diff = true; // Indica che stiamo aggiungendo oggetti
        
        // Crea collision objects per ogni ostacolo caricato dal YAML
      for (const auto& obstacle : obstacles_) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = obstacle.name;

    // Percorso al file STL (usa package:// se il file è nel tuo pacchetto ROS)
    std::string mesh_path;
    if (obstacle.name == "frigorifero") {
        mesh_path = "package://build_scene_package/meshes/Frigorifero.stl";
    } else if (obstacle.name == "Roboqbo") {
        mesh_path = "package://build_scene_package/meshes/Roboqbo.stl";
    } else if (obstacle.name.find("Tavolo") != std::string::npos) {
        mesh_path = "package://build_scene_package/meshes/Tavolo_inox.stl";
    } else if (obstacle.name == "Rastrelliera") {
        mesh_path = "package://build_scene_package/meshes/Rastrelliera.stl";
    } else if (obstacle.name == "Trasportatore1") {
        mesh_path = "package://build_scene_package/meshes/Trasportatore1.stl";
    } else if (obstacle.name == "floor") {
        mesh_path = "package://build_scene_package/meshes/Pavimento1.stl";
    } else {
        mesh_path = "package://build_scene_package/meshes/default_box.stl";
    }

    try {
        shape_msgs::msg::Mesh mesh_msg = loadMesh(mesh_path);
        collision_object.meshes.push_back(mesh_msg);
        collision_object.mesh_poses.push_back(obstacle.pose);
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("collision_loader"), "Errore caricando mesh %s: %s",
                    mesh_path.c_str(), e.what());
        continue;
    }

    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    planning_scene_msg.world.collision_objects.push_back(collision_object);
}

        
        // Pubblica la scena
        planning_scene_publisher_->publish(planning_scene_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published planning scene with %zu objects from YAML configuration", 
                   planning_scene_msg.world.collision_objects.size());
    }

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BuildSceneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}