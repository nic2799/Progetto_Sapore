#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

class MeshSceneNode : public rclcpp::Node
{
public:
    MeshSceneNode() : Node("mesh_scene_node")
    {
        // Inizializzo il PlanningSceneInterface
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Chiamo il metodo per inserire la mesh
        SearchObstacles();
        printCollisionObjectPoses();
    }

private:
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    void SearchObstacles(){
        RCLCPP_INFO(this->get_logger(), "Inserimento mesh nella scena attraverso lo YAML...");
        // Carico il file YAML
         YAML::Node config = YAML::LoadFile("/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/src/build_scene_package/obstacle_descritpion/environment.yaml");

     
         for (const auto& obstacle : config["obstacles"]) {
            std::string mesh_resource = obstacle["cad"].as<std::string>();
            std::string id = obstacle["name"].as<std::string>();
            geometry_msgs::msg::Pose pose;
            // Posizione
            auto position = obstacle["pose"]["position"];
            pose.position.x = position[0].as<double>();
            pose.position.y = position[1].as<double>();
            pose.position.z = position[2].as<double>();

            // Orientamento: rpy -> quaternion
            auto rpy = obstacle["pose"]["rpy"];
            double roll = rpy[0].as<double>();
            double pitch = rpy[1].as<double>();
            double yaw = rpy[2].as<double>();
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            RCLCPP_INFO(this->get_logger(), "Quaternion: x=%f y=%f z=%f w=%f",
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);


            insertMeshObstacle(mesh_resource, pose, id);
        }
        


    }
    void printCollisionObjectPoses()
{
    // Legge tutti gli oggetti collision presenti nella scena
    auto objects = planning_scene_interface_->getObjects();
    if (objects.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Nessun CollisionObject presente nella scena.");
        return;
    }

    for (const auto& pair : objects)
    {
        const auto& id = pair.first;
        const auto& obj = pair.second;

        RCLCPP_INFO(this->get_logger(), "CollisionObject id: %s", id.c_str());

        for (size_t i = 0; i < obj.mesh_poses.size(); ++i)
        {
            const auto& p = obj.mesh_poses[i];
            RCLCPP_INFO(this->get_logger(),
                        "Mesh %zu: position=(%.3f, %.3f, %.3f) orientation=(x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                        i,
                        p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
        }

        for (size_t i = 0; i < obj.primitive_poses.size(); ++i)
        {
            const auto& p = obj.primitive_poses[i];
            RCLCPP_INFO(this->get_logger(),
                        "Primitive %zu: position=(%.3f, %.3f, %.3f) orientation=(x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                        i,
                        p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
        }
    }
}


    void insertMeshObstacle(std::string mesh_resources, geometry_msgs::msg::Pose pose,std::string id)
    {
        // Definisco la pose dell'oggetto
        geometry_msgs::msg::Pose pose_temp;
        pose_temp.orientation = pose.orientation;
        pose_temp.position.x = pose.position.x;
        pose_temp.position.y = pose.position.y;
        pose_temp.position.z = pose.position.z;
        const std::string frame_id = "world";
        std::string mesh_resource = mesh_resources; // Percorso della mesh
       

        // Creo il CollisionObject con la mesh
        moveit_msgs::msg::CollisionObject collision_object = createMeshCollisionObject(
            frame_id,           
            id,                              // frame_id                            // id
            mesh_resource, // percorso mesh
            pose_temp,
            0.001                                           // scala da mm a metri
        );

        // Applico l'oggetto alla scena
        planning_scene_interface_->applyCollisionObject(collision_object);

        RCLCPP_INFO(this->get_logger(), "Mesh inserita nella scena tramite applyCollisionObject()");
    }

    moveit_msgs::msg::CollisionObject createMeshCollisionObject(
        const std::string& frame_id,
        const std::string& id,
        const std::string& mesh_resource,
        const geometry_msgs::msg::Pose& pose,
        double scale = 0.001)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = id;

        // Carico la mesh
        shapes::Mesh* m = shapes::createMeshFromResource(mesh_resource);
        if (!m)
        {
            RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento della mesh: %s", mesh_resource.c_str());
            return collision_object;
        }

        // Scala manuale dei vertici
        for (unsigned int i = 0; i < m->vertex_count; ++i)
        {
            m->vertices[3*i+0] *= scale;
            m->vertices[3*i+1] *= scale;
            m->vertices[3*i+2] *= scale;
        }

        // Conversione in shape_msgs::msg::Mesh
        shape_msgs::msg::Mesh mesh_msg;
        shapes::ShapeMsg mesh_msg_tmp;
        shapes::constructMsgFromShape(m, mesh_msg_tmp);
        mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);

        collision_object.meshes.push_back(mesh_msg);
        collision_object.mesh_poses.push_back(pose);
        //PLOTTIAMO GLI ORIENTAMENTI
            RCLCPP_INFO(this->get_logger(), "Orientamento caricamento mesh: x=%f y=%f z=%f w=%f",
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MeshSceneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
