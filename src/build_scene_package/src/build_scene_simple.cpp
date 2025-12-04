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
#include <ament_index_cpp/get_package_share_directory.hpp>

class MeshSceneNode : public rclcpp::Node
{
public:
    MeshSceneNode() : Node("mesh_scene_node")
    {
        // Inizializzo il PlanningSceneInterface
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        /*Classe moveit che fornisce un'interfaccia per interagire con la scena di pianificazione in ROS. scena di pianificazione: è una rappresentazione dello stato del mondo che include robot*/
        // Chiamo il metodo per inserire la mesh
        SearchObstacles();
    }

private:
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    void SearchObstacles(){
        RCLCPP_INFO(this->get_logger(), "Inserimento mesh nella scena attraverso lo YAML...");
        // Carico il file YAML
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("build_scene_package");
    std::string yaml_path = package_share_dir + "/obstacle_descritpion/environment.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);  
     
         for (const auto& obstacle : config["obstacles"]) {//iteriamo sugli ostacoli definiti nel file yaml
            std::string mesh_resource = obstacle["cad"].as<std::string>();//percorso della mesh
            std::string id = obstacle["name"].as<std::string>();//Nome identificativo dell'ostacolo
            geometry_msgs::msg::Pose pose;//inizializziamo la pose
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
            //Effettuiamo la conversione rpy -> quaternion
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            RCLCPP_INFO(this->get_logger(), "Quaternion: x=%f y=%f z=%f w=%f",
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);


            insertMeshObstacle(mesh_resource, pose, id);//metodo che inserisce la mesh nella scena
        }
        


    }
    


    void insertMeshObstacle(std::string mesh_resources, geometry_msgs::msg::Pose pose,std::string id)
    {
        // Definisco la pose dell'oggetto
        geometry_msgs::msg::Pose pose_temp;
        pose_temp.orientation = pose.orientation;//quaternion
        pose_temp.position.x = pose.position.x;
        pose_temp.position.y = pose.position.y;
        pose_temp.position.z = pose.position.z;
        const std::string frame_id = "world";//riferimento con cui viene inserito l'oggetto
        std::string mesh_resource = mesh_resources; // Percorso della mesh
       

        // Creo il CollisionObject con la mesh
        moveit_msgs::msg::CollisionObject collision_object = createMeshCollisionObject(
            frame_id,           
            id,                            
            mesh_resource, 
            pose_temp,
            0.001// scala da mm a metri, poiche file STL è in mm 
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
/*Carica mesh e la converte in un oggetto di tipo shapes::Mesh utilizzata per moveit
questa rappresentazione contiene vertici e facce della mesh, che possono essere usati per
calcolo della collisione*/
        shapes::Mesh* m = shapes::createMeshFromResource(mesh_resource);//puntatore a oggetto Shapes::mesh->(contiene: vertici e facce della mesh)
        if (!m)
        {
            RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento della mesh: %s", mesh_resource.c_str());
            return collision_object;
        }

        // Scala manuale dei vertici
    //    if(id!="Roboqbo"){
        for (unsigned int i = 0; i < m->vertex_count; ++i)
        {
            m->vertices[3*i+0] *= scale;//struttura è m->vertices = {x1, y1, z1, x2, y2, z2, x3, y3, z3};  
            m->vertices[3*i+1] *= scale;//dunque si accede alle coordinate x,y,z di ogni vertice sommando 0 o 1 o 2 a 3*i(che serve per spostarsi tra i vertici)
            m->vertices[3*i+2] *= scale;//infine moltiplichiamo per la scala
        }

        // Conversione in shape_msgs::msg::Mesh
        shape_msgs::msg::Mesh mesh_msg;
        shapes::ShapeMsg mesh_msg_tmp;//serve come contenitore temporaneo per convertire shapes::Mesh in shape_msgs::msg::Mesh
        shapes::constructMsgFromShape(m, mesh_msg_tmp);
        mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);

        collision_object.meshes.push_back(mesh_msg);
        collision_object.mesh_poses.push_back(pose);//assegno posa dell'ostacolo
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
