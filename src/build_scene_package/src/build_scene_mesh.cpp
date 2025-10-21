#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometric_shapes/shape_operations.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class BuildSceneMesh : public rclcpp::Node
{
public:
    BuildSceneMesh() : Node("build_scene_mesh")
    {
        loadObstacleFromYAML();
    }

private:
    void loadObstacleFromYAML()
    {
        // Percorso del file YAML
        std::string yaml_path = "src/build_scene_package/obstacle_description_total/environment.yaml";

        // Carica il file YAML
        YAML::Node config = YAML::LoadFile(yaml_path);//rappresenta la radice del file yaml(config rappresenta tutto il file yaml)
        auto obstacle = config["obstacle"];//accede al nodo figlio con chiave "obstacle" il tipo restituito è YAML::Node

        // Estrai i dati
        std::string name = obstacle["name"].as<std::string>();//convertiamo il nodo YAML in stringa
        std::string mesh_path = obstacle["mesh"].as<std::string>();
        auto position = obstacle["position"].as<std::vector<double>>();//convertiamo il nodo YAML in vettore di double
        auto orientation_rpy = obstacle["orientation_rpy"].as<std::vector<double>>();

        // Converti RPY in quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(orientation_rpy[0], orientation_rpy[1], orientation_rpy[2]);//definiamo un quaternione in base agli angoli RPY reperiti dal file yaml

        // Crea il CollisionObject
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = name;
        collision_object.header.frame_id = "world";

        // Carica la mesh
        shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
        /*La funzione di sopra restituisce un puntatore a un oggetto shaped::Mesh.
        Oggetto mesh contiene vertici e facce*/
        if (!mesh)
        {
            RCLCPP_ERROR(this->get_logger(), "Errore nel caricamento della mesh: %s", mesh_path.c_str());
            return;
        }

        // Riscalatura della mesh (da millimetri a metri)
        const double scale_factor = 0.001; // Conversione da mm a metri
        for (unsigned int i = 0; i < mesh->vertex_count; ++i)
        {
            mesh->vertices[3 * i] *= scale_factor;
            mesh->vertices[3 * i + 1] *= scale_factor;
            mesh->vertices[3 * i + 2] *= scale_factor;
        }

        // Converte la mesh in un messaggio ROS
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(mesh, shape_msg);
        collision_object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));

        // Imposta la posa
        geometry_msgs::msg::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();
        collision_object.mesh_poses.push_back(pose);

        // Aggiungi l'oggetto alla scena
        collision_object.operation = collision_object.ADD;
        planning_scene_interface_.applyCollisionObject(collision_object);

        RCLCPP_INFO(this->get_logger(), "Mesh caricata e aggiunta alla scena: %s", name.c_str());
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BuildSceneMesh>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}