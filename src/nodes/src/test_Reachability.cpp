#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>

using moveit_msgs::srv::GetPositionIK;
using namespace std::chrono_literals;

class ReachabilityIKNode : public rclcpp::Node
{
public:
    ReachabilityIKNode() : Node("reachability_ik_node")
    {
        // ---- CLIENT PER /compute_ik ----
        client_ = this->create_client<GetPositionIK>("/compute_ik");
        RCLCPP_INFO(this->get_logger(), "Attendo servizio /compute_ik...");
        client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Servizio /compute_ik disponibile!");

        // ---- PUBLISHER PER RVIZ ----
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
    }

    // Metodo pubblico per avviare la generazione della griglia e il test
    void start()
    {
        auto tavolo = generateGrid(-0.25, -0.24, -1.0, -0.2, 0.2, 1.8, 0, 3.14/2, 0.0, 0.1);
        RCLCPP_INFO(this->get_logger(), "Avvio test di raggiungibilità...");
        testReachability(tavolo, "right_arm");
    }

private:
    // --- Genera griglia 3D ---
    std::vector<geometry_msgs::msg::PoseStamped> generateGrid(double xmin, double xmax, double ymin, double ymax,
                                                              double zmin, double zmax, double roll, double pitch, double yaw, double step)
    {
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion ros_quaternion;
        tf2::convert(quaternion, ros_quaternion);

        std::vector<geometry_msgs::msg::PoseStamped> grid_points;
        for (double x = xmin; x <= xmax; x += step)
        {
            for (double y = ymin; y <= ymax; y += step)
            {
                for (double z = zmin; z <= zmax; z += step)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "world";
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = z;
                    pose.pose.orientation = ros_quaternion;
                    grid_points.push_back(pose);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Generati %zu punti nella griglia.", grid_points.size());
        return grid_points;
    }

    // --- Testa ogni punto e controlla IK + collisioni ---
    void testReachability(const std::vector<geometry_msgs::msg::PoseStamped> &grid_points, const std::string &group_name)
    {
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;

        // --- Inizializzo la Planning Scene Monitor ---
       planning_scene_monitor::PlanningSceneMonitorPtr psm =
    std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(this->shared_from_this(), "robot_description");

        if (!psm->getPlanningScene())
        {
            RCLCPP_ERROR(this->get_logger(), "Errore: impossibile creare PlanningSceneMonitor. robot_description mancante?");
            return;
        }

        psm->startSceneMonitor();
                psm->requestPlanningSceneState("get_planning_scene");//chiede esplicitamente l'aggiornamento della scena di
        psm->startWorldGeometryMonitor();
        psm->startStateMonitor();

        // attendo un po’ che arrivi la scena
        rclcpp::sleep_for(500ms);
        auto planning_scene = psm->getPlanningScene();

        for (const auto &pose : grid_points)
        {
            // Prima richiesta IK (con avoid_collisions = true)
            auto req = std::make_shared<GetPositionIK::Request>();
            req->ik_request.group_name = group_name;
            req->ik_request.pose_stamped = pose;
            req->ik_request.ik_link_name = "right_tcp";
            req->ik_request.avoid_collisions = true;
            req->ik_request.timeout.sec = 1;

            auto future = client_->async_send_request(req);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "Timeout per richiesta IK.");
                continue;
            }

            auto response = future.get();
            bool reachable = (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

            RCLCPP_INFO(this->get_logger(), "IK response code = %d", response->error_code.val);

            // Se fallisce, proviamo senza collision avoidance
            sensor_msgs::msg::JointState candidate_js;
            bool have_candidate = false;

            if (!reachable)
            {
                RCLCPP_WARN(this->get_logger(), "IK (avoid_collisions=true) fallita, provo con avoid_collisions=false...");
                auto req2 = std::make_shared<GetPositionIK::Request>(*req);
                req2->ik_request.avoid_collisions = false;

                auto future2 = client_->async_send_request(req2);
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future2) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto resp2 = future2.get();
                    if (resp2->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
                    {
                        candidate_js = resp2->solution.joint_state;
                        have_candidate = !candidate_js.position.empty();
                        RCLCPP_INFO(this->get_logger(), "IK (avoid_collisions=false) SII ESISTE UNA SOLUZIONE IK ANCHE SE È IN COLLISIONE.");
                       
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Anche avoid_collisions=false non ha trovato soluzione.");
                    }
                }
            }

            // Prepara stato robot per collision check
            moveit::core::RobotState current_state = planning_scene->getCurrentState();

            if (have_candidate)
            {
                current_state.setVariablePositions(candidate_js.name, candidate_js.position);
                current_state.update();
            }

            // Configura collision check
            collision_detection::CollisionRequest collision_request;
            collision_request.contacts = true;
            collision_request.max_contacts = 100;
            collision_request.max_contacts_per_pair = 5;
            collision_request.group_name = group_name;
            
            

            collision_detection::CollisionResult collision_result;
            planning_scene->checkCollision(collision_request, collision_result, current_state);

            if (collision_result.collision)
            {
                RCLCPP_WARN(this->get_logger(), "Collisione rilevata!");
                for (const auto &entry : collision_result.contacts)
                {
                    RCLCPP_INFO(this->get_logger(),
                                " - Link '%s' collide con '%s' (%zu contatti)",
                                entry.first.first.c_str(), entry.first.second.c_str(), entry.second.size());
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Nessuna collisione rilevata.");
            }

            // --- Marker per RViz ---
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = pose.header.frame_id;
            marker.header.stamp = this->now();
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
            marker.pose = pose.pose;
            marker.color.a = 1.0;
            marker.color.r = reachable ? 0.0 : 1.0;
            marker.color.g = reachable ? 1.0 : 0.0;
            marker.color.b = 0.0;
            markers.markers.push_back(marker);
            marker_pub_->publish(markers);
        }

        RCLCPP_INFO(this->get_logger(), "Pubblicati %zu marker su RViz.", markers.markers.size());
    }

    // --- Membri ---
    rclcpp::Client<GetPositionIK>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

// --- MAIN ---
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachabilityIKNode>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
