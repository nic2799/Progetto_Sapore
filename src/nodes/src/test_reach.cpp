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
        client_ = this->create_client<GetPositionIK>("/compute_ik");
        RCLCPP_INFO(this->get_logger(), "Attendo servizio /compute_ik...");
        client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Servizio /compute_ik disponibile!");

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        this->declare_parameter("xmin", -1.0);
        this->declare_parameter("xmax", 1.0);
        this->declare_parameter("ymin", -1.0);
        this->declare_parameter("ymax", 1.0);
        this->declare_parameter("zmin", 0.0);
        this->declare_parameter("zmax", 1.0);
        this->declare_parameter("roll", 0.0);
        this->declare_parameter("pitch", 0.0);
        this->declare_parameter("yaw", 0.0);
        this->declare_parameter("step", 0.1);
        this->declare_parameter("group_name", "right_arm");

        // Lettura dei parametri effettivi
        xmin_ = this->get_parameter("xmin").as_double();
        xmax_ = this->get_parameter("xmax").as_double();
        ymin_ = this->get_parameter("ymin").as_double();
        ymax_ = this->get_parameter("ymax").as_double();
        zmin_ = this->get_parameter("zmin").as_double();
        zmax_ = this->get_parameter("zmax").as_double();
        roll_ = this->get_parameter("roll").as_double();
        pitch_ = this->get_parameter("pitch").as_double();
        yaw_ = this->get_parameter("yaw").as_double();
        step_ = this->get_parameter("step").as_double();
        group = this->get_parameter("group_name").as_string();

    }

    void start()
    {
        //auto tavolo = generateGrid(-0.25, -0.24, -1.0, -0.2, 0.2, 1.8, 0, 3.14/2, 0.0, 0.1);
        auto pedana = generateGrid(xmin_,xmax_,ymin_,ymax_,zmin_,zmax_,roll_,pitch_,yaw_ , step_);
        RCLCPP_INFO(this->get_logger(), "Avvio test di raggiungibilità...");
        testReachability(pedana, group + "arm");
    }

private:
    double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_,roll_, pitch_,yaw_,step_;
    std::string group;

    std::vector<geometry_msgs::msg::PoseStamped> generateGrid(double xmin, double xmax, double ymin, double ymax,
                                                              double zmin, double zmax, double roll, double pitch, double yaw, double step)
    {
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion ros_quaternion;
        tf2::convert(quaternion, ros_quaternion);

        double r, p, y;
            tf2::Matrix3x3(quaternion).getRPY(r, p, y);
            RCLCPP_INFO(this->get_logger(), "RPY: roll=%f pitch=%f yaw=%f", r, p, y);


        std::vector<geometry_msgs::msg::PoseStamped> grid_points;
        for (double x = xmin; x <= xmax; x += step)
            for (double y = ymin; y <= ymax; y += step)
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

        RCLCPP_INFO(this->get_logger(), "Generati %zu punti nella griglia.", grid_points.size());
        
        return grid_points;
    }

    void testReachability(const std::vector<geometry_msgs::msg::PoseStamped> &grid_points, const std::string &group_name)
    {
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;

        planning_scene_monitor::PlanningSceneMonitorPtr psm =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(this->shared_from_this(), "robot_description");

        if (!psm->getPlanningScene())
        {
            RCLCPP_ERROR(this->get_logger(), "Errore: impossibile creare PlanningSceneMonitor.");
            return;
        }

        //psm->startSceneMonitor();
        psm->requestPlanningSceneState("get_planning_scene");
        //psm->startWorldGeometryMonitor();
        //psm->startStateMonitor();
        //rclcpp::sleep_for(500ms);

        auto planning_scene = psm->getPlanningScene();

        for (const auto &pose : grid_points)
        {
            auto req = std::make_shared<GetPositionIK::Request>();
            req->ik_request.group_name = group_name;
            req->ik_request.pose_stamped = pose;
            req->ik_request.ik_link_name = group + "tcp";//probabilmente non serve proviamo a toglierlo
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
            RCLCPP_INFO(this->get_logger(), "Per nodo: (%s) IK response code = %d",this->get_name(), response->error_code.val);

            sensor_msgs::msg::JointState candidate_js;
            bool have_candidate = false;

            if (!reachable)
            {
                RCLCPP_INFO(this->get_logger(), "iL PUNTO PUO ESSERE O NON RAGGIUNGIBILE O IN COLLISIONE. PROVO A CERCARE UNA SOLUZIONE SENZA EVITARE LE COLLISIONI.");
                auto req2 = std::make_shared<GetPositionIK::Request>(*req);
                req2->ik_request.avoid_collisions = false;

                auto future2 = client_->async_send_request(req2);
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future2) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto resp2 = future2.get();
                    if (resp2->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
                    {
                        // FILTRO SOLO giunti "right_"
                        for (size_t i = 0; i < resp2->solution.joint_state.name.size(); ++i)
                        {
                            if (resp2->solution.joint_state.name[i].find(group) != std::string::npos)
                            {
                                candidate_js.name.push_back(resp2->solution.joint_state.name[i]);
                       
                                candidate_js.position.push_back(resp2->solution.joint_state.position[i]);
                                
                       
                               // std::ostringstream oss;
                               // for (size_t j = 0; j < candidate_js.name.size(); ++j)
                                 //   oss << candidate_js.name[j] << "=" << candidate_js.position[j] << " ";
                                //RCLCPP_INFO_STREAM(this->get_logger(), "GLI JOINT_STATES: " << oss.str());

                            }
                        }
                        have_candidate = !candidate_js.position.empty();
                        RCLCPP_INFO(this->get_logger(), "AVOID_COLLISIONS=FALSE: IL PUNTO È RAGGIUNGIBILE, MA IN COLLISIONE.");
                        RCLCPP_INFO(this->get_logger(), "LA POSIZIONE DELL'END EFFECTOR CHE È in collisione è: x=%f y=%f z=%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                        RCLCPP_INFO(this->get_logger(), "orientamento in collisione è: ox=%f oy=%f oz=%f ow=%f",
                                     pose.pose.orientation.x, pose.pose.orientation.y,
                                     pose.pose.orientation.z, pose.pose.orientation.w);
                                    double r, p, y;
                                    // convertiamo geometry_msgs::msg::Quaternion -> tf2::Quaternion
                                    tf2::Quaternion q;
                                    tf2::fromMsg(pose.pose.orientation, q);
                                    tf2::Matrix3x3(q).getRPY(r, p, y);
                                    RCLCPP_INFO(this->get_logger(), "RPY: roll=%f pitch=%f yaw=%f", r, p, y);
                    }
                }
            }

            moveit::core::RobotState current_state = planning_scene->getCurrentState();

            if (have_candidate)//SE TROVO POSA RAGGIUNGIBILE IN COLLISIONE
            {
                current_state.setVariablePositions(candidate_js.name, candidate_js.position);
                current_state.update();
            

            collision_detection::CollisionRequest collision_request;
            collision_request.contacts = true;
            collision_request.max_contacts = 100;
            collision_request.max_contacts_per_pair = 5;
            collision_request.group_name = group_name;

            collision_detection::CollisionResult collision_result;
            planning_scene->checkCollision(collision_request, collision_result, current_state);

            
                RCLCPP_WARN(this->get_logger(), "LA COLLISIONE È TRA : ");
                for (const auto &entry : collision_result.contacts)
                    RCLCPP_INFO(this->get_logger(), " - Link '%s' collide con '%s' (%zu contatti)",
                                entry.first.first.c_str(), entry.first.second.c_str(), entry.second.size());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Nessuna collisione rilevata.");
            }

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
            marker.color.b = have_candidate ? 1.0 : 0.0;
            //marker.color.b = 0.0;
            marker.ns = this->get_name();

            markers.markers.push_back(marker);
            marker_pub_->publish(markers);
        }

        RCLCPP_INFO(this->get_logger(), "Pubblicati %zu marker su RViz.", markers.markers.size());
    }

    rclcpp::Client<GetPositionIK>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachabilityIKNode>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
