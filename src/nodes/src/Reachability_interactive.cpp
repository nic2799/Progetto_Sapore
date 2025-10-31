#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <sensor_msgs/msg/joint_state.hpp> // se non già incluso


using moveit_msgs::srv::GetPositionIK;

class ReachabilityIKNode : public rclcpp::Node
{
public:
    ReachabilityIKNode() : Node("reachability_ik_node") 
    {
        client_ = this->create_client<GetPositionIK>("/compute_ik");
        RCLCPP_INFO(this->get_logger(), "Attendo servizio /compute_ik...");
        client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Servizio /compute_ik disponibile!");
        traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/right_arm_controller/follow_joint_trajectory");
            if (!traj_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Azione FollowJointTrajectory non disponibile!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Azione FollowJointTrajectory disponibile!");
        }

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        marker_sub_ = this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
            "/reachability_marker/feedback", 10,
            std::bind(&ReachabilityIKNode::markerCallback, this, std::placeholders::_1));

        trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "start_reachability_test",
            std::bind(&ReachabilityIKNode::triggerCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        this->declare_parameter("group_name", "right_arm");
        group_ = this->get_parameter("group_name").as_string();
    }

    void init()
    {
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            this->shared_from_this(), "robot_description");

        if (!planning_scene_monitor_->getPlanningScene())
            throw std::runtime_error("Impossibile inizializzare il PlanningSceneMonitor");

        planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
        RCLCPP_INFO(this->get_logger(), "PlanningSceneMonitor inizializzato correttamente.");
    }

    void spinLoop()
    {
        rclcpp::Rate rate(10); // 10 Hz loop
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());

            if (started_ && has_pose_)
            {
                testReachability(last_pose_, group_);
                started_ = false; // reset per evitare loop continui
            }

            rate.sleep();
        }
    }
    void ControllerTrajectory(sensor_msgs::msg::JointState &solution)
    {
        // Implementazione della funzione per il controller della traiettoria
        RCLCPP_INFO(this->get_logger(), "Invio traiettoria al controller...");
        trajectory_msgs::msg::JointTrajectory traj;
        auto planning_scene = planning_scene_monitor_->getPlanningScene();
        auto current_state = planning_scene->getCurrentState();
        std::vector<std::string> joint_names =  {"right_joint_a1","right_joint_a2","right_joint_a3","right_joint_a4","right_joint_a5","right_joint_a6","right_joint_a7"};
        traj.joint_names = joint_names;
        RCLCPP_INFO(this->get_logger(), "Joint da muovere:");
        for (const auto &name : joint_names)
        {
            RCLCPP_INFO(this->get_logger(), " - %s", name.c_str());
        }
        for(const auto &pos : solution.position)
        {
            RCLCPP_INFO(this->get_logger(), " - Posizione: %f", pos);
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;
        // Aggiungi punti alla traiettoria
        point.positions = solution.position;
        point.time_from_start = rclcpp::Duration::from_seconds(5.0);
        traj.points.push_back(point);   
        // Costruisci il goal per l'azione
        control_msgs::action::FollowJointTrajectory::Goal goal;
        goal.trajectory = traj;
        // Invia il goal al server d'azione
        auto send_goal_options =
            rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        auto future = traj_client_->async_send_goal(goal, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Traiettoria inviata al controller.");

        


    }

private:
    rclcpp::Client<GetPositionIK>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr marker_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_client_;
    bool started_ = false;

    geometry_msgs::msg::PoseStamped last_pose_;
    bool has_pose_ = false;
    std::string group_;

   void markerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg)
{
    last_pose_.pose = msg->pose;       // solo il campo pose
    last_pose_.header = msg->header;   // header separato
    has_pose_ = true;
    RCLCPP_INFO(this->get_logger(), "Nuova posa salvata da interactive marker.");
}


    void triggerCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (!has_pose_)
        {
            response->success = false;
            response->message = "Nessuna posa disponibile.";
            return;
        }

        started_ = true;
        response->success = true;
        response->message = "Test di raggiungibilità avviato.";
    }

    void testReachability(const geometry_msgs::msg::PoseStamped &pose, const std::string &group_name)
    {
        auto planning_scene = planning_scene_monitor_->getPlanningScene();
        bool reachable = false;
        bool collision = false;

        // --- IK con collision avoidance = true ---
        auto req = std::make_shared<GetPositionIK::Request>();
        req->ik_request.group_name = group_name;
        req->ik_request.pose_stamped = pose;
        req->ik_request.ik_link_name = "right_tcp";
        req->ik_request.avoid_collisions = true;
        req->ik_request.timeout.sec = 1;

        auto future = client_->async_send_request(req);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        reachable = (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

        if (!reachable)
        {   RCLCPP_WARN(this->get_logger(), "Posa non raggiungibile con avoidance ON.");
            // --- IK con collision avoidance = false ---
            auto req_no_coll = std::make_shared<GetPositionIK::Request>();
            req_no_coll->ik_request.group_name = group_name;
            req_no_coll->ik_request.pose_stamped = pose;
            req_no_coll->ik_request.ik_link_name = "right_tcp";
            req_no_coll->ik_request.avoid_collisions = false;
            req_no_coll->ik_request.timeout.sec = 1;

            auto future_no_coll = client_->async_send_request(req_no_coll);
            rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_no_coll);
            auto response_no_coll = future_no_coll.get();

            if (response_no_coll->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                reachable = true;
                collision = true;

                // Aggiorna robot_state per check collision
                moveit::core::RobotState robot_state = planning_scene->getCurrentState();
                sensor_msgs::msg::JointState candidate_js;
                for (size_t i = 0; i < response_no_coll->solution.joint_state.name.size(); ++i)
                {
                    if (response_no_coll->solution.joint_state.name[i].find("right_") != std::string::npos)
                    {
                        candidate_js.name.push_back(response_no_coll->solution.joint_state.name[i]);
                        candidate_js.position.push_back(response_no_coll->solution.joint_state.position[i]);
                        //plottiamo i giunti
                        RCLCPP_INFO(this->get_logger(), " - Giunto '%s' a posizione %3f",
                            response_no_coll->solution.joint_state.name[i].c_str(),
                            response_no_coll->solution.joint_state.position[i]);
                    }
                }
                robot_state.setVariablePositions(candidate_js.name, candidate_js.position);
                robot_state.update();

                collision_detection::CollisionRequest collision_request;
                collision_detection::CollisionResult collision_result;
                collision_request.contacts = true;
                collision_request.max_contacts = 100;
                collision_request.max_contacts_per_pair = 5;
                planning_scene->checkCollision(collision_request, collision_result,robot_state);
                RCLCPP_INFO(this->get_logger(), "Controllo collisioni eseguito.");
                if (collision_result.collision){
                    RCLCPP_WARN(this->get_logger(), "LA COLLISIONE È TRA : ");
                    sensor_msgs::msg::JointState filtered_joint_state;
                    const auto &group_joints = planning_scene->getRobotModel()->getJointModelGroup(group_name)->getVariableNames();

                    for (size_t i = 0; i < response_no_coll->solution.joint_state.name.size(); ++i)
                    {
                        const auto &joint_name = response_no_coll->solution.joint_state.name[i];
                        if (std::find(group_joints.begin(), group_joints.end(), joint_name) != group_joints.end())
                        {
                            filtered_joint_state.name.push_back(joint_name);
                            filtered_joint_state.position.push_back(response_no_coll->solution.joint_state.position[i]);
                        }
                    }
                    ControllerTrajectory(filtered_joint_state);
                    for (const auto &entry : collision_result.contacts){
                        RCLCPP_INFO(this->get_logger(), " - Link '%s' collide con '%s' (%zu contatti)",
                                    entry.first.first.c_str(), entry.first.second.c_str(), entry.second.size());    }

                
                   
            }
        }

       
      

    }
     // --- Pubblica marker ---
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker marker;
        marker.header = pose.header;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
        marker.pose = pose.pose;
        marker.color.a = 1.0;
        marker.color.r = reachable ? 0.0 : 1.0;
        marker.color.g = reachable ? 1.0 : 0.0;
        marker.color.b = collision ? 1.0 : 0.0;
        markers.markers.push_back(marker);
        marker_pub_->publish(markers);

        RCLCPP_INFO(this->get_logger(), "Test completato: reachable=%d, collision=%d", reachable, collision);
        //plottiamo i link in collisione
}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachabilityIKNode>();
    node->init();

    // Loop principale che controlla started_
    node->spinLoop();

    rclcpp::shutdown();
    return 0;
}
