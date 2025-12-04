#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <SaveYaml.hpp>
#include <MoveitHelper.hpp>
#include <tf2/LinearMath/Quaternion.h>

class PickPlaceServer : public rclcpp::Node {
public:
    PickPlaceServer() : Node("pick_place_server") {
        this->declare_parameter("planning_group", "left_arm");
        this->declare_parameter("plans_output_file", "PickPouring.yaml");
        this->declare_parameter("hand", "right_hand");

        marker_feedback_sub_ = this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
            "/reachability_marker/feedback", 10,
            std::bind(&PickPlaceServer::markerFeedbackCallback, this, std::placeholders::_1));
        trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/trigger_pick_place",
            std::bind(&PickPlaceServer::triggerCallback, this, std::placeholders::_1, std::placeholders::_2));
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("place_debug_marker", 10);
        constraints_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("constraints_visualization", 10);

        RCLCPP_INFO(this->get_logger(), "PickandPlace Server avviato");
    }

    void init() {     
        planning_group_name_ = this->get_parameter("planning_group").as_string();
        plans_output_file_ = this->get_parameter("plans_output_file").as_string();
        hand = this->get_parameter("hand").as_string();
        base_frame_ = "world"; // Fisso a "world" 

        moveit_helper_ = std::make_shared<moveit_utils::MoveItHelper>(shared_from_this(), planning_group_name_);
        hand_helper_ = std::make_shared<moveit_utils::MoveItHelper>(shared_from_this(), hand);
        hand_helper_->init();
        moveit_helper_->init();
        
        RCLCPP_INFO(this->get_logger(), "🔧 MoveIt Helper inizializzato su frame: %s", base_frame_.c_str());
    }

private:
    int plan_counter_ = 6;
    std::string plans_output_file_;
    std::string hand;
    std::string base_frame_; 
    std::shared_ptr<moveit_utils::MoveItHelper> moveit_helper_;
    std::shared_ptr<moveit_utils::MoveItHelper> hand_helper_;
    std::string planning_group_name_;
    rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr marker_feedback_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr constraints_marker_pub_;
    geometry_msgs::msg::Pose latest_target_pose_;
    bool has_target_pose_ = false;

    void markerFeedbackCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg) {
        if (msg->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP ||
            msg->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
            latest_target_pose_ = msg->pose;
            has_target_pose_ = true;
        }
    }

    void triggerCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (!has_target_pose_) {
            response->success = false;
            response->message = " Nessuna target pose disponibile.";
            return;
        }

        std::string object_id = moveit_helper_->spawnCollisionObject(latest_target_pose_, 0.12, 0.02);
        if (object_id.empty()) { response->success = false; return; }

        bool success = executePickAndPlace(object_id, latest_target_pose_);
        moveit_helper_->removeCollisionObject(object_id);

        response->success = success;
        response->message = success ? "Pick&Place completato!" : " Fallito.";
    }

    bool executePickAndPlace(const std::string& object_id, const geometry_msgs::msg::Pose& object_pose) {
        auto orientation_constraints = moveit_helper_->createNoRotationYZConstraint();
        moveit_helper_->setStartStateToCurrentState();
        moveit_helper_->clearPoseTargets();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success;

        // --- 1. APPROACH ---
        geometry_msgs::msg::Pose pre_approach_pose = object_pose;
        pre_approach_pose.position.x -= 0.15;
        success = (moveit_helper_->planWithFallback(pre_approach_pose, plan, "approach") == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) return false;
        moveit_helper_->executePlan(plan);
        
        geometry_msgs::msg::PoseStamped log_pose;
        log_pose.header.frame_id = base_frame_; // Uso frame corretto
        log_pose.header.stamp = this->now();
        log_pose.pose = pre_approach_pose;
        SaveYaml::savePlanToYAML(plan, log_pose, "approach", plan_counter_, plans_output_file_, 0.0);

        // --- 2. GRASP (Cartesian) ---
        geometry_msgs::msg::Pose grasp_pose = object_pose;
        grasp_pose.position.x += 0.06;
        grasp_pose.orientation = pre_approach_pose.orientation;
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = moveit_helper_->computeCartesianPath({grasp_pose}, trajectory);
        if (fraction < 0.20) return false;
        
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory = trajectory;
        moveit_helper_->executePlan(cartesian_plan);

        log_pose.pose = grasp_pose;
        SaveYaml::savePlanToYAML(cartesian_plan, log_pose, "progress", plan_counter_, plans_output_file_, 0.0);

        moveit_helper_->setPathConstraints(orientation_constraints);
        
        // Chiudi mano
        moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
        if (hand_helper_->planToNamedTarget("chiusa", hand_plan)) {
            hand_helper_->executePlan(hand_plan);
        }
        moveit_helper_->attachObject(object_id);

        // --- 3. RETRACT ---
        geometry_msgs::msg::Pose retract_pose = grasp_pose;
        retract_pose.position.x -= 0.15;
        fraction = moveit_helper_->computeCartesianPath({retract_pose}, trajectory);
        if (fraction < 0.20) { moveit_helper_->detachObject(object_id); return false; }
        
        cartesian_plan.trajectory = trajectory;
        moveit_helper_->executePlan(cartesian_plan);
        
        log_pose.pose = retract_pose;
        SaveYaml::savePlanToYAML(cartesian_plan, log_pose, "retract", plan_counter_, plans_output_file_, 1.0);

        // --- 4. LIFT ---
        geometry_msgs::msg::Pose lift_pose = retract_pose;
        lift_pose.position.z = 0.90;
        lift_pose.position.x = -1.50;
        lift_pose.position.y = -0.40;
        tf2::Quaternion quat_tf2;
        quat_tf2.setRPY(M_PI/2, M_PI/2, -M_PI/2);
        lift_pose.orientation.x = quat_tf2.x(); lift_pose.orientation.y = quat_tf2.y();
        lift_pose.orientation.z = quat_tf2.z(); lift_pose.orientation.w = quat_tf2.w();

        moveit_helper_->setStartStateToCurrentState();
        moveit_helper_->clearPoseTargets();
        success = (moveit_helper_->planWithFallback(lift_pose, plan, "lift") == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) { moveit_helper_->detachObject(object_id); moveit_helper_->clearPathConstraints(); return false; }
        moveit_helper_->executePlan(plan);

        log_pose.pose = lift_pose;
        SaveYaml::savePlanToYAML(plan, log_pose, "lift", plan_counter_, plans_output_file_, 1.0);

        // --- 5. POURING ---
        const auto &joint_names = plan.trajectory.joint_trajectory.joint_names;
        const auto &last_point = plan.trajectory.joint_trajectory.points.back();
        moveit_helper_->setStartStateFromJointPositions(joint_names, last_point.positions);
        moveit_helper_->clearPathConstraints();
        
        moveit::planning_interface::MoveGroupInterface::Plan pouring_plan;
        auto pouring_result = moveit_helper_->planPouringRotation(120.0 * M_PI / 180.0, pouring_plan, last_point.positions);
        if (pouring_result != moveit::core::MoveItErrorCode::SUCCESS) { moveit_helper_->detachObject(object_id); return false; }
        moveit_helper_->executePlan(pouring_plan);

        log_pose.pose = lift_pose; // La posa cartesiana non cambia molto, ma l'orientamento sì
        SaveYaml::savePlanToYAML(pouring_plan, log_pose, "pouring", plan_counter_, plans_output_file_, 1.0);

        // =========================================================
        //                 FASE DI PLACE 
        // =========================================================
        geometry_msgs::msg::Pose PlacePose;
        tf2::Quaternion quat_tf2_place;

        if(planning_group_name_ == "right_arm"){
            PlacePose.position.x = -0.95;
            PlacePose.position.y = -1.90;
            PlacePose.position.z = 1.14;
            quat_tf2_place.setRPY(M_PI/2, M_PI/2, 0);
        } else {
            // LEFT ARM
            PlacePose.position.x = -0.15;
            PlacePose.position.y = 0.70; 
            PlacePose.position.z = 1.10;
            
            quat_tf2_place.setRPY(0, M_PI/2, 0); 
        }

        PlacePose.orientation.x = quat_tf2_place.x();
        PlacePose.orientation.y = quat_tf2_place.y();
        PlacePose.orientation.z = quat_tf2_place.z();
        PlacePose.orientation.w = quat_tf2_place.w();

        // --- DEBUG: PUBBLICAZIONE MARKER ---
        publishPlaceMarker(PlacePose);
        RCLCPP_INFO(this->get_logger(), " Target Place Pose pubblicata: [%.2f, %.2f, %.2f]", 
            PlacePose.position.x, PlacePose.position.y, PlacePose.position.z);

        // Pianificazione Place
        moveit_helper_->setStartStateToCurrentState();
        moveit_helper_->clearPoseTargets();
        
        moveit::planning_interface::MoveGroupInterface::Plan place_plan;
        
        success = (moveit_helper_->planWithFallback(PlacePose, place_plan, "place") == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Fallita pianificazione Place. Controlla il Marker rosso in RViz!");
            return false;
        }

        success = moveit_helper_->executePlan(place_plan);
        if (!success) return false;

        log_pose.pose = PlacePose;
        SaveYaml::savePlanToYAML(place_plan, log_pose, "place", plan_counter_, plans_output_file_, 0.0);

        RCLCPP_INFO(this->get_logger(), " Pick&Place completato!");
        return true;
    }

    void publishPlaceMarker(const geometry_msgs::msg::Pose& pose) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_; 
        marker.header.stamp = this->now();
        marker.ns = "debug_place_pose";
        marker.id = 999;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 0.1; 
        marker.scale.y = 0.1; 
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // 
        marker.color.r = 1.0; // ROSSO per visibilità
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_publisher_->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceServer>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}