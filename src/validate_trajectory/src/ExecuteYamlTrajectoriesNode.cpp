#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <atomic> 
#include <map>
#include <thread>
#include <chrono>

struct TrajectoryInfo {
    std::string name;
    trajectory_msgs::msg::JointTrajectory trajectory;
    double gripper_value = -1.0;
};

struct PlanInfo {
    int id;
    geometry_msgs::msg::PoseStamped target_pose;
    std::vector<TrajectoryInfo> trajectories;
};

class ExecuteYamlTrajectoriesNode : public rclcpp::Node {
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GripperCommand = control_msgs::action::GripperCommand; 
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

public:
    ExecuteYamlTrajectoriesNode() : Node("execute_yaml_trajectories_node") {
        // Callback group per permettere esecuzione parallela di azioni e timer
        
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);

        // --- Inizializzazione Client con Callback Group ---
        left_arm_controller_topic_ = "/left_arm_controller/follow_joint_trajectory";
        left_hand_controller_topic_ = "/left_hand_controller/gripper_cmd";
        
        left_arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, left_arm_controller_topic_, callback_group_);
        left_hand_action_client_ = rclcpp_action::create_client<GripperCommand>(
            this, left_hand_controller_topic_, callback_group_);

        right_arm_controller_topic_ = "/right_arm_controller/follow_joint_trajectory";
        right_hand_controller_topic_ = "/right_hand_controller/gripper_cmd";
        
        right_arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, right_arm_controller_topic_, callback_group_);
        right_hand_action_client_ = rclcpp_action::create_client<GripperCommand>(
            this, right_hand_controller_topic_, callback_group_);

        // --- ATTESA DEI SERVER ---
        RCLCPP_INFO(this->get_logger(), " Attesa connessione con i controller...");
        
        if (!left_arm_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server Braccio SINISTRO non trovato!");
        }
        if (!right_arm_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server Braccio DESTRO non trovato!");
        }
        // I gripper sono opzionali, ma proviamo ad aspettarli
        left_hand_action_client_->wait_for_action_server(std::chrono::seconds(2));
        right_hand_action_client_->wait_for_action_server(std::chrono::seconds(2));

        RCLCPP_INFO(this->get_logger(), "Controller connessi.");

        // Carica YAML
        bool left_ok = loadTrajectoriesFromYAML("PickPouringLeft.yaml", left_plans_);
        bool right_ok = loadTrajectoriesFromYAML("PickPouringRight.yaml", right_plans_);
        
        if (!left_ok && !right_ok) {
            RCLCPP_ERROR(this->get_logger(), "Fallimento caricamento YAML.");
            return;
        }

        publishTrajectoryMarkers();

        // Usiamo un timer one-shot per avviare l'esecuzione DOPO che il costruttore è finito
        // Questo assicura che l'Executor sia già attivo.
        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ExecuteYamlTrajectoriesNode::startWorkflow, this),
            callback_group_
        );
    }

private:
    std::string left_arm_controller_topic_, left_hand_controller_topic_;
    std::string right_arm_controller_topic_, right_hand_controller_topic_;
    std::vector<PlanInfo> left_plans_, right_plans_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_arm_action_client_, right_arm_action_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr left_hand_action_client_, right_hand_action_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Gruppo callback per multithreading
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr start_timer_;

    // Variabile "Semaforo" condivisa. 
    std::atomic<bool> mixing_bowl_busy_{false}; 
    std::map<std::string, rclcpp::TimerBase::SharedPtr> wait_timers_; 

    void startWorkflow() {
        // Cancella il timer di avvio per non rieseguirlo
        start_timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), " AVVIO PARALLELO: Esecuzione Left e Right!");

        // Lancia il thread logico sinistro (ID 4 come esempio)
        std::thread([this](){
            executePlanById(left_plans_, left_arm_action_client_, left_hand_action_client_, 5, "LEFT", [](){});
        }).detach();
        
        std::thread([this](){
            executePlanById(right_plans_, right_arm_action_client_, right_hand_action_client_, 5, "RIGHT", [](){});
        }).detach();
    }

    void executePlanById(std::vector<PlanInfo>& plans,
                         rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client,
                         rclcpp_action::Client<GripperCommand>::SharedPtr hand_client,
                         int id,
                         std::string side,
                         std::function<void()> callback_on_finish) {
        
        auto it = std::find_if(plans.begin(), plans.end(), [id](const PlanInfo& plan) { return plan.id == id; });
        if (it == plans.end()) {
            RCLCPP_ERROR(this->get_logger(), "[%s] Piano ID %d non trovato!", side.c_str(), id);
            return;
        }
        
        sendPlanTrajectoriesRecursive(*it, arm_client, hand_client, 0, side, callback_on_finish);
    }

    void sendPlanTrajectoriesRecursive(const PlanInfo& plan,
                                       rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client,
                                       rclcpp_action::Client<GripperCommand>::SharedPtr hand_client,
                                       size_t idx,
                                       std::string side,
                                       std::function<void()> callback_on_finish) {
        
        if (idx >= plan.trajectories.size()) {
            RCLCPP_INFO(this->get_logger(), "[%s]  Piano completato.", side.c_str());
            if (callback_on_finish) callback_on_finish();
            return;
        }

        const auto& traj_info = plan.trajectories[idx];

        // --- GESTIONE SINCRONIZZAZIONE (MUTEX) ---
        bool is_critical_action = (traj_info.name.find("lift") != std::string::npos);

        if (is_critical_action) {
            if (mixing_bowl_busy_) {
                RCLCPP_WARN(this->get_logger(), "[%s]  Zona Pouring occupata! Attendo...", side.c_str());
                
                // Timer per riprovare
                wait_timers_[side] = this->create_wall_timer(
                    std::chrono::milliseconds(500), 
                    [this, plan, arm_client, hand_client, idx, side, callback_on_finish]() {
                        this->wait_timers_[side]->cancel(); 
                        this->sendPlanTrajectoriesRecursive(plan, arm_client, hand_client, idx, side, callback_on_finish);
                    },
                    callback_group_ // Importante: usa il gruppo Reentrant
                );
                return; 
            } else {
                RCLCPP_INFO(this->get_logger(), "[%s] Inizio Pouring (BLOCCO Risorsa).", side.c_str());
                mixing_bowl_busy_ = true;
            }
        }

        moveGripperAndThenArm(traj_info, plan, arm_client, hand_client, idx, side,
            [this, &plan, arm_client, hand_client, idx, side, callback_on_finish]() {
                sendPlanTrajectoriesRecursive(plan, arm_client, hand_client, idx + 1, side, callback_on_finish);
            }
        );
    }

    void moveGripperAndThenArm(const TrajectoryInfo& traj_info,
                               const PlanInfo& plan,
                               rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client,
                               rclcpp_action::Client<GripperCommand>::SharedPtr hand_client,
                               size_t idx,
                               const std::string& side,
                               std::function<void()> on_arm_finish) {
        
        if (traj_info.gripper_value >= 0 && hand_client->action_server_is_ready()) {
             GripperCommand::Goal goal;
             goal.command.position = (traj_info.gripper_value > 0.1) ? 0.02 : 0.0; 
             goal.command.max_effort = 100.0;
             
             auto opts = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
             opts.result_callback = [this, traj_info, plan, arm_client, hand_client, idx, side, on_arm_finish](auto) {
                 // Indipendentemente dal risultato del gripper, muoviamo il braccio
                 moveArm(traj_info, plan, arm_client, hand_client, idx, side, on_arm_finish);
             };
             hand_client->async_send_goal(goal, opts);
        } else {
            moveArm(traj_info, plan, arm_client, hand_client, idx, side, on_arm_finish);
        }
    }

    void moveArm(const TrajectoryInfo& traj_info,
                 const PlanInfo& plan,
                 rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client,
                 rclcpp_action::Client<GripperCommand>::SharedPtr hand_client,
                 size_t idx,
                 const std::string& side,
                 std::function<void()> on_complete) {
        
        FollowJointTrajectory::Goal goal;
        goal.trajectory = traj_info.trajectory;
        
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        send_goal_options.result_callback = [this, traj_info, plan, arm_client, hand_client, idx, side, on_complete](const GoalHandleFollowJointTrajectory::WrappedResult &result) {
            
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "[%s]  Movimento '%s' finito.", side.c_str(), traj_info.name.c_str());
                
                // Se abbiamo finito di versare, sblocca
                if (traj_info.name.find("lift") != std::string::npos) {
                    RCLCPP_INFO(this->get_logger(), "[%s] Fine Pouring (SBLOCCO).", side.c_str());
                    mixing_bowl_busy_ = false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "[%s]  Movimento fallito code: %d", side.c_str(), (int)result.code);
                // Sblocca in caso di errore 
                if (traj_info.name.find("lift") != std::string::npos) mixing_bowl_busy_ = false;
            }

            if (on_complete) on_complete();//Quando concludiamo il movimento, chiamiamo il callback facendo partire il prossimo idx+1 esempio: pick->pouring->lift->...
        };
        
        // Controlla server prima di inviare
        if (!arm_client->action_server_is_ready()) {
             RCLCPP_ERROR(this->get_logger(), "[%s] Server braccio caduto! Impossibile inviare.", side.c_str());
             return;
        }

        arm_client->async_send_goal(goal, send_goal_options);
    }
    
    // --- Funzione caricamento YAML ---
    bool loadTrajectoriesFromYAML(const std::string& yaml_file, std::vector<PlanInfo>& plans) {
        try {
            YAML::Node yaml = YAML::LoadFile(yaml_file);
            if (!yaml["plans"]) return false;

            for (const auto& plan : yaml["plans"]) {
                PlanInfo plan_info;
                plan_info.id = plan["id"].as<int>();

                if (plan["target_pose"]) {
                    auto p = plan["target_pose"];
                    plan_info.target_pose.header.frame_id = "world";
                    plan_info.target_pose.pose.position.x = p["position"]["x"].as<double>();
                    plan_info.target_pose.pose.position.y = p["position"]["y"].as<double>();
                    plan_info.target_pose.pose.position.z = p["position"]["z"].as<double>();
                    plan_info.target_pose.pose.orientation.x = p["orientation"]["x"].as<double>();
                    plan_info.target_pose.pose.orientation.y = p["orientation"]["y"].as<double>();
                    plan_info.target_pose.pose.orientation.z = p["orientation"]["z"].as<double>();
                    plan_info.target_pose.pose.orientation.w = p["orientation"]["w"].as<double>();
                }

                if (plan["trajectories"]) {
                    for (const auto& traj : plan["trajectories"]) {
                        TrajectoryInfo traj_info;
                        traj_info.name = traj["name"].as<std::string>();
                        
                        if(traj["joint_names"]) {
                            for (const auto& jn : traj["joint_names"]) 
                                traj_info.trajectory.joint_names.push_back(jn.as<std::string>());
                        }

                        if(traj["points"]) {
                            for (const auto& point : traj["points"]) {
                                trajectory_msgs::msg::JointTrajectoryPoint pt;
                                for (const auto& pos : point["positions"]) pt.positions.push_back(pos.as<double>());
                                if (point["velocities"]) for (auto v : point["velocities"]) pt.velocities.push_back(v.as<double>());
                                
                                double tfs = point["time_from_start"].as<double>();
                                pt.time_from_start.sec = (int)tfs;
                                pt.time_from_start.nanosec = (int)((tfs - pt.time_from_start.sec)*1e9);

                                if (point["gripper"]) traj_info.gripper_value = point["gripper"].as<double>();
                                
                                traj_info.trajectory.points.push_back(pt);
                            }
                        }
                        plan_info.trajectories.push_back(traj_info);
                    }
                }
                plans.push_back(plan_info);
            }
            RCLCPP_INFO(this->get_logger(), "Caricato %s: %zu piani trovati.", yaml_file.c_str(), plans.size());
            return true;
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore YAML in %s: %s", yaml_file.c_str(), e.what());
            return false;
        }
    }
    
    void publishTrajectoryMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;
        for (const auto& plan : left_plans_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world"; // Assicurati sia world
            marker.header.stamp = this->now();
            marker.ns = "trajectory_targets_left";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = plan.target_pose.pose;
            marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05;
            marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }
        for (const auto& plan : right_plans_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "trajectory_targets_right";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = plan.target_pose.pose;
            marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05;
            marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ExecuteYamlTrajectoriesNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}