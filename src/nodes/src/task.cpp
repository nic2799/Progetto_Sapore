#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <interfaces/action/jtraj.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <memory>
#include <thread>

class DualArmTaskNode : public rclcpp::Node
{
private:
    using Jtraj = interfaces::action::Jtraj;
    using JtrajGoalHandle = rclcpp_action::ClientGoalHandle<Jtraj>;
    
    // Action clients for both arms
    rclcpp_action::Client<Jtraj>::SharedPtr left_arm_client_;
    rclcpp_action::Client<Jtraj>::SharedPtr right_arm_client_;
    
    // Subscription to monitor joint states
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    // Timer for executing task sequence
    rclcpp::TimerBase::SharedPtr task_timer_;
    
    // Task state management
    enum class TaskState {
        IDLE,
        MOVING_TO_HOME,
        MOVING_TO_CONFIG1,
        MOVING_TO_CONFIG2,
        MOVING_TO_CONFIG3,
        TASK_COMPLETE
    };
    
    TaskState current_state_;
    bool left_arm_ready_;
    bool right_arm_ready_;
    
    // Predefined configurations (7 DOF for each IIWA arm)
    struct ArmConfiguration {
        std::vector<double> left_arm;
        std::vector<double> right_arm;
        double duration;
        std::string description;
    };
    
    std::vector<ArmConfiguration> configurations_;

public:
    DualArmTaskNode() : Node("dual_arm_task_node"), 
                        current_state_(TaskState::IDLE),
                        left_arm_ready_(false), 
                        right_arm_ready_(false)
    {
        // Initialize action clients
        left_arm_client_ = rclcpp_action::create_client<Jtraj>(this, "jtraj_left");
        right_arm_client_ = rclcpp_action::create_client<Jtraj>(this, "jtraj_right");
        
        // Subscribe to joint states for monitoring
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&DualArmTaskNode::jointStateCallback, this, std::placeholders::_1));
        
        // Initialize predefined configurations
        initializeConfigurations();
        
        // Wait for action servers to be available
        RCLCPP_INFO(this->get_logger(), "Waiting for action servers...");
        
        if (!left_arm_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Left arm action server not available!");
            return;
        }
        
        if (!right_arm_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Right arm action server not available!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Action servers ready! Starting task execution...");
        
        // Start task execution timer
        task_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DualArmTaskNode::executeTask, this));
    }

private:
    void initializeConfigurations()
    {
        // Configuration 1: Home position (all joints at 0)
        configurations_.push_back({
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Left arm
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Right arm
            3.0,  // Duration
            "Home Position"
        });
        
        // Configuration 2: Symmetric spread (arms extending outward)
        configurations_.push_back({
            {0.5, -0.3, 0.0, -1.2, 0.0, 0.8, 0.0},   // Left arm
            {-0.5, -0.3, 0.0, -1.2, 0.0, 0.8, 0.0},  // Right arm
            4.0,
            "Symmetric Spread"
        });
        
        // Configuration 3: Collaborative position (arms closer together)
        configurations_.push_back({
            {0.3, -0.5, 0.2, -1.0, -0.2, 1.0, 0.1},  // Left arm
            {-0.3, -0.5, -0.2, -1.0, 0.2, 1.0, -0.1}, // Right arm
            4.5,
            "Collaborative Position"
        });
        
        // Configuration 4: Working position (arms ready for manipulation)
        configurations_.push_back({
            {0.2, -0.8, 0.1, -1.5, 0.0, 0.7, 0.0},   // Left arm
            {-0.2, -0.8, -0.1, -1.5, 0.0, 0.7, 0.0}, // Right arm
            3.5,
            "Working Position"
        });
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Monitor joint states (optional: add safety checks, logging, etc.)
        // This could be used to verify that movements are being executed correctly
    }
    
    void executeTask()
    {
        static size_t config_index = 0;
        static bool first_execution = true;
        
        switch (current_state_) {
            case TaskState::IDLE:
                RCLCPP_INFO(this->get_logger(), "Starting dual-arm task sequence...");
                current_state_ = TaskState::MOVING_TO_HOME;
                config_index = 0;
                first_execution = true;
                break;
                
            case TaskState::MOVING_TO_HOME:
            case TaskState::MOVING_TO_CONFIG1:
            case TaskState::MOVING_TO_CONFIG2:
            case TaskState::MOVING_TO_CONFIG3:
                // For the first execution or when both arms are ready
                if (first_execution || (left_arm_ready_ && right_arm_ready_)) {
                    if (config_index < configurations_.size()) {
                        executeConfiguration(configurations_[config_index]);
                        config_index++;
                        
                        // Update state based on configuration
                        if (config_index == 1) current_state_ = TaskState::MOVING_TO_CONFIG1;
                        else if (config_index == 2) current_state_ = TaskState::MOVING_TO_CONFIG2;
                        else if (config_index == 3) current_state_ = TaskState::MOVING_TO_CONFIG3;
                        else current_state_ = TaskState::TASK_COMPLETE;
                        
                        left_arm_ready_ = false;
                        right_arm_ready_ = false;
                        first_execution = false;
                    }
                }
                break;
                
            case TaskState::TASK_COMPLETE:
                RCLCPP_INFO(this->get_logger(), "All configurations executed successfully!");
                RCLCPP_INFO(this->get_logger(), "Task completed. Shutting down...");
                rclcpp::shutdown();
                break;
        }
    }
    
    void executeConfiguration(const ArmConfiguration& config)
    {
        RCLCPP_INFO(this->get_logger(), "Executing configuration: %s", config.description.c_str());
        
        // Send goals to both arms simultaneously
        sendGoalToLeftArm(config.left_arm, config.duration);
        sendGoalToRightArm(config.right_arm, config.duration);
    }
    
    void sendGoalToLeftArm(const std::vector<double>& target_position, double duration)
    {
        auto goal_msg = Jtraj::Goal();
        goal_msg.qf = target_position;
        goal_msg.tf = duration;
        
        auto send_goal_options = rclcpp_action::Client<Jtraj>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](auto goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Left arm goal was rejected!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Left arm goal accepted");
                }
            };
            
        send_goal_options.feedback_callback =
            [this](auto, const std::shared_ptr<const Jtraj::Feedback> feedback) {
                // Optional: log feedback
                // RCLCPP_DEBUG(this->get_logger(), "Left arm feedback received");
            };
            
        send_goal_options.result_callback =
            [this](const JtrajGoalHandle::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Left arm reached target position!");
                    left_arm_ready_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Left arm failed to reach target!");
                }
            };
        
        left_arm_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    void sendGoalToRightArm(const std::vector<double>& target_position, double duration)
    {
        auto goal_msg = Jtraj::Goal();
        goal_msg.qf = target_position;
        goal_msg.tf = duration;
        
        auto send_goal_options = rclcpp_action::Client<Jtraj>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](auto goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Right arm goal was rejected!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Right arm goal accepted");
                }
            };
            
        send_goal_options.feedback_callback =
            [this](auto, const std::shared_ptr<const Jtraj::Feedback> feedback) {
                // Optional: log feedback
                // RCLCPP_DEBUG(this->get_logger(), "Right arm feedback received");
            };
            
        send_goal_options.result_callback =
            [this](const JtrajGoalHandle::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Right arm reached target position!");
                    right_arm_ready_ = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Right arm failed to reach target!");
                }
            };
        
        right_arm_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Dual-Arm Task Execution Node");
    
    auto node = std::make_shared<DualArmTaskNode>();
    
    // Use MultiThreadedExecutor for handling multiple action clients
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}