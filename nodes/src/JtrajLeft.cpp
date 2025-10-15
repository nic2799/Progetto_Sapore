#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <interfaces/action/jtraj.hpp>

class JtrajLeftNode : public rclcpp::Node
{
private: 
    using Jtraj = interfaces::action::Jtraj;
    using GoalHandleJtraj = rclcpp_action::ServerGoalHandle<Jtraj>;
    rclcpp_action::Server<Jtraj>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    sensor_msgs::msg::JointState last_joint_state_;
    const double T = 0.01; 
    
    // Joint names for left IIWA arm (7 DOF)
    const std::vector<std::string> JOINT_NAMES = {
        "left_joint_a1", "left_joint_a2", "left_joint_a3", "left_joint_a4", 
        "left_joint_a5", "left_joint_a6", "left_joint_a7"
    };

public:
    bool last_joint_state_received_ = false;

    JtrajLeftNode() : Node("jtraj_left_node")
    {
        // Publisher for left arm joint commands
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_arm/cmd/joint_position", 1);
        
        // Subscription to joint states
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 1,
            std::bind(&JtrajLeftNode::jointStateCallback, this, std::placeholders::_1));
        
        // Action server for left arm
        action_server_ = rclcpp_action::create_server<Jtraj>(
            this,
            "jtraj_left",
            std::bind(&JtrajLeftNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&JtrajLeftNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&JtrajLeftNode::handle_accepted, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Action server 'jtraj_left' started for left IIWA arm.");
    }
   
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Jtraj::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request for LEFT arm with tf %f", goal->tf);
        (void)uuid;
        
        // Check if goal has correct number of joints (7 for IIWA)
        if (goal->qf.size() != JOINT_NAMES.size()) {
            RCLCPP_ERROR(this->get_logger(), "Goal has %zu joints, expected %zu for IIWA", 
                        goal->qf.size(), JOINT_NAMES.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleJtraj> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel LEFT arm goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleJtraj> goal_handle)
    {
        std::thread{std::bind(&JtrajLeftNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleJtraj> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Jtraj::Feedback>();
        auto result = std::make_shared<Jtraj::Result>();
        rclcpp::Time start_wait = this->now();

        // Wait for joint states
        while (rclcpp::ok()) {
            if (last_joint_state_received_) break;
            
            if ((this->now() - start_wait).seconds() > 10.0) {
                RCLCPP_ERROR(this->get_logger(), "[jtraj_left] Timeout: no joint_states received.");
                goal_handle->abort(result);
                return;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        // Extract current positions for left arm joints
        std::vector<double> qi = extractLeftArmPositions();
        generate_quintic_trajectory(goal_handle, qi, feedback, result);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = *msg;
        last_joint_state_received_ = true;
    }
    
    std::vector<double> extractLeftArmPositions()
    {
        std::vector<double> left_positions(JOINT_NAMES.size(), 0.0);
        
        if (!last_joint_state_received_) {
            RCLCPP_WARN(this->get_logger(), "No joint state received, assuming zero positions for left arm.");
            return left_positions;
        }
        
        // Extract positions for left arm joints
        for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
            auto it = std::find(last_joint_state_.name.begin(), last_joint_state_.name.end(), JOINT_NAMES[i]);
            if (it != last_joint_state_.name.end()) {
                size_t index = std::distance(last_joint_state_.name.begin(), it);
                if (index < last_joint_state_.position.size()) {
                    left_positions[i] = last_joint_state_.position[index];
                }
            }
        }
        
        return left_positions;
    }
    
    void publish_q(const std::vector<double> & q)
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = JOINT_NAMES;
        message.position = q;
        publisher_->publish(message);
    }
    
    void generate_quintic_trajectory(const std::shared_ptr<GoalHandleJtraj> &goal_handle, 
                                   const std::vector<double> &qi, 
                                   std::shared_ptr<Jtraj::Feedback> feedback, 
                                   std::shared_ptr<Jtraj::Result> result)
    {
        const auto goal = goal_handle->get_goal();
        rclcpp::Rate loop_rate(1/T);
        double N = goal->tf / T;
        double tau = 0;

        std::vector<double> q(qi.size());
       
        for (size_t i = 1; i <= N && rclcpp::ok(); i++) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "LEFT arm goal canceled.");
                return;
            }
           
            tau = i / N;
            for (size_t j = 0; j < qi.size(); j++){
                q[j] = qi[j] + (goal->qf[j] - qi[j]) * (6 * pow(tau, 5) - 15 * pow(tau, 4) + 10 * pow(tau, 3));
            }
            
            publish_q(q);
            feedback->q_now = q;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }
        
        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "LEFT arm goal succeeded.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JtrajLeftNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}