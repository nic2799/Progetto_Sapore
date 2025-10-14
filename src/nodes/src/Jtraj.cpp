#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <interfaces/action/jtraj.hpp>



class JtrajNode : public rclcpp::Node
{
    private: 
     using Jtraj = interfaces::action::Jtraj;
     using GoalHandleJtraj = rclcpp_action::ServerGoalHandle<Jtraj>;
     rclcpp_action::Server<Jtraj>::SharedPtr action_server_;
     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
     sensor_msgs::msg::JointState last_joint_state_;
     const double T= 0.01; 
     const std::vector<std::string>  JOINT_NAMES = {"meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint", "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"};

    


    public:
            bool last_joint_state_received_ = false;

  JtrajNode() : Node("jtraj_node")
  {
    // Publisher for joint states
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd/joint_position", 1);
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1,
      std::bind(&JtrajNode::jointStateCallback, this, std::placeholders::_1));
    action_server_ = rclcpp_action::create_server<Jtraj>(
      this,
      "jtraj",
      std::bind(&JtrajNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JtrajNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&JtrajNode::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Action server 'jtraj' started.");
  

    

  }
   
      rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const Jtraj::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with tf %f", goal->tf);
            (void)uuid;
            
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleJtraj> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void handle_accepted(const std::shared_ptr<GoalHandleJtraj> goal_handle)
        {
            std::thread{std::bind(&JtrajNode::execute, this, std::placeholders::_1), goal_handle}.detach();
        }
    void execute(const std::shared_ptr<GoalHandleJtraj> goal_handle){
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Jtraj::Feedback>();
        auto result = std::make_shared<Jtraj::Result>();
        rclcpp::Time start_wait = this->now();

      while (rclcpp::ok()) {
            {
                if (last_joint_state_received_) break;
            }
            if ((this->now() - start_wait).seconds() > 10.0) {
                RCLCPP_ERROR(this->get_logger(), "[jtraj] Timeout: nessun messaggio ricevuto su joint_states.");
                goal_handle->abort(result);
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }


            std::vector<double> qi;
            {
                if (last_joint_state_received_ && last_joint_state_.position.size() == JOINT_NAMES.size()) {
                    qi = last_joint_state_.position;
                } else {
                    qi = std::vector<double>(JOINT_NAMES.size(), 0.0);//OPPURE INIZIALIZZA A ZERO
                    RCLCPP_WARN(this->get_logger(), "No joint state ricevuta, assumiamo zero position.");
                }
            }
                    generate_quintic_trajectory(goal_handle, qi, feedback, result);




    }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
   last_joint_state_ = *msg;
  
    last_joint_state_received_ = true;
     
  }
  void publish_q(const std::vector<double> & q)
        {
            auto message = sensor_msgs::msg::JointState();
            message.name =JOINT_NAMES;
            message.position = q;
            publisher_->publish(message);
        }
  void generate_quintic_trajectory(const std::shared_ptr<GoalHandleJtraj> &goal_handle, const std::vector<double> &qi, std::shared_ptr<Jtraj::Feedback> feedback, std::shared_ptr<Jtraj::Result> result)
        {
            const auto goal = goal_handle->get_goal();
            rclcpp::Rate loop_rate(1/T);
            double N = goal->tf / T;
            double tau = 0;


            std::vector<double> q(qi.size());//inizializza q con stessa dimensione di qi
       
            for (size_t i = 1; i <= N && rclcpp::ok(); i++) {
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
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
                RCLCPP_INFO(this->get_logger(), "Goal succeeded. Result published. Success: %s", result->success ? "true" : "false");
            }
        }


};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JtrajNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}