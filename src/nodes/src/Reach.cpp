// costruiamo un nodo che ci permetta dato un punto verificarne la raggiungibilita
// e nel caso in cui non fosse raggiungibile, per vedere a occhio dove si genera
// la collisione andiamo a utilizzare FollowJointTrajectory per visualizzare
// la posa finale del robot in RViz.

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <algorithm>

using moveit_msgs::srv::GetPositionIK;

class ReachabilityIKNode : public rclcpp::Node
{
public:
    ReachabilityIKNode() : Node("reachability_ik_node")
    {
        // Client per servizio IK
        client_ = this->create_client<GetPositionIK>("/compute_ik");
        RCLCPP_INFO(this->get_logger(), "Attendo servizio /compute_ik...");
        client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Servizio /compute_ik disponibile!");

        // Client per azione FollowJointTrajectory
        traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/right_arm_controller/follow_joint_trajectory");

        RCLCPP_INFO(this->get_logger(), "Attendo azione /right_arm_controller/follow_joint_trajectory...");
        if (!traj_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Azione FollowJointTrajectory non disponibile!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Azione FollowJointTrajectory disponibile!");
        }
    }

    void setSinglePoint()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.pose.position.x = -1.050;
        pose.pose.position.y = -2.000;
        pose.pose.position.z = 0.850;
        pose.pose.orientation.x = 0.500000;
        pose.pose.orientation.y = 0.500000;
        pose.pose.orientation.z = -0.499602;
        pose.pose.orientation.w = 0.500398;

     

        // Prima richiesta IK con avoid_collisions = true
        auto req = std::make_shared<GetPositionIK::Request>();
        req->ik_request.group_name = "right_arm";
        req->ik_request.pose_stamped = pose;
        req->ik_request.ik_link_name = "right_tcp";
        req->ik_request.avoid_collisions = true;
        req->ik_request.timeout.sec = 1;

        auto future = client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(), "Timeout per richiesta IK.");
            return;
        }

        auto response = future.get();
        bool reachable = (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "IK response code = %d", response->error_code.val);

        if (!reachable)
        {
            RCLCPP_WARN(this->get_logger(), "IK (avoid_collisions=true) fallita, provo con avoid_collisions=false...");

            // Seconda richiesta: avoid_collisions = false
            auto req2 = std::make_shared<GetPositionIK::Request>();
            req2->ik_request.group_name = "right_arm";
            req2->ik_request.pose_stamped = pose;
            req2->ik_request.ik_link_name = "right_tcp";
            req2->ik_request.avoid_collisions = false;
            req2->ik_request.timeout.sec = 1;

            auto future2 = client_->async_send_request(req2);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future2) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "Timeout per seconda richiesta IK.");
                return;
            }

            auto response2 = future2.get();
            if (response2->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "La posa è in collisione, ma è raggiungibile (avoid_collisions=false).");

                // Stampa compatta della soluzione IK
                std::ostringstream oss;
                size_t n = std::min(response2->solution.joint_state.name.size(),
                                    response2->solution.joint_state.position.size());
               /* for (size_t i = 0; i < n; ++i)
                {
                    oss << response2->solution.joint_state.name[i]
                        << "=" << response2->solution.joint_state.position[i];
                    if (i + 1 < n)
                        oss << ", ";
                }
                RCLCPP_INFO(this->get_logger(), "Joint solution: %s", oss.str().c_str());
             POSSIAMO NOTARE solution.joint_state.position CONTIENE TUTTI I GIUNTI DEL ROBOT A NOI INTERESSANO SOLO QUELLI
             DEL RIGHT ARM   */

                // Costruzione del messaggio di traiettoria filtrando solo i joint controllati
                std::vector<std::string> controller_joints = {
                    "right_joint_a1", "right_joint_a2", "right_joint_a3", "right_joint_a4",
                    "right_joint_a5", "right_joint_a6", "right_joint_a7"};

                trajectory_msgs::msg::JointTrajectory traj;//definiamo i punti della traiettoria
                traj.joint_names = controller_joints;

                trajectory_msgs::msg::JointTrajectoryPoint point;
                for (const auto &jname : controller_joints)
                {
                    auto it = std::find(response2->solution.joint_state.name.begin(),
                                        response2->solution.joint_state.name.end(), jname);
                    if (it != response2->solution.joint_state.name.end())
                    {
                        size_t idx = std::distance(response2->solution.joint_state.name.begin(), it);
                        point.positions.push_back(response2->solution.joint_state.position[idx]);
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Joint %s non trovata nella soluzione IK!", jname.c_str());
                        return;
                    }
                }
                point.time_from_start = rclcpp::Duration::from_seconds(5.0);//in quanto tempo deve raggiungere la posa
                traj.points.push_back(point);

                auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
                goal_msg.trajectory = traj;

                // Callback di risultato
                auto send_goal_options =
                    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
                send_goal_options.result_callback =
                    [this](const rclcpp_action::ClientGoalHandle<
                           control_msgs::action::FollowJointTrajectory>::WrappedResult &result)
                {
                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                    {
                        RCLCPP_INFO(this->get_logger(), "Movimento completato con successo!");
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Movimento terminato con codice: %d", static_cast<int>(result.code));
                    }
                };

                // Invio del goal
                traj_client_->async_send_goal(goal_msg, send_goal_options);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Non trova soluzione neanche con avoid_collisions=false.");
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Posa raggiungibile senza collisioni.");
        }
    }

private:
    rclcpp::Client<GetPositionIK>::SharedPtr client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachabilityIKNode>();
    node->setSinglePoint();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
