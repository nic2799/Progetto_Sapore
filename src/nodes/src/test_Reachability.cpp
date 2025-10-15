#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class ReachabilityNode : public rclcpp::Node
{
public:
    ReachabilityNode() : Node("reachability_node") {}

    // Metodo per inizializzare MoveGroupInterface dopo che il nodo è in shared_ptr
    void initializeMoveGroup()
    {
        // Nome del planning group del robot
        const std::string PLANNING_GROUP = "left_arm";

        // Passo lo shared_ptr del nodo (shared_from_this()) a MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), PLANNING_GROUP
        );
        move_group_->setPoseReferenceFrame("world");

        // Analisi di raggiungibilità
        checkReachability(-0.25, 1.60, -0.5); // y e z
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    void checkReachability(double y, double z, double x)
    {
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;  // direzione neutra
        pose.position.x = x;     // puoi scegliere un valore fisso per x
        pose.position.y = y;
        pose.position.z = z;

        move_group_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success)
            RCLCPP_INFO(this->get_logger(), "Il punto (%.2f, %.2f, %.2f) e' raggiungibile!", pose.position.x, y, z);
        else
            RCLCPP_WARN(this->get_logger(), "Il punto (%.2f, %.2f, %.2f) NON e' raggiungibile!", pose.position.x, y, z);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Nodo creato in shared_ptr
    auto node = std::make_shared<ReachabilityNode>();

    // Inizializzo MoveGroupInterface dopo che il nodo è in shared_ptr
    node->initializeMoveGroup();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
