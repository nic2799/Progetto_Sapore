#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class InteractiveMarkerPublisher : public rclcpp::Node
{
public:
 InteractiveMarkerPublisher()
 : Node("interactive_marker_publisher")
 {
  //Definiamo un publisher per pubblicare la posa selezionata tramite l'interactive marker
   pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
 }

 void initInteractiveMarker()
 {//definizione del server per l'interactive marker
   server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
       "reachability_marker",
       shared_from_this() // qui funziona perché l'oggetto è già in shared_ptr
   );

   make6DofMarker("target_pose_marker");
   server_->applyChanges();
 }



private:
   std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

 void make6DofMarker(const std::string &name)
 {
   visualization_msgs::msg::InteractiveMarker int_marker;
   int_marker.header.frame_id = "world";
   int_marker.name = name;
   int_marker.description = "Move me!";
   int_marker.scale = 0.2;

   // initial pose
   int_marker.pose.position.x = 0.4;
   int_marker.pose.position.y = 0.0;
   int_marker.pose.position.z = 0.3;



   // Add 6-DOF controls
   add6DofControls(int_marker);
//inseriamo il marker nel server con la callback per il feedback
   server_->insert(int_marker,
     std::bind(&InteractiveMarkerPublisher::processFeedback, this, std::placeholders::_1));
 }

 void add6DofControls(visualization_msgs::msg::InteractiveMarker &marker)
 {
   visualization_msgs::msg::InteractiveMarkerControl control;

   control.always_visible = true;

   // X axis move
   control.orientation.w = 1;
   control.orientation.x = 1;
   control.orientation.y = 0;
   control.orientation.z = 0;
   control.name = "move_x";
   control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
   marker.controls.push_back(control);

   // Y axis move
   control.orientation.x = 0;
   control.orientation.y = 1;
   control.name = "move_y";
   marker.controls.push_back(control);

   // Z axis move
   control.orientation.y = 0;
   control.orientation.z = 1;
   control.name = "move_z";
   marker.controls.push_back(control);

   // Rotations
   control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
   control.name = "rotate_x";
   control.orientation.x = 1;
   control.orientation.y = 0;
   control.orientation.z = 0;
   marker.controls.push_back(control);

   control.name = "rotate_y";
   control.orientation.x = 0;
   control.orientation.y = 1;
   control.orientation.z = 0;
   marker.controls.push_back(control);

   control.name = "rotate_z";
   control.orientation.x = 0;
   control.orientation.y = 0;
   control.orientation.z = 1;
   marker.controls.push_back(control);
 }

 void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
 {
   if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
   {
     geometry_msgs::msg::PoseStamped pose_msg;
     pose_msg.header = feedback->header;
     pose_msg.pose = feedback->pose;
     pose_pub_->publish(pose_msg);
   }
 }
};

int main(int argc, char **argv)
{
 rclcpp::init(argc, argv);
 auto node = std::make_shared<InteractiveMarkerPublisher>();
node->initInteractiveMarker();
rclcpp::spin(node);
 rclcpp::shutdown();
 return 0;
}

