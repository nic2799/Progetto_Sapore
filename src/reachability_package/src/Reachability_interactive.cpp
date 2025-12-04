#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <sensor_msgs/msg/joint_state.hpp> 

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/object_color.hpp> 
#include <std_msgs/msg/color_rgba.hpp>  

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/object_color.hpp> 
#include <std_msgs/msg/color_rgba.hpp>     // E anche questo

#include <moveit_msgs/msg/display_trajectory.hpp> //
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/moveit_error_code.h>


using moveit_msgs::srv::GetPositionIK;

class ReachabilityIKNode : public rclcpp::Node
{
public:
    ReachabilityIKNode() : Node("reachability_ik_node") 
    {

     // --- Leggiamo il lato del braccio (right o left) ---
        this->declare_parameter<std::string>("arm_side", "left");
        arm_side_ = this->get_parameter("arm_side").as_string();

        // --- Costruiamo i nomi dinamicamente ---
        group_ = arm_side_ + "_arm";
        tcp_link_ = arm_side_ + "_tcp";
        controller_name_ = "/" + arm_side_ + "_arm_controller/follow_joint_trajectory";
        joint_prefix_ = arm_side_ + "_";

        RCLCPP_INFO(this->get_logger(), "Configurazione braccio: %s", arm_side_.c_str());
        RCLCPP_INFO(this->get_logger(), "Gruppo MoveIt: %s", group_.c_str());
        RCLCPP_INFO(this->get_logger(), "TCP link: %s", tcp_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Controller: %s", controller_name_.c_str());


        client_ = this->create_client<GetPositionIK>("/compute_ik");
        RCLCPP_INFO(this->get_logger(), "Attendo servizio /compute_ik...");
        client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Servizio /compute_ik disponibile!");
        traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, controller_name_);
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

        // Rimosso: group_ è già impostato correttamente da arm_side_ sopra
        // this->declare_parameter("group_name", "right_arm");
        // group_ = this->get_parameter("group_name").as_string();

                // display robot state publisher
                display_state_pub_ = this->create_publisher<moveit_msgs::msg::DisplayRobotState>("/display_robot_state", 10);
                // publisher per pubblicare la traiettoria (tipo corretto DisplayTrajectory)
                display_traj_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);


    }

    void init()
    {
          
      //moveit::planning_interface::MoveGroupInterface move_group_interface( this->shared_from_this(), group_);
    //  rclcpp::sleep_for(std::chrono::milliseconds(4000));

        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            this->shared_from_this(), "robot_description");

        if (!planning_scene_monitor_->getPlanningScene())   
            throw std::runtime_error("Impossibile inizializzare il PlanningSceneMonitor");

        planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
        RCLCPP_INFO(this->get_logger(), "PlanningSceneMonitor inizializzato correttamente.");

                 if (!planning_scene_monitor_->getPlanningScene())
                {
                    RCLCPP_ERROR(this->get_logger(), "Errore: impossibile creare PlanningSceneMonitor.");
                    return;
                }

                //psm->startSceneMonitor();
                planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
                //psm->startWorldGeometryMonitor();
                //psm->startStateMonitor();
                //rclcpp::sleep_for(500ms);
                planning_scene = planning_scene_monitor_->getPlanningScene();
                if (!move_group_interface) {
                // crea localmente se non presente (ma è meglio inizializzare nel ctor)
                move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), group_);
                    rclcpp::sleep_for(std::chrono::milliseconds(4000));

    }

            
    }

    void spinLoop()
    {
        rclcpp::Rate rate(10); // 10 Hz loop
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());
            bool raggiunto_orientamento_modificato = false;

            if (started_ && has_pose_)
            {
                bool raggiunto = testReachability(last_pose_, group_);
                if(!raggiunto){

                    RCLCPP_INFO(this->get_logger(), "Non raggiungibile verifichiamo con altro orientamento...");
                    tf2::Quaternion q_orig;
                    tf2::Quaternion q_rot;
                    tf2::fromMsg(last_pose_.pose.orientation, q_orig);
                    double Delta_max = 45*M_PI / 180; 
                    for (double delta : {+Delta_max, -Delta_max}){
                    RCLCPP_INFO(this->get_logger(), "Proviamo con delta orientamento di %f rad.", delta);
                    
                    q_rot.setRPY(delta, 0, 0); // rotazione attorno all'asse Z del tcp
                    tf2::Quaternion q_new = q_orig * q_rot;
                    q_new.normalize();  
                    geometry_msgs::msg::Quaternion ros_q_new = tf2::toMsg(q_new);
                    geometry_msgs::msg::PoseStamped new_pose = last_pose_;
                    new_pose.pose.orientation = ros_q_new;
                    raggiunto_orientamento_modificato = testReachability(new_pose, group_);
                    if(raggiunto_orientamento_modificato){
                        RCLCPP_INFO(this->get_logger(), "RAGGIUNGIBILE CON ORIENTAMENTO MODIFICATO DI 10 GRADI.");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "NON RAGGIUNGIBILE ANCHE CON ORIENTAMENTO MODIFICATO.");
                    }
                }

                }else if(raggiunto || raggiunto_orientamento_modificato){
                    RCLCPP_INFO(this->get_logger(), "Punto raggiungibile senza collisioni.");
                    bool pour = testPlannerPouring(last_pose_, group_, 120.0 * M_PI/180.0, 0.8);
                    if(pour){
                        RCLCPP_INFO(this->get_logger(), "Planner ha trovato traiettoria per pouring.");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Planner NON ha trovato traiettoria per pouring.");
                    }
                }
                started_ = false; // reset per evitare loop continui
            }

            rate.sleep();
        }
    }




   bool testPlannerPouring(const geometry_msgs::msg::PoseStamped &base_pose,
                        const std::string &group_name,
                        double final_z_rotation_rad = 120.0 * M_PI / 180.0, // rotazione finale attorno a Z
                        double step = 0.01)
{
    using MoveItErrorCode = moveit::core::MoveItErrorCode;

    move_group_interface->setPlannerId("RRTConnectkConfigDefault");
    move_group_interface->setPlanningTime(100.0);
    move_group_interface->setPoseReferenceFrame(base_pose.header.frame_id);

    // --- STEP 1: Pianifica movimento verso base_pose ---
    RCLCPP_INFO(this->get_logger(), "Step 1: Pianificazione verso base_pose (pre-pouring)...");
    
    move_group_interface->setStartStateToCurrentState();
    move_group_interface->setPoseTarget(base_pose.pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan_to_base;
    MoveItErrorCode result_base = move_group_interface->plan(plan_to_base);
    
    if (result_base != MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Piano verso base_pose fallito: %d", result_base.val);
        move_group_interface->clearPoseTargets();
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Step 1: Piano verso base_pose completato.");
    
    // Ottieni lo stato finale del piano 1 (sarà lo start del piano 2)
    moveit::core::RobotState end_state_base(*move_group_interface->getCurrentState());
    const auto& last_point_base = plan_to_base.trajectory.joint_trajectory.points.back();
    end_state_base.setJointGroupPositions(group_name, last_point_base.positions);
    end_state_base.update();
    
    move_group_interface->clearPoseTargets();

    // --- STEP 2:  Plan per rotazione (pouring) ---
    const auto &joint_names = plan_to_base.trajectory.joint_trajectory.joint_names;
    const auto &last_point = plan_to_base.trajectory.joint_trajectory.points.back();
    
    moveit::core::RobotState start_state(move_group_interface->getRobotModel());
    start_state.setVariablePositions(joint_names, last_point.positions);
    start_state.update();

    //  Imposta stato finale del primo piano come start per il secondo
    move_group_interface->setStartState(start_state); 
    
    //  Tolleranze STRETTE per movimento rotazionale preciso
    move_group_interface->setGoalJointTolerance(0.01);  // ±0.57° per ogni giunto (molto preciso!)
    
    //  Copia le posizioni correnti
    std::vector<double> joint_positions;
    start_state.copyJointGroupPositions(group_name, joint_positions);

    //  Ruota solo il giunto A7
    int joint_a7_index = 6; //settimo elemento i joint positions
    double posizione_iniziale_a7 = joint_positions[joint_a7_index];
    joint_positions[joint_a7_index] += final_z_rotation_rad;
    
    RCLCPP_INFO(this->get_logger(), "  A7 posizione iniziale: %.3f rad (%.1f°)", 
                posizione_iniziale_a7, posizione_iniziale_a7 * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  A7 target finale: %.3f rad (%.1f°)", 
                joint_positions[joint_a7_index], joint_positions[joint_a7_index] * 180.0 / M_PI);

    //  Imposta SOLO target sui giunti (non setPoseTarget!)
    move_group_interface->setJointValueTarget(joint_positions);




    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    MoveItErrorCode result2 = move_group_interface->plan(plan2);

    if (result2 != MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Piano 2 fallito (codice %d)", result2.val);
        
        move_group_interface->clearPoseTargets();
        return false;
    }

  
for (const auto& point : plan_to_base.trajectory.joint_trajectory.points) {
    moveit_msgs::msg::DisplayRobotState display_msg;
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = plan_to_base.trajectory.joint_trajectory.joint_names;
    joint_state.position = point.positions;
    joint_state.velocity = point.velocities;
    joint_state.effort = point.effort;
    display_msg.state.joint_state = joint_state;
    display_state_pub_->publish(display_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
}

for (const auto& point : plan2.trajectory.joint_trajectory.points) {
    moveit_msgs::msg::DisplayRobotState display_msg;
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = plan2.trajectory.joint_trajectory.joint_names;
    joint_state.position = point.positions;
    joint_state.velocity = point.velocities;
    joint_state.effort = point.effort;
    display_msg.state.joint_state = joint_state;
    display_state_pub_->publish(display_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
}
    RCLCPP_INFO(this->get_logger(), "Traiettoria di pouring pianificata e pubblicata con successo!");
    return true;
}


private:
    rclcpp::Client<GetPositionIK>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr marker_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    planning_scene::PlanningScenePtr planning_scene;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_client_;
   // rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr display_state_pub_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
       // rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_state_pub_;
     rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr display_state_pub_;
     // publisher per il tipo DisplayTrajectory usato per mostrare la traiettoria in RViz
     rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_;

    

    bool started_ = false;

    geometry_msgs::msg::PoseStamped last_pose_;
    bool has_pose_ = false;
    std::string group_;

    std::string arm_side_;
    std::string tcp_link_;
    std::string controller_name_;
    std::string joint_prefix_;

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
        (void)request;//per evitare warning unused variable
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
    bool CheckLinkCollision(bool reachable, const geometry_msgs::msg::PoseStamped &pose){
         bool have_candidate = false;
         sensor_msgs::msg::JointState candidate_js;
         moveit_msgs::msg::DisplayRobotState display_msg;
         moveit::core::RobotState current_state(planning_scene->getRobotModel());
            if (!reachable)
            { 
                RCLCPP_INFO(this->get_logger(), "iL PUNTO PUO ESSERE O NON RAGGIUNGIBILE O IN COLLISIONE. SETTIAMO AVOID_COLLISIONS=FALSE E RITENTIAMO.");
              
                moveit::core::RobotState current_state = planning_scene->getCurrentState();
                const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(group_);
                bool found_ik = current_state.setFromIK(joint_model_group, pose.pose,1);
                if (found_ik)//dunque il punto è raggiungibile ma in collisione
                {
                    // Estraggo i joint states
                    current_state.copyJointGroupPositions(
                        joint_model_group, candidate_js.position);
                    candidate_js.name = joint_model_group->getVariableNames();
                    have_candidate = true;
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
        
            if (have_candidate)//SE TROVO POSA RAGGIUNGIBILE è QUINDI IN COLLISIONE
            {
                

            current_state.setVariablePositions(candidate_js.name, candidate_js.position);
            current_state.update();
            

            collision_detection::CollisionRequest collision_request;
            collision_request.contacts = true;
            collision_request.max_contacts = 100;
            collision_request.max_contacts_per_pair = 5;
            collision_request.group_name = group_;

            collision_detection::CollisionResult collision_result;
            planning_scene->checkCollision(collision_request, collision_result, current_state);
            

                // 1. Crea il messaggio DisplayRobotState
                display_msg.state.joint_state = candidate_js; // Usa i joint_state che hai trovato

                // 2. Definisci il colore per l'evidenziazione
                std_msgs::msg::ColorRGBA highlight_color;
                highlight_color.r = 1.0; // Rosso
                highlight_color.g = 0.0;
                highlight_color.b = 0.0;
                highlight_color.a = 0.9; // Quasi opaco

                RCLCPP_WARN(this->get_logger(), "LA COLLISIONE È TRA : ");
                for (const auto &entry : collision_result.contacts)
                {
                    RCLCPP_INFO(this->get_logger(), " - Link '%s' collide con '%s' (%zu contatti)",
                                entry.first.first.c_str(), entry.first.second.c_str(), entry.second.size());
                    
                    // 3. Aggiungi i link da evidenziare
                    // Controlliamo che il link appartenga al robot e non sia "world" o un ostacolo
                    if (current_state.getRobotModel()->hasLinkModel(entry.first.first)) {
                        moveit_msgs::msg::ObjectColor obj_color;
                        obj_color.id = entry.first.first; // Nome del primo link
                        obj_color.color = highlight_color;
                        display_msg.highlight_links.push_back(obj_color);
                    }
                    if (current_state.getRobotModel()->hasLinkModel(entry.first.second)) {
                        moveit_msgs::msg::ObjectColor obj_color;
                        obj_color.id = entry.first.second; // Nome del secondo link
                        obj_color.color = highlight_color;
                        display_msg.highlight_links.push_back(obj_color);
                    }
                }

                // 4. Pubblica il messaggio
                display_state_pub_->publish(display_msg);            

            RCLCPP_WARN(this->get_logger(), "LA COLLISIONE È TRA : ");
                for (const auto &entry : collision_result.contacts)
                    RCLCPP_INFO(this->get_logger(), " - Link '%s' collide con '%s' (%zu contatti)",
                                entry.first.first.c_str(), entry.first.second.c_str(), entry.second.size());
            }
           
            return have_candidate;

    }

    bool testReachability(const geometry_msgs::msg::PoseStamped &pose, const std::string &group_name)
    {
        visualization_msgs::msg::MarkerArray markers;
        moveit_msgs::msg::DisplayRobotState display_msg;

        int id = 0;
        bool all_orientations_reachable = true;
        


            auto req = std::make_shared<GetPositionIK::Request>();
            req->ik_request.group_name = group_name;
            req->ik_request.pose_stamped = pose;
            req->ik_request.ik_link_name = tcp_link_;//left_tcp,right_tcp(group è left_ o right_)
            req->ik_request.avoid_collisions = true;//PRIMA PROVO CON AVOID COLLISIONS TRUE
            req->ik_request.timeout.sec = 1;

            auto future = client_->async_send_request(req);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "Timeout per richiesta IK.");
                
            }

            auto response = future.get();
            bool reachable = (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
            RCLCPP_INFO(this->get_logger(), "Per nodo: (%s) IK response code = %d",this->get_name(), response->error_code.val);

        
            sensor_msgs::msg::JointState Joint_states = response->solution.joint_state;
           
     

           
           bool have_candidate = CheckLinkCollision(reachable, pose);//se ho una collisione ritorna true
           if(!have_candidate){//se non ho collisioni
            display_msg.state.joint_state = Joint_states;
           
            display_state_pub_->publish(display_msg);
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
        return reachable;

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
