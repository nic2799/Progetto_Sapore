#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/object_color.hpp> 
#include <std_msgs/msg/color_rgba.hpp>     

#include <moveit_msgs/msg/display_trajectory.hpp> 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/moveit_error_code.h>

#include <iostream>  
#include <string>    
#include <chrono>    
#include <atomic>   
#include <std_srvs/srv/trigger.hpp>  
#include <fstream>   // Per salvare file
#include <iomanip>  
#include <ctime>  




using moveit_msgs::srv::GetPositionIK;
using namespace std::chrono_literals;

class ReachabilityIKNode : public rclcpp::Node
{
public:

    ReachabilityIKNode() : Node("reachability_ik_node"), continue_flag_(false)
    {
        client_ = this->create_client<GetPositionIK>("/compute_ik");
        RCLCPP_INFO(this->get_logger(), "Attendo servizio /compute_ik...");
        client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "Servizio /compute_ik disponibile!");
        
        display_state_pub_ = this->create_publisher<moveit_msgs::msg::DisplayRobotState>("/display_robot_state", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        display_traj_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);

        // Servizio per avanzare alla posa successiva
        continue_service_ = this->create_service<std_srvs::srv::Trigger>(
            "continue_test",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                this->continue_flag_ = true;
                response->success = true;
                response->message = "Continuo con la posa successiva";
                RCLCPP_INFO(this->get_logger(), " Ricevuto comando di continuazione");
            });

        this->declare_parameter("xmin", -1.0);
        this->declare_parameter("xmax", 1.0);
        this->declare_parameter("ymin", -1.0);
        this->declare_parameter("ymax", 1.0);
        this->declare_parameter("zmin", 0.0);
        this->declare_parameter("zmax", 1.0);
        this->declare_parameter("roll", 0.0);
        this->declare_parameter("pitch", 0.0);
        this->declare_parameter("yaw", 0.0);
        this->declare_parameter("step", 0.1);
        this->declare_parameter("group_name", "right_arm");
        this->declare_parameter("name_file", "plans"); // Nome file per salvare i piani

        // Lettura dei parametri effettivi
        xmin_ = this->get_parameter("xmin").as_double();
        xmax_ = this->get_parameter("xmax").as_double();
        ymin_ = this->get_parameter("ymin").as_double();
        ymax_ = this->get_parameter("ymax").as_double();
        zmin_ = this->get_parameter("zmin").as_double();
        zmax_ = this->get_parameter("zmax").as_double();
        roll_ = this->get_parameter("roll").as_double();
        pitch_ = this->get_parameter("pitch").as_double();
        yaw_ = this->get_parameter("yaw").as_double();
        step_ = this->get_parameter("step").as_double();
        group = this->get_parameter("group_name").as_string();
        group_ = group + "arm";
        tcp_link_ = group + "tcp";
        obstacle_name =this->get_parameter("name_file").as_string();
        
        // Genera nome file YAML con timestamp
      
        plans_output_file_ =obstacle_name + "_plans_.yaml" ;
        
        RCLCPP_INFO(this->get_logger(), " I piani saranno salvati in: %s", plans_output_file_.c_str());

    }

    void start()
{   /*Definiamo il psm serve per monitorare scena e tenere sotto controllo lo stato del robot*/
    psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(this->shared_from_this(), "robot_description");
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), group_);

    if (!psm->getPlanningScene()) {
        RCLCPP_ERROR(this->get_logger(), "Errore: impossibile creare PlanningSceneMonitor.");
        return;
    }

    psm->requestPlanningSceneState("get_planning_scene");//aggiorna la scena di pianificazione
    planning_scene = psm->getPlanningScene();//salva la scena di pianificazione come membro della classe

    auto ostacolo = generateGrid(xmin_, xmax_, ymin_, ymax_, zmin_, zmax_, roll_, pitch_, yaw_, step_);

    RCLCPP_INFO(this->get_logger(), "Avvio test di raggiungibilità...");
    RCLCPP_INFO(this->get_logger(), "Totale pose da testare: %zu", ostacolo.size());
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_WARN(this->get_logger(), " Se continue_flag settato a false per avanzare alla posa successiva, esegui in un altro terminale:");
    RCLCPP_WARN(this->get_logger(), "   ros2 service call /%s/continue_test std_srvs/srv/Trigger", this->get_name());
    RCLCPP_INFO(this->get_logger(), "");

    int pose_counter = 0;
    for (const auto &pose : ostacolo) {
        pose_counter++;
        // PAUSA INTERATTIVA - ASPETTA CHIAMATA SERVIZIO
       
        RCLCPP_INFO(this->get_logger(), "\n---------------------------------------------");
        RCLCPP_INFO(this->get_logger(), " POSA %d/%zu", pose_counter, ostacolo.size());
        RCLCPP_INFO(this->get_logger(), "   Posizione: (%.3f, %.3f, %.3f)", 
                    pose.pose.position.x, 
                    pose.pose.position.y, 
                    pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
        
        // Reset flag
        continue_flag_ = true;//setta a false nel caso in cui si voglia fare una pausa prima di ogni test
        
        RCLCPP_WARN(this->get_logger(), "\n IN PAUSA - Esegui: ros2 service call /%s/continue_test std_srvs/srv/Trigger\n", 
                    this->get_name());
        
        // Attendi che il servizio venga chiamato
        while (!continue_flag_ && rclcpp::ok()) {
            rclcpp::spin_some(this->shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!rclcpp::ok()) break;
        
        RCLCPP_INFO(this->get_logger(), " Inizio test...\n");

        int raggiunto = testReachability(pose, group_);
        int final_info = raggiunto; // -1 non raggiungibile, 0 collisione, 1 raggiungibile

        if (raggiunto == 0 || raggiunto == -1) {
            RCLCPP_INFO(this->get_logger(), "Non raggiungibile — verifico con altri orientamenti...\n");

            tf2::Quaternion q_orig;
            tf2::fromMsg(pose.pose.orientation, q_orig);
            double Delta_max = 30 * M_PI / 180;  // ±45° o 20

            bool trovato = false;

            for (double delta : {+Delta_max, -Delta_max}) {
                tf2::Quaternion q_rot;
                q_rot.setRPY(delta, 0, 0);  // ruota solo roll (puoi cambiare asse)
                tf2::Quaternion q_new = q_orig * q_rot;
                q_new.normalize();

                geometry_msgs::msg::PoseStamped new_pose = pose;
                new_pose.pose.orientation = tf2::toMsg(q_new);

                int risultato = testReachability(new_pose, group_);

                if (risultato == 1) {
                    RCLCPP_INFO(this->get_logger(), "Raggiungibile con orientamento modificato.");
                    final_info = 1;
                    trovato = true;
                    break;
                } else if (risultato == 0) {
                    RCLCPP_INFO(this->get_logger(), "Collisione anche con orientamento variato.");
                    final_info = 0;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Non raggiungibile nemmeno con orientamento variato.");
                    final_info = -1;
                }
            }

          
        } else {
            RCLCPP_INFO(this->get_logger(), "Punto raggiungibile senza collisioni.\n");
        }

      
        if (final_info == 1) {
            
             // Pouring con rotazione di 120° (verificato rispetto ai limiti del giunto)
             bool pour_ok = testPlannerPouring(pose, group_, 120.0 * M_PI/180.0);
             if (pour_ok) {
       
                InsertMarker(pose, 2); 
                RCLCPP_INFO(this->get_logger(), " \n Il pouring è possibile da questa posa. \n");}
                else{
                    InsertMarker(pose, final_info);
                }
       
        }else{InsertMarker(pose, final_info);}
    }
    
    RCLCPP_INFO(this->get_logger(), "\n Test completato su tutte le %zu pose!", ostacolo.size());
}


    void resetMarkers()//serve per resettare all'avvio i marker
    {
        visualization_msgs::msg::MarkerArray delete_msg;
        visualization_msgs::msg::Marker m;
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_msg.markers.push_back(m);

        // Pubblica subito DELETEALL
        marker_pub_->publish(delete_msg);
    }

    private:
    double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_,roll_, pitch_,yaw_,step_;
    std::string group;
    std::string obstacle_name;

    std::vector<geometry_msgs::msg::PoseStamped> generateGrid(double xmin, double xmax, double ymin, double ymax,
                                                              double zmin, double zmax, double roll, double pitch, double yaw, double step)
    {
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion ros_quaternion;
        tf2::convert(quaternion, ros_quaternion);

        double r, p, y;
            tf2::Matrix3x3(quaternion).getRPY(r, p, y);
          //  RCLCPP_INFO(this->get_logger(), "RPY: roll=%f pitch=%f yaw=%f", r, p, y);


        std::vector<geometry_msgs::msg::PoseStamped> grid_points;
        for (double x = xmin; x <= xmax; x += step)
            for (double y = ymin; y <= ymax; y += step)
                for (double z = zmin; z <= zmax; z += step)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "world";
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = z;
                    pose.pose.orientation = ros_quaternion;
                    grid_points.push_back(pose);
                }

      //  RCLCPP_INFO(this->get_logger(), "Generati %zu punti nella griglia.", grid_points.size());
        
        return grid_points;
    }

    bool CheckLinkCollision(bool reachable, const geometry_msgs::msg::PoseStamped &pose){
             bool have_candidate = false;
             sensor_msgs::msg::JointState candidate_js;
             moveit_msgs::msg::DisplayRobotState display_msg;
            if (!reachable)
            {               
                moveit::core::RobotState current_state = planning_scene->getCurrentState();
                const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(group_);
                bool found_ik = current_state.setFromIK(joint_model_group, pose.pose,2);
                if (found_ik)//dunque il punto è raggiungibile ma in collisione
                {
                    // Estraggo i joint states
                    current_state.copyJointGroupPositions(
                    joint_model_group, candidate_js.position);
                    candidate_js.name = joint_model_group->getVariableNames();
                    have_candidate = true;
             
                    double r, p, y;
                    // convertiamo geometry_msgs::msg::Quaternion -> tf2::Quaternion
                    tf2::Quaternion q;
                    tf2::fromMsg(pose.pose.orientation, q);
                    tf2::Matrix3x3(q).getRPY(r, p, y);
                }
 
            }
        
            if (have_candidate)//SE TROVO POSA RAGGIUNGIBILE E QUINDI IN COLLISIONE DEVO MOSTRARE I LINK IN COLLISIONE
            {
                

            moveit::core::RobotState current_state = planning_scene->getCurrentState();
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

   
  bool testPlannerPouring(const geometry_msgs::msg::PoseStamped &base_pose,
                        const std::string &group_name,
                        double final_z_rotation_rad = 120.0 * M_PI / 180.0)
{
    using MoveItErrorCode = moveit::core::MoveItErrorCode;
    RCLCPP_INFO(this->get_logger(), "\n === TEST POURING - PIANIFICAZIONE MOVIMENTO ===");

    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(100.0);

    move_group_->setGoalPositionTolerance(0.03);  // ±2cm sulla posizione
    move_group_->setGoalOrientationTolerance(0.2); // ±5.7° di tolleranza sull'orientamento
    move_group_->setNumPlanningAttempts(60);
    move_group_->setEndEffectorLink(tcp_link_);
    move_group_->setStartStateToCurrentState();
    //comando per ritentare
   // move_group_->setPlanningAttempts(60);

  
    
    // PIANO 1: verso la posa base
   
    RCLCPP_INFO(this->get_logger(), "\n PIANO 1: Verso base_pose...");
    move_group_->setPoseTarget(base_pose.pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    MoveItErrorCode result1 = move_group_->plan(plan1);

    if (result1 != MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), " Piano 1 fallito (codice %d)", result1.val);
       
        move_group_->clearPoseTargets();
        return false;
    }

    RCLCPP_INFO(this->get_logger(), " Piano 1 riuscito (%zu waypoints)",
                plan1.trajectory.joint_trajectory.points.size());
    savePlanToYAML(plan1, base_pose, "plan_to_base_pose");
    move_group_->clearPoseTargets();



     if(obstacle_name!="Rastrelliera"){
   
    // PIANO 2: rotazione finale per pouring
   
    RCLCPP_INFO(this->get_logger(), "\n PIANO 2: Pouring (rotazione A7 di %.1f°)...", 
                final_z_rotation_rad * 180.0 / M_PI);
    
    //  Recupera stato finale del primo piano
    const auto &joint_names = plan1.trajectory.joint_trajectory.joint_names;
    const auto &last_point = plan1.trajectory.joint_trajectory.points.back();
    
    moveit::core::RobotState start_state(move_group_->getRobotModel());
    start_state.setVariablePositions(joint_names, last_point.positions);
    start_state.update();

    //  Imposta stato finale del primo piano come start per il secondo
    move_group_->setStartState(start_state); 
    
    //  Tolleranze STRETTE per movimento rotazionale preciso
    move_group_->setGoalJointTolerance(0.01);  
    
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

    //  Imposta SOLO target sui giunti 
    move_group_->setJointValueTarget(joint_positions);




    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    MoveItErrorCode result2 = move_group_->plan(plan2);

    if (result2 != MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), " Piano 2 fallito (codice %d)", result2.val);
        
        move_group_->clearPoseTargets();
        return false;
    }

    //  Verifica rotazione effettivamente raggiunta
    const auto& final_point = plan2.trajectory.joint_trajectory.points.back();
    double posizione_finale_a7_raggiunta = final_point.positions[joint_a7_index];
    double rotazione_effettiva = posizione_finale_a7_raggiunta - posizione_iniziale_a7;
    
    RCLCPP_INFO(this->get_logger(), "Piano 2 riuscito (%zu waypoints)",
                plan2.trajectory.joint_trajectory.points.size());
    RCLCPP_INFO(this->get_logger(), "  A7 finale raggiunto: %.3f rad (%.1f°)",
                posizione_finale_a7_raggiunta, posizione_finale_a7_raggiunta * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  Rotazione effettiva: %.3f rad (%.1f°)",
                rotazione_effettiva, rotazione_effettiva * 180.0 / M_PI);
   
    
    
    savePlanToYAML(plan2, base_pose, "plan_pouring_pose");
    move_group_->clearPoseTargets();

    RCLCPP_INFO(this->get_logger(), "\n === POURING POSSIBILE ===\n");
}
    return true;

}



private:
    
    // METODO: Salva piano in formato YAML
   
    void savePlanToYAML(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                        const geometry_msgs::msg::PoseStamped& target_pose,
                        const std::string& plan_name)
    {
        plan_counter_++;
        
        std::ofstream file;
        // Prima volta: crea file e scrivi header
        if (plan_counter_ == 1) {
            file.open(plans_output_file_, std::ios::out);
            auto now = std::time(nullptr);
            file << "# Piano di Traiettorie MoveIt\n";
            file << "# Generato: " << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << "\n";
            file << "plans:\n";
        } else {
            // Aggiungi al file esistente
            file.open(plans_output_file_, std::ios::app);
        }
        
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Impossibile aprire file: %s", plans_output_file_.c_str());
            return;
        }
        
        // Scrivi dati del piano
        file << "  - id: " << plan_counter_ << "\n";
        file << "    name: \"" << plan_name << "\"\n";
        file << "    target_pose:\n";
        file << "      position: {x: " << target_pose.pose.position.x 
             << ", y: " << target_pose.pose.position.y 
             << ", z: " << target_pose.pose.position.z << "}\n";
        file << "      orientation: {x: " << target_pose.pose.orientation.x 
             << ", y: " << target_pose.pose.orientation.y 
             << ", z: " << target_pose.pose.orientation.z 
             << ", w: " << target_pose.pose.orientation.w << "}\n";
        
        // Traiettoria
        const auto& traj = plan.trajectory.joint_trajectory;//Prendiamo la traiettoria del piano che contiene : joint_names e points(waypoints)
        file << "    trajectory:\n";
        file << "      joint_names: [";
        for (size_t i = 0; i < traj.joint_names.size(); ++i) {
            file << "\"" << traj.joint_names[i] << "\"";//backslash per le doppie virgolette
            if (i < traj.joint_names.size() - 1) file << ", ";
        }
        file << "]\n";
        
        file << "      waypoints: " << traj.points.size() << "\n";
        file << "      points:\n";


        //Salviamo i waypoint
        for (size_t idx = 0; idx < traj.points.size(); ++idx) {
            const auto& point = traj.points[idx];
            file << "        - index: " << idx << "\n";
            file << "          positions: [";
            for (size_t i = 0; i < point.positions.size(); ++i) {
                file << std::fixed << std::setprecision(6) << point.positions[i];//std::fixed per notazione fissa, std::setprecision per 6 cifre decimali
                if (i < point.positions.size() - 1) file << ", ";//aggiunge la virgola tra i joint_names fino al penultimo
            }
            file << "]\n";
            
            // Aggiungi anche velocità e accelerazioni se presenti
            if (!point.velocities.empty()) {
                file << "          velocities: [";
                for (size_t i = 0; i < point.velocities.size(); ++i) {
                    file << std::fixed << std::setprecision(6) << point.velocities[i];
                    if (i < point.velocities.size() - 1) file << ", ";
                }
                file << "]\n";
            }
            
            if (!point.accelerations.empty()) {
                file << "          accelerations: [";
                for (size_t i = 0; i < point.accelerations.size(); ++i) {
                    file << std::fixed << std::setprecision(6) << point.accelerations[i];
                    if (i < point.accelerations.size() - 1) file << ", ";
                }
                file << "]\n";
            }
            
            file << "          time_from_start: " << point.time_from_start.sec 
                 << "." << std::setfill('0') << std::setw(9) << point.time_from_start.nanosec << "\n";
        }
        
        file << "\n";
        file.close();
        
        RCLCPP_INFO(this->get_logger(), "💾 Piano %d salvato (%zu waypoints) in %s", 
                    plan_counter_, traj.points.size(), plans_output_file_.c_str());
    }
    
  
    
   



    int testReachability(const geometry_msgs::msg::PoseStamped &pose, const std::string &group_name)
    {
        bool all_orientations_reachable = true;
        bool reachable = false;
        moveit_msgs::msg::DisplayRobotState display_msg;   

            // Ottieni lo stato corrente del robot
            moveit::core::RobotState current_state = psm->getPlanningScene()->getCurrentState();

            // Converti in messaggio ROS
            moveit_msgs::msg::RobotState robot_state_msg;
            moveit::core::robotStateToRobotStateMsg(current_state, robot_state_msg);

            // Assegna al campo robot_state della richiesta
            // Indica che è uno stato differenziale

            auto req = std::make_shared<GetPositionIK::Request>();
            req->ik_request.group_name = group_name;
            req->ik_request.robot_state = robot_state_msg;
            req->ik_request.robot_state.is_diff = true;
            /*Se is_diff = false (valore predefinito):
            Il messaggio contiene lo stato completo del robot — ogni giunto deve avere nome e posizione.
            MoveIt sostituisce completamente il suo stato interno con quello del messaggio.

            Se is_diff = true:
            Il messaggio contiene solo le parti modificate rispetto allo stato attuale (oppure anche niente).
            MoveIt non azzera tutto, ma aggiorna solo i valori presenti nel messaggio, lasciando invariati gli altri.*/
            req->ik_request.pose_stamped = pose;
            req->ik_request.ik_link_name = tcp_link_;//left_tcp,right_tcp(group è left_ o right_)
            req->ik_request.avoid_collisions = true;//PRIMA PROVO CON AVOID COLLISIONS TRUE
            req->ik_request.timeout.sec = 1;

            auto future = client_->async_send_request(req);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=  rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "Timeout per richiesta IK.");
            }

            auto response = future.get();
            reachable = (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
            RCLCPP_INFO(this->get_logger(), "Per nodo: (%s) IK response code = %d",this->get_name(), response->error_code.val);
            auto Joint_states = response->solution.joint_state;

            bool have_candidate = CheckLinkCollision(reachable, pose);//se ho una collisione ritorna true
            if(!have_candidate){//se non genera la collisione
                display_msg.state.joint_state = Joint_states;
            display_state_pub_->publish(display_msg);
        }

            
        if(reachable){
            return 1;//se è raggiungibile e privo di collisioni
        } else if(have_candidate){
            return 0;//se si genera la collisione
        } else {
            return -1;//se non è raggiungibile ovvero non esiste soluzione IK
        }
            
    }


    void InsertMarker(const geometry_msgs::msg::PoseStamped &pose, int info)
    {
        visualization_msgs::msg::Marker marker;
        visualization_msgs::msg::MarkerArray markers;

        marker.header.frame_id = pose.header.frame_id;
        marker.header.stamp = this->now();
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
        marker.pose = pose.pose;
        marker.color.a = 0.8; // Trasparenza
        
        // Legenda colori:
        // -1 = Rosso: non raggiungibile (no IK solution)
        //  0 = Blu: raggiungibile ma in collisione
        //  1 = Verde: raggiungibile senza collisioni (ma pouring non testato/fallito)
        //  2 = Ciano: raggiungibile E pouring possibile
        if (info == -1) {
            // Rosso: non raggiungibile
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (info == 0) {
            // Blu: in collisione
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        } else if (info == 1) {
            // Verde: raggiungibile
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (info == 2) {
            // Ciano (azzurro): pouring possibile
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }
        
        marker.ns = this->get_name();
        markers.markers.push_back(marker);
        marker_pub_->publish(markers);
        RCLCPP_INFO(this->get_logger(), "Pubblicato marker (info=%d) su RViz.", info);
    }
    rclcpp::Client<GetPositionIK>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr display_state_pub_;
    std::string group_;
    planning_scene::PlanningScenePtr planning_scene;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm;


    std::string arm_side_;
    std::string tcp_link_;
    int id = 0;
    
    // Servizio e flag per modalità interattiva
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr continue_service_;
    std::atomic<bool> continue_flag_;
    
    // Salvataggio piani
    int plan_counter_ = 0;
    std::string plans_output_file_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachabilityIKNode>();
  
    node->resetMarkers();
    
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
