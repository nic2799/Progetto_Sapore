#include "SaveYaml.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <rclcpp/logging.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <iostream>

namespace SaveYaml {

void savePlanToYAML(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                    const geometry_msgs::msg::PoseStamped& target_pose,
                    const std::string& plan_name,
                    int& plan_counter_,
                    const std::string& plans_output_file_,
                    double gripper_state)
{
    YAML::Node root;
    
    // 1. Carica il file YAML esistente (se esiste)
    if (rcpputils::fs::exists(plans_output_file_)) {
        try {
            root = YAML::LoadFile(plans_output_file_);
        } catch (const YAML::ParserException& e) {
            std::cerr << "Errore parsing YAML:  Creo nuovo root." << std::endl;
            root = YAML::Node(); 
        }
    }
    
    if (!root["plans"]) {
        root["plans"] = YAML::Node(YAML::NodeType::Sequence);
    }
    YAML::Node plans = root["plans"];

    // 2. Cerca se esiste già un piano con lo stesso ID
    size_t plan_index = plans.size();
    for (size_t i = 0; i < plans.size(); ++i) {
        if (plans[i]["id"] && plans[i]["id"].as<int>() == plan_counter_) {
            plan_index = i;
            break;
        }
    }

    YAML::Node current_plan;
    if (plan_index < plans.size()) {
        // Piano già esistente: recupera
        current_plan = plans[plan_index];
    } else {
        // Nuovo piano: inizializza
        current_plan["id"] = plan_counter_;
        current_plan["target_pose"]["position"]["x"] = target_pose.pose.position.x;
        current_plan["target_pose"]["position"]["y"] = target_pose.pose.position.y;
        current_plan["target_pose"]["position"]["z"] = target_pose.pose.position.z;
        current_plan["target_pose"]["orientation"]["x"] = target_pose.pose.orientation.x;
        current_plan["target_pose"]["orientation"]["y"] = target_pose.pose.orientation.y;
        current_plan["target_pose"]["orientation"]["z"] = target_pose.pose.orientation.z;
        current_plan["target_pose"]["orientation"]["w"] = target_pose.pose.orientation.w;
    }


    if (!current_plan["trajectories"]) {
        current_plan["trajectories"] = YAML::Node(YAML::NodeType::Sequence);
    }

    // 3. COSTRUZIONE DEL NUOVO NODO TRAIETTORIA
    YAML::Node new_traj;
    new_traj["name"] = plan_name;
    const auto& traj = plan.trajectory.joint_trajectory;
    
    for (const auto& j : traj.joint_names)
        new_traj["joint_names"].push_back(j);
        
    new_traj["waypoints"] = traj.points.size();

    for (size_t i = 0; i < traj.points.size(); ++i) {
        YAML::Node wp;
        wp["index"] = i;
        
        for (auto v : traj.points[i].positions) wp["positions"].push_back(v);
        for (auto v : traj.points[i].velocities) wp["velocities"].push_back(v);
        for (auto v : traj.points[i].accelerations) wp["accelerations"].push_back(v);
        
        double tfs = traj.points[i].time_from_start.sec + traj.points[i].time_from_start.nanosec / 1e9;
        wp["time_from_start"] = tfs;
        
        // Aggiungi stato gripper
        wp["gripper"] = gripper_state;

        new_traj["points"].push_back(wp);
    }

    // 4. SOVRASCRITTURA O AGGIUNTA (FIX)
    YAML::Node trajectories = current_plan["trajectories"];
    bool overwritten = false;

    for (size_t i = 0; i < trajectories.size(); ++i) {
        // Controlla se esiste già una traiettoria con questo nome (es. "approach")
        if (trajectories[i]["name"] && trajectories[i]["name"].as<std::string>() == plan_name) {
            // TROVATO! Sovrascriviamo il nodo a questo indice
            trajectories[i] = new_traj;
            overwritten = true;
            std::cout << " Traiettoria '" << plan_name << "' esistente sovrascritta nel piano " << plan_counter_ << "." << std::endl;
            break;
        }
    }

    if (!overwritten) {
        // Se non esiste, aggiungi in coda
        trajectories.push_back(new_traj);
        std::cout << " Nuova traiettoria '" << plan_name << "' aggiunta al piano " << plan_counter_ << "." << std::endl;
    }

    current_plan["trajectories"] = trajectories;

    // 5. Salva tutto nel nodo principale
    if (plan_index < plans.size()) {
        plans[plan_index] = current_plan;
    } else {
        plans.push_back(current_plan);
    }

    // 6. Scrivi su file
    std::ofstream out(plans_output_file_);
    out << root;
    out.close();
}

} // namespace SaveYaml