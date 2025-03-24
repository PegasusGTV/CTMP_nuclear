#include <ctmp_single/ctmp_preprocess_planner.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/collision_detection/collision_tools.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cmath>

void writeYamlFile(const std::string& filename, const std::vector<double>& discretization) {
    YAML::Node root;
    
    std::vector<std::string> keys = {"delta_x", "delta_y", "delta_z", "delta_roll", "delta_pitch", "delta_yaw"};
    
    for (size_t i = 0; i < keys.size(); ++i) {
        YAML::Node primitive;
        
        // Force inline sequence style
        YAML::Node sequence1, sequence2;
        sequence1 = std::vector<double>{0, 0, 0, 0, 0, 0};
        sequence2 = std::vector<double>{0, 0, 0, 0, 0, 0};
        sequence2[i] = (i >= 3) ? std::round(discretization[i] * (360.0 / M_PI)) : discretization[i];

        sequence1.SetStyle(YAML::EmitterStyle::Flow);
        sequence2.SetStyle(YAML::EmitterStyle::Flow);
        
        primitive["mprim_sequence"].push_back(sequence1);
        primitive["mprim_sequence"].push_back(sequence2);

        // Ensure costs are inline too
        YAML::Node costs = YAML::Node(std::vector<int>{1000, 0});
        costs.SetStyle(YAML::EmitterStyle::Flow);

        primitive["mprim_sequence_transition_costs"] = costs;
        primitive["generate_negative"] = true;
        
        root["short_primitives"][keys[i]] = primitive;
    }
    
    std::ofstream fout(filename);
    fout << root;
    fout.close();
}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "ctmp_single_preprocess");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(8);
    spinner.start();
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // std::map<std::string, moveit_msgs::CollisionObject> collision_objects;
    // collision_objects = planning_scene_interface.getObjects();
    // // Iterate through the objects and print their names
    // for (const auto& object : collision_objects)
    // {
    //     ROS_INFO("Collision object: %s", object.first.c_str());
    // }

    // // Assuming `group` is a valid MoveGroupInterface
    // moveit::planning_interface::MoveGroupInterface group("manipulator_1");

    // // Get the robot model
    // std::shared_ptr<const moveit::core::RobotModel> robot_model = group.getRobotModel();

    // // Access the collision matrix
    // collision_detection::CollisionMatrix collision_matrix = robot_model->getLinkModelGroup("manipulator_1")->getCollisionMatrix();

    // // Check if collision checking is enabled between arm1 and arm2
    // bool is_collision_enabled = collision_matrix["manipulator_1"]["manipulator_2"];

    // if (is_collision_enabled)
    // {
    //     ROS_INFO("Collision checking between arm1 and arm2 is enabled.");
    // }
    // else
    // {
    //     ROS_WARN("Collision checking between arm1 and arm2 is NOT enabled.");
    // }

    // ros::param::set("/manipulator/regions/place_region/min_limits",
    //             std::vector<double>{0.90, -0.07, 0.46, 0.0, 0.0, 0.0});
    
    // ros::param::set("/manipulator/regions/place_region/max_limits",
    //             std::vector<double>{0.90, 0.45, 0.46, 0.0, 0.0, 0.0});

    std::string group_name = "manipulator_1";

    double discret = 1; //arg 1
    bool query_mode; //arg 2 - default = false
    std::string planner_type = "RRTConnect";

    ros::param::get("/query_mode", query_mode);

    std::string region_type;
    ros::param::get("/region_type", region_type);

    // if (argc == 0){
    //     ROS_INFO_STREAM(BOLDMAGENTA << "No arguments given: using default values");
    //     ROS_INFO_STREAM("<group_name(string)>");
    //     ROS_INFO_STREAM("Using default values: manipulator 1 0" << RESET);
    // }
    // else if (argc == 2){
    //     group_name == argv[1];
    // }
    // else if (argc == 3){
    //     group_name == argv[1];
    //     query_mode = std::stoi(argv[2]);
    // }
    // else{
    //     ROS_INFO_STREAM(BOLDMAGENTA << "No arguments given: using default values");
    //     ROS_INFO_STREAM("<group_name(string)> <discretization(int)> <save_experience(bool int)>" );
    //     ROS_INFO_STREAM("Using default values: manipulator_1 1 0" << RESET);
    // }

    auto full_path = ros::package::getPath("manipulation_planning");

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    ims::MoveitInterface scene_interface(group_name);

    // add collision object to the scene - box
    // std::vector<BoxWorldObject> box_object = {
    //     BoxWorldObject(Eigen::Vector3d(0.92, 0.19, 0.18), Eigen::Vector3d(0.5, 0.8, 0.35), "place_surface")
    // };

    // // add the box as collision object to the scene
    // std::vector<moveit_msgs::CollisionObject> collision_objects;

    // scene_interface.addBoxToScene(box_object, collision_objects);

    // std::vector<SphereWorldObject> sphere_object = {
    // //     SphereWorldObject(Eigen::Vector3d(0.8, -0.3, 0.56), 0.1, "pick_location")
    // // };

    // std::vector<moveit_msgs::CollisionObject> point_object;
    // scene_interface.addSpheresToScene(sphere_object, point_object);

    // get the number of joints
    // int num_joints = (int)move_group.getVariableCount();

    // auto full_path = boost::filesystem::path(__FILE__).parent_path().parent_path();
    StateType discretization(6);
    nh.getParam("/"+ group_name + "/regions/" + region_type + "_region/discretization", discretization);
    std::string path_mprim = full_path + "/config/ws_mprim_multires.yaml";
    writeYamlFile(path_mprim, discretization);
    
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // // check for collision
    // planning_scene::PlanningScenePtr planning_scene;
    // planning_scene.reset(new planning_scene::PlanningScene(move_group.getRobotModel()));
    // collision_detection::CollisionRequest collision_request;
    // collision_detection::CollisionResult collision_result;
    // planning_scene->checkCollision(collision_request, collision_result, *current_state);

    auto df = ims::getDistanceFieldMoveIt();

    ros::Publisher bb_pub = nh.advertise<visualization_msgs::Marker>("bb_marker", 10);
    // get the planning frame
    ims::visualizeBoundingBox(df, bb_pub, move_group.getPlanningFrame());

    // heuristic params for trajectory planning to attractor state - define in that function only
    // passing heuristic value as params to preprocess_planner to generate trajectories from start to attractor state
    // auto* heuristic = new ims::BFSHeuristic(df, group_name);
    // double weight = 100.0;
    // PreprocessPlannerParams params(heuristic, weight);

    ims::ctmpActionType action_type(path_mprim);

    // States are x, y, z, thetax, thetay, thetaz
    // StateType discretization {0.005, 0.005, 0.005, M_PI/360, M_PI/360, M_PI/360};
    

    
    if (discretization.size() == 6)
    {
        ROS_INFO("Discretization loaded successfully");
    }
    else
    {
        ROS_ERROR("Discretization does not have 6 elements");
        return 1;
    }
    


    action_type.Discretization(discretization);
    action_type.setSpaceType(ims::ManipulationType::SpaceType::WorkSpace);

    std::shared_ptr<ims::ctmpActionSpace> action_space = std::make_shared<ims::ctmpActionSpace> (scene_interface, action_type);

    // States are x, y, z, thetax, thetay, thetaz
    StateType start_state {0, 0, 0, 0, 0, 0};
    // move_group.setNamedTarget("ready" + group_name.substr(group_name.size() - 1));
    move_group.setNamedTarget("ready1");
    move_group.setMaxVelocityScalingFactor(1.0);  // Max speed (1.0 means 100%)
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.move();
    // ros::Duration(1).sleep();

    const std::vector<std::string>& joint_names = move_group.getVariableNames();
    std::vector<double> joint_values(joint_names.size());
    for (int i = 0; i < joint_names.size(); i++) {
        joint_values[i] = current_state->getVariablePosition(joint_names[i]);
        ROS_INFO_STREAM("Joint " << joint_names[i] << " is " << joint_values[i]);
    }

    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

    start_state[0] = current_pose.pose.position.x;
    start_state[1] = current_pose.pose.position.y;
    start_state[2] = current_pose.pose.position.z;

    // If using hopf coordinates:
    Eigen::Quaterniond current_pose_eigen;
    tf::quaternionMsgToEigen(current_pose.pose.orientation, current_pose_eigen);

    ims::get_euler_zyx(current_pose_eigen, start_state[5], start_state[4], start_state[3]);
    ims::normalize_euler_zyx(start_state[5], start_state[4], start_state[3]);
    
    // States are x, y, z, thetax, thetay, thetaz
    

    StateType goal_state(6);

    // Get the current object ID from the ROS parameter "curr_object"
    int curr_object;
    if (!nh.getParam("/curr_object", curr_object)) {
        ROS_ERROR("Failed to retrieve 'curr_object' parameter, cannot perform multiple objects assembly. Defaulting to reading from regions.yaml");
        if (nh.getParam("/"+ group_name + "/regions/" + region_type + "_region/goal", goal_state))
        {
            if (goal_state.size() == 6)
            {
                ROS_INFO("Goal state loaded successfully");
            }
            else
            {
                ROS_ERROR("Goal state does not have 6 elements");
                return 1;
            }
        }
        else
        {
            ROS_ERROR("Failed to load goal state from parameter server");
            return 1;
        }
    }
    else
    {
        XmlRpc::XmlRpcValue objects;
        if (!nh.getParam("objects", objects)) {
            ROS_ERROR("Failed to get 'objects' from the parameter server.");
            return 1;
        }
        // Ensure the index is within range
        if (curr_object < 1 || curr_object > objects.size()) {
            ROS_ERROR("Invalid 'curr_object' value: %d. Must be between 1 and %d.", curr_object, objects.size());
            return 1;
        }

        // Adjust for 0-based indexing
        int object_index = curr_object+1; // Index of the object in the objects list (-1 for 0 based indexing, +2 for the two other meshes that are loaded. Total is +1)
        std::string target;
        if (region_type == "pick") {
            target = "pose";
        } else if (region_type == "place") {
            target = "goal";
        } else {
            ROS_ERROR("Invalid region type: %s. Must be 'pick' or 'place'.", region_type.c_str());
            return 1;
        }
        // Access the goal state for the selected object
        if (objects[object_index].hasMember(target)) {
            XmlRpc::XmlRpcValue goal = objects[object_index][target];

            // Get the position and orientation of the goal
            std::vector<double> goal_position(3);
            std::vector<double> goal_orientation(3);
            for (int i = 0; i < 3; ++i) {
                goal_position[i] = static_cast<double>(goal["position"][i]);
                goal_orientation[i] = static_cast<double>(goal["orientation"][i]);
            }

            if (region_type == "pick") 
            {
                goal_position[2] += 0.16; // TODO: This is a hardcoded value, should be read from the object's dimensions
            } // Grasp at the top of the object 

            // Log the goal state (or use it as needed)
            ROS_INFO("Goal state for object %d loaded successfully", curr_object);
            ROS_INFO("Position: [%f, %f, %f]", goal_position[0], goal_position[1], goal_position[2]);
            ROS_INFO("Orientation: [%f, %f, %f]", goal_orientation[0], goal_orientation[1], goal_orientation[2]);

            // Combine position and orientation into your goal state
            for (int i = 0; i < 3; ++i) {
                goal_state[i] = goal_position[i];
                goal_state[i + 3] = goal_orientation[i];
            }
        }
    }
    
    // StateType goal_state = {0.6, 0.0, 1.1, 0, 0, 0};
    // discrtize the goal state
    for (int i = 0; i < 6; i++) {
        goal_state[i] = std::round(goal_state[i] / discretization[i]) * discretization[i];
        start_state[i] = std::round(start_state[i] / discretization[i]) * discretization[i];
    }
    Eigen::Quaterniond start_pose_eigen;
    ims::from_euler_zyx(start_state[5], start_state[4], start_state[3], start_pose_eigen);
    geometry_msgs::Pose pose_check;
    pose_check.position.x = start_state[0]; pose_check.position.y = start_state[1]; pose_check.position.z = start_state[2];
    tf::quaternionEigenToMsg(start_pose_eigen, pose_check.orientation);


    std::cout << "rounded pose " << pose_check.position.x << " " << pose_check.position.y << " " << pose_check.position.z << " "
              << pose_check.orientation.x << " " << pose_check.orientation.y << " " << pose_check.orientation.z << " " << pose_check.orientation.w << std::endl;

    std::cout << "original pose " << current_pose.pose.position.x << " " << current_pose.pose.position.y << " " << current_pose.pose.position.z << " "
              << current_pose.pose.orientation.x << " " << current_pose.pose.orientation.y << " " << current_pose.pose.orientation.z << " " << current_pose.pose.orientation.w << std::endl;

    // check if the inverse kinematics solution exists for the current pose and check if the solution is equal to the current joint state
    std::vector<double> current_joint_state = move_group.getCurrentJointValues();
    std::vector<double> ik_solution;

    if (!scene_interface.calculateIK(pose_check, ik_solution)) {
        std::cout << "No IK solution for the current pose" << std::endl;
        return 0;
    }
    else {
        ims::rad2deg(ik_solution); ims::rad2deg(current_joint_state);
        std::cout << "IK solution for the current pose" << std::endl;
        for (int i = 0; i < ik_solution.size(); i++) {
            std::cout << "joint " << i << " " << ik_solution[i] << " " << current_joint_state[i] << std::endl;
        }
    }
    std::cout << "start state " << start_state[0] << " " << start_state[1] << " " << start_state[2] << " "
              << start_state[3] << " " << start_state[4] << " " << start_state[5] << std::endl;
    std::cout << "goal state " << goal_state[0] << " " << goal_state[1] << " " << goal_state[2] << " "
              << goal_state[3] << " " << goal_state[4] << " " << goal_state[5] << std::endl;

    
    /// heuristic params for constructing neighborhood - don't forget that this heuristic wont work for joint angles
    auto* heuristic_neighborhood = new ims::SE3HeuristicRPY;
    ims::BestFirstSearchParams params_neighborhood(heuristic_neighborhood);

    PreprocessPlannerParams params(query_mode, planner_type);
    /// TODO: pass ims bfsheuristic params as argument to planner function
    PreprocessPlanner planner(params);
    auto start = std::chrono::high_resolution_clock::now();
    if (!planner.initializePlanner(action_space, start_state, goal_state, params_neighborhood)){
        ROS_INFO_STREAM(RED << "Preprocess Planner initialization failed" << RESET);
        return 0;
    }

    std::vector<StateType> path_;
    if (!planner.plan(path_)) {
        ROS_INFO_STREAM(RED << "No path found" << RESET);
        return 0;
    }
    else {
        ROS_INFO_STREAM(GREEN << "Path found" << RESET);
    }

    // Print nicely the path
    int counter = 0;
    for (auto& state : path_) {
        std::cout << "State: " << counter++ << ": ";
        for (auto& val : state) {
            std::cout << val << ", ";
        }
        std::cout << std::endl;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    
    std::string log_filename = ros::package::getPath("ctmp_single") + "/results/planning_times_with_init.log";

    std::ofstream log_file(log_filename, std::ios::app);
    if (log_file.is_open()) {
        log_file << "Time: " << duration.count() << " s, "
                << "Object: " << curr_object << ", "
                << "Region: " << region_type << "\n";
        log_file.close();
    } else {
        ROS_WARN_STREAM("Could not open log file to write planning time.");
    }

    ROS_INFO_STREAM(GREEN << "Path found in " << duration.count() << " seconds" << RESET);



    std::vector<StateType> traj;
    for (auto& state : path_) {
        traj.push_back(state);
    }
    moveit_msgs::RobotTrajectory trajectory;
    current_joint_state = move_group.getCurrentJointValues();
    move_group.setMaxVelocityScalingFactor(0.4);
    move_group.setMaxAccelerationScalingFactor(0.4);
    path_.insert(path_.begin(), current_joint_state);
    // I've changed something here, I am not sure if it is right
    ims::profileTrajectory(path_.at(0),
                    path_.back(),
                    path_,
                    move_group,
                    trajectory);

    move_group.execute(trajectory);
    if (region_type == "pick" && query_mode) 
    {
    ROS_INFO("Region type is 'pick'. Attaching object to arm1_Tool0.");
    
    // Object ID and the link to attach to
    std::string object_id = "object" + std::to_string(curr_object);   // TODO: Load this in from rosparam in the future 
    std::string link_name = "arm_1tool0"; // Link to attach to
    
    // Attach the object to the link using MoveGroupInterface
    move_group.attachObject(object_id, link_name);  // Attach object to the link
    ROS_INFO("Object attached to %s", link_name.c_str());
    }
    if (region_type == "place" && query_mode) 
    {
    ROS_INFO("Region type is 'pick'. Attaching object to arm1_Tool0.");
    
    // Object ID and the link to attach to
    std::string object_id = "object" + std::to_string(curr_object);  
    std::string link_name = "arm_1tool0"; // Link to attach to
    
    // Attach the object to the link using MoveGroupInterface
    move_group.detachObject(object_id);  // Attach object to the link
    ROS_INFO("Object detached from %s", link_name.c_str());
    }

return 0;

    return 0;
}
