#include <memory>
#include <ctmp_single/ctmp_preprocess_planner.hpp>
#include <ctmp_single/ctmp_preprocess_main.hpp>

PreprocessPlanner::PreprocessPlanner(const PreprocessPlannerParams &params_ctmp) : params_ctmp_(params_ctmp) {}

bool PreprocessPlanner::initializePlanner(const std::shared_ptr<ims::ctmpActionSpace> &actionSpacePtr,
                               const StateType &start,
                               const StateType &goal,
                               const ims::BestFirstSearchParams &params){

    std::cout << "is it working?" << params_ctmp_.planner_type << std::endl;
    task_space_ = actionSpacePtr;
    start_ = start;
    goal_ = goal;
    arm_name_ = task_space_->getPlanningGroupName();

    ROS_INFO("Preprocess Planner Initialized");
    preprocess_main_ = std::make_shared<ims::preprocessMain>(params);
    try{
        preprocess_main_->initializePlanner(actionSpacePtr, start, goal);
    }
    catch(std::exception &e){
        ROS_ERROR("Failed to initialize Preprocess Main Planner");
        return false;
    }
    
    // DOUBT: STILL CONFUSED WITH WHAT IS HAPPENING HERE
    //define pick and place regions
    std::string region_type;
    ros::param::get("/region_type", region_type);
    std::string current_dir = ros::package::getPath("ctmp_single");
    read_write_dir_ = current_dir + "/data/";
    read_write_path_ = read_write_dir_ + arm_name_ + "_" + region_type + ".dat";
    nh = ros::NodeHandle();
    m_nh = ros::NodeHandle(arm_name_);

    ROS_INFO("Read write path: %s", read_write_path_.c_str());
    // get goal region from the file, defined by the user
    if (regions_.empty()){
        ROS_INFO("there are no regions, reading regions");
        ReadRegions();
        task_space_->PassRegions(&regions_, &iregions_);
    }

    return true;
}

bool PreprocessPlanner::plan(std::vector<StateType> &path){
    if (!params_ctmp_.query_mode){
        ROS_INFO("Preprocessing Goal Region");
        int start_ind = task_space_->getOrCreateRobotState(start_);
        // passing start state which is in cspace
        PreProcess(task_space_->getRobotState(start_ind)->state_mapped);
        return false; // indicates that preprocessing has been finished
    }
    else{
        ROS_INFO("Querying");
        Query(path, "");
        if (path.empty()){
            ROS_WARN("No path found");
            return false;
        }
        else{
            ROS_INFO("Path found");
            return true;
        }
    }

    return true;
}

void PreprocessPlanner::PreProcess(const StateType &full_start_state){

    regions_.clear();
    ROS_INFO ("Preprocessing");
    if (params_ctmp_.planner_type == "RRTConnect"){
        InitMoveitOMPL();
    }
    // else{
    //     InitwAstarIMS();
    // }

    double radius_max_v = 0.2;
    double radius_max_i = 0.2;
    ROS_INFO("Getting regions");
    task_space_->PassRegions(&regions_, &iregions_);

    // SAMPLE_VALID_STATE from this region (in WORKSPACE)
    int max_tries = 10000;
    StateType sampled_state;
    int sampled_state_id = task_space_->SampleAttractorState(sampled_state, max_tries);
    std::cout << "Sampled first attractor" << std::endl;
    preprocess_main_->getOrCreateSearchState(sampled_state_id);
    if (sampled_state_id == -1) {
        ROS_ERROR("Failed to sample first attractor");
        return;
    }
    task_space_->VisualizePoint(sampled_state_id, "attractor");

    while (!task_space_->m_valid_front.empty() || !task_space_->m_invalid_front.empty()){
        while (!task_space_->m_valid_front.empty()){

            std::cout << "Entered while loop" << std::endl;
            // pop a valid state from the set of Valid states
            int attractor_state_id = task_space_->SetAttractorState();

            // check if any subregion is covering this state
            if (!preprocess_main_->is_state_covered(attractor_state_id)){
                task_space_->VisualizePoint(attractor_state_id, "attractor");

                // compute a path from start(CSPACE) to attractor(WSPACE)
                std::vector<StateType> path;

                bool ret;
                auto attr_state = task_space_->getRobotState(attractor_state_id);
                if (attr_state->state_mapped.empty()){
                    StateType mapped_state;
                    task_space_->getIKSolution(attr_state->state, mapped_state);
                    attr_state->state_mapped = mapped_state;
                }
                ret = PlanPathFromStartToAttractorOMPL(attr_state->state_mapped, path);

                if (!ret){
                    ROS_ERROR("Could not find a path from start to attractor");
                    continue;
                }

                std::cout << "Computing Reachability" << std::endl;
                // compute subregion around this attractor state - COMPUTE_REACHABILITY
                task_space_->UpdateSearchMode(REACHABILITY);
                double radius = preprocess_main_->compute_reachability(radius_max_v, attractor_state_id);

                region r;
                r.start = full_start_state;
                // task_space_->VisualizePoint(task_space_->getOrCreateRobotState(full_start_state), "start");
                r.radius = radius;
                r.state = attr_state->state;
                r.id = attractor_state_id;
                // task_space_->VisualizePoint(attractor_state_id, "attractor");
                r.path = path;
                regions_.push_back(r);

                ROS_INFO("Radius %f, Regions so far %zu", radius, regions_.size());
                ROS_INFO("Path size: %zu", r.path.size());

                // save all frontier state ids in open
                std::vector<int> open;
                preprocess_main_->get_frontier_stateids(open);
                task_space_->FillFrontierLists(open);
            }
        }
        while (!task_space_->m_invalid_front.empty()){
            int iv_start_state_id = task_space_->SetInvalidStartState();

            if (!preprocess_main_->is_state_covered(iv_start_state_id)){

                task_space_->VisualizePoint(sampled_state_id, "attractor");
                double radius = preprocess_main_->search_for_valid_uncovered_states(radius_max_i, iv_start_state_id);
                region r;
                r.radius = radius;
                r.state = task_space_->getRobotState(iv_start_state_id)->state;
                iregions_.push_back(r);

                ROS_INFO("Radius %f, IRegions so far %zu", radius, iregions_.size());

                std::vector<int> open;
                preprocess_main_->get_frontier_stateids(open);
                task_space_->FillFrontierLists(open);

                if (!task_space_->m_valid_front.empty()) {
                    break;
                }
            }
        }
    }
    task_space_->PruneRegions();

    std::cout << "Visualizing Regions along with their start and attractor states" << std::endl;
    
    for (const auto &reg : regions_){
        task_space_->VisualizeRegion(reg.id, reg.radius);
        // task_space_->VisualizePoint(task_space_->getOrCreateRobotState(full_start_state), "start");
        task_space_->VisualizePoint(reg.id, "attractor");
    }

    // write regions to a file
    WriteRegions();
}

// void PreprocessPlanner::InitwAstarIMS(){
//     ROS_INFO("Planning path with IMS");

//     // wastar_ptr_ = std::make_unique<ims::wAStar>(params_ctmp_);
// }

void PreprocessPlanner::InitMoveitOMPL(){
    group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(arm_name_);
    ROS_INFO("Planning path with OMPL");
    group_->setPlanningTime(10.0);
    group_->setPlannerId("BiTRRT");
}

bool PreprocessPlanner::PlanPathFromStartToAttractorOMPL(const StateType &attractor, std::vector<StateType> &path){


    ROS_INFO("Planning path with OMPL");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    group_->setStartStateToCurrentState();

    robot_state::RobotState goal_state(*group_->getCurrentState());
    goal_state.setJointGroupPositions(arm_name_, attractor);
    group_->setJointValueTarget(goal_state);

    ROS_INFO_STREAM(group_->getName());
    // plan
    ROS_INFO("Going to plan!");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto ret = group_->plan(my_plan);
    sleep(1);

    if (ret != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_WARN("OMPL failed to plan");
        return false;
    }
    else {
        ROS_INFO("Solution found by OMPL");
    }

    path.resize(my_plan.trajectory_.joint_trajectory.points.size());
    for (size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); ++i) {
        auto positions = my_plan.trajectory_.joint_trajectory.points[i].positions;
        path[i] = positions;
    }

    // int counter = 0;
    // for (auto& state : path) {
    //     std::cout << "State: " << counter++ << ": ";
    //     for (auto& val : state) {
    //         std::cout << val << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    spinner.stop();

    return true;
}

void PreprocessPlanner::Query(std::vector<StateType> &path, std::string grasp_dir){
    
    auto start_time = std::chrono::system_clock::now();
    task_space_->UpdateSearchMode(QUERY);
    if (grasp_dir.empty()){
        grasp_dir = read_write_dir_;
    }
    compute_path(path, grasp_dir);
    auto find_time = std::chrono::system_clock::now();
    std::chrono::duration<double> find_time_elapsed = find_time - start_time;
    stats_.time = find_time_elapsed.count() * 1000.0;
    if (!path.empty()){
        StateType start_state = path.front();
        StateType goal_state = path.back();
        std::shared_ptr<std::vector<StateType>> path_ptr = std::make_shared<std::vector<StateType>>(path);
    }
    return;
}

void PreprocessPlanner::compute_path(std::vector<StateType> &path, std::string grasp_dir){
    ROS_INFO("Grasp dir: %s", grasp_dir.c_str());
    boost::filesystem::path dir(grasp_dir);
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(dir); itr != end_itr; ++itr){
        std::string grasp_dir_ = itr->path().string();
        std::string grasp_name = itr->path().filename().string();
        if (grasp_name.find(task_space_->getPlanningGroupName()) == std::string::npos){
            continue;
        }
        ROS_INFO("Grasp dir: %s", grasp_dir.c_str());
        ROS_INFO("Grasp: %s", grasp_name.c_str());

        regions_.clear(); iregions_.clear();
        task_space_->PassRegions(&regions_,&iregions_);
        ReadRegions(grasp_dir_);
        ROS_INFO("Regions: %zu", regions_.size());

        ROS_INFO("Look for region containing goal state");
        int reg_idx = task_space_->FindRegionContainingState_WS(goal_,false);

        if (reg_idx == -1){
            ROS_INFO_STREAM("Query state not covered in file: " << grasp_name);
            continue;
        }

        ROS_INFO("Region index: %d", reg_idx);
        auto region = regions_[reg_idx];
        auto attr_state_ind = task_space_->getOrCreateRobotState(region.state); 
        // auto attr_state_ind = region.id;
        auto attr_state = task_space_->getRobotState(attr_state_ind);
        if (attr_state->state_mapped.empty()){
            attr_state->state_mapped = region.path.back();
        }
        auto goal_state_ind = task_space_->getOrCreateRobotState(goal_);
        auto goal_state = task_space_->getRobotState(goal_state_ind);

        std::vector<int> greedy_path;
        if (!preprocess_main_->findGreedyPath(goal_state_ind, attr_state_ind, greedy_path)){
            ROS_INFO("No greedy path found");
            continue;
        }
        ROS_INFO("Greedy path size: %zu", greedy_path.size());
        path = region.path;
        
        // loop over the greedy path and get the IK of all states. Add to greedy path to the path
        auto seed = attr_state->state_mapped; // Why are we doing this??
        // StateType seed = {0, -0.8, -1.0, -1.5, 0, 0};
        bool first_iter=true;
        for (auto state_ind : greedy_path){
            auto state_ = task_space_->getRobotState(state_ind);
            StateType joint_state;
            ROS_INFO("state->state elements:");
            for (size_t i = 0; i < state_->state.size(); ++i) {
                ROS_INFO("[%lu]: %f", i, state_->state[i]);
            }
        
            ROS_INFO("Seeded");
            task_space_->getIKSolution(state_->state, seed, joint_state); // Use seed
            
            state_->state_mapped = joint_state;

            path.push_back(state_->state_mapped);   // state_->getState()
            seed = state_->state_mapped;
        }
        break;
    }
}

void PreprocessPlanner::WriteRegions(std::string path){

    std::sort(regions_.begin(), regions_.end(), [] (const region &a,
                                                    const region &b)
    {
        return (a.radius > b.radius);
    });

    if (path.empty()){
        path = read_write_path_;
    }
    ROS_INFO("Writing regions to file");
    boost::filesystem::path myFile = path;
    std::cout << myFile;
    boost::filesystem::ofstream ofs(myFile);
    boost::archive::text_oarchive ta(ofs);
    ta << regions_;
}

void PreprocessPlanner::ReadRegions(std::string path) {

    ROS_INFO("Reading regions from file");

    try {
        if (path.empty()){
            path = read_write_path_;
        }
        boost::filesystem::path myFile = path; // boost::filesystem::current_path() /
        boost::filesystem::ifstream ifs(myFile/*.native()*/);
        boost::archive::text_iarchive ta(ifs);
        ta >> regions_;
    }
    catch (...) {
        ROS_WARN("Unable to read preprocessed file");
    }
}

