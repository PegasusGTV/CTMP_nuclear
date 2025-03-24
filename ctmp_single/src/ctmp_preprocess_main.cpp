#include <ctmp_single/ctmp_preprocess_main.hpp>

ims::preprocessMain::preprocessMain(const BestFirstSearchParams &params) : 
BestFirstSearch(params), call_number_(0), iteration_(1), h_max_(0), reachability_expansions_(0) {}

auto ims::preprocessMain::getSearchState(int state_id) -> ims::preprocessMain::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::preprocessMain::getOrCreateSearchState(int state_id) -> ims::preprocessMain::SearchState * {
    if (state_id >= states_.size()){
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr){
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState;
        states_[state_id]->state_id = state_id;
    }
    return states_[state_id];
}

void ims::preprocessMain::initializePlanner(const std::shared_ptr<ctmpActionSpace> &actionSpacePtr,
                                            const StateType &start, const StateType &goal){
    
    ctmp_action_space_ = actionSpacePtr;
    action_space_ptr_ = actionSpacePtr;    //WHY IS THIS REQUIRED??? - to access BFS functions
    StateType seed, joint_state_start;

    // Deal with this when working on query phase

    ctmp_action_space_->getCurrJointStates(seed);
    if (!ctmp_action_space_->isStateValid(start, joint_state_start)){
        ROS_ERROR("Start State is not valid initPlanner_main");
    }
    StateType joint_state_goal = joint_state_start;
    if (!ctmp_action_space_->isStateValid(goal, joint_state_goal)){
        ROS_ERROR("Goal State is not valid initPlanner_main");
    }
    int start_ind = ctmp_action_space_->getOrCreateRobotState(start);
    auto start_robot = ctmp_action_space_->getRobotState(start_ind);
    start_robot->state_mapped = joint_state_start;
    auto start_state = getOrCreateSearchState(start_ind);
    start_state->parent_id = START;
    int goal_ind = ctmp_action_space_->getOrCreateRobotState(goal);
    auto goal_state = getOrCreateSearchState(goal_ind);
    goals_.push_back(goal_ind);

    goal_state->parent_id = GOAL;
    heuristic_->setGoal(const_cast<StateType &>(goal));
    start_state->g = 0;
    start_state->h = computeHeuristic(start_ind);
    start_state->f = start_state->h;
    goal_state->h = 0;
    ROS_INFO("Preprocess Main planner initialized");
}

void ims::preprocessMain::reinitSearchState(int s_ind) {
    // check if state index has already been added to the set of reachable states
    if (state_initiated_.find(s_ind) == state_initiated_.end()){
        state_initiated_.insert(s_ind);
        auto s = getSearchState(s_ind);
        s->covered_this = false;
        s->greedy = false;
        s->h = INF_DOUBLE;
        s->f = INF_DOUBLE;
        s->in_open = false; 
        s->in_closed = false;
    }
}

bool ims::preprocessMain::is_state_covered(int id){
    auto search_state = getSearchState(id);
    return search_state->covered;
}

void ims::preprocessMain::get_frontier_stateids(std::vector<int> &state_ids) {
    for (auto it = open_.begin(); it != open_.end(); ++it) {
        state_ids.push_back((*it)->state_id);
    }
}

double ims::preprocessMain::compute_reachability(double r_max, int attractor_state_id){
    ++call_number_;
    iteration_ = 1;
    h_max_ = 0;
    h_max_states_.clear();
    state_initiated_.clear();
    open_.clear();

    // reinit the attractor state variables - heuristic = 0
    auto attractor_state = getSearchState(attractor_state_id);
    reinitSearchState(attractor_state_id);
    attractor_state->greedy = true;
    attractor_state->covered = true;
    attractor_state->h = 0;
    attractor_state->setClosed();

    preds_.clear(); // closed?
    costs_.clear();
    // get successors of the attractor state
    ctmp_action_space_->getSuccessors(attractor_state_id, preds_, costs_);

    for (size_t i{0}; i < preds_.size(); ++i){
        const std::vector<int> & preds_state_ids = preds_[i];
        int pred_id = preds_state_ids.back();
        auto pred_state = getOrCreateSearchState(pred_id);
        reinitSearchState(pred_id);
        pred_state->h = computeHeuristic(pred_id, attractor_state_id);
        pred_state->f = pred_state->h;
        if (open_.contains(pred_state)){
            open_.decrease(pred_state);
        }
        else{
            open_.push(pred_state);
        }
    }

    // // h(s, sa)
    // for (const auto &pred : preds_){
    //     auto pred_state = getOrCreateSearchState(pred);
    //     reinitSearchState(pred);
    //     pred_state->h = computeHeuristic(pred, attractor_state_id);
    //     pred_state->f = pred_state->h;
    //     if (open_.contains(pred_state)){
    //         open_.decrease(pred_state);
    //     }
    //     else{
    //         open_.push(pred_state);
    //     }
    // }

    double radius = 0;

    while (radius <= r_max && !open_.empty()){
        succs_.clear();
        costs_.clear();
        auto min_state = open_.min(); // successor of Sa
        open_.pop();
        min_state->setClosed();
        reachability_expansions_++;

        // if the predecessor (argmin) has h value greater than the max h value - this is not the greedy predecessor
        // init a h_max to 0
        if (min_state->h > h_max_){
            h_max_states_.clear();
        }

        // update h_max value to be that of the max h value of the succ of Sa
        h_max_ = min_state->h;

        // if not previously covered - i.e. this pred state is now covered - succ of Sa
        if (!min_state->covered) {
            min_state->covered = true;
            min_state->covered_this = true;
        }

        // push the pred state with max h value to the vector of h_max_states_ - why are we keeping track of this?? DOUBT
        h_max_states_.push_back(min_state);

        // getting succs of the min_h val pred state of Sa
        auto start = std::chrono::system_clock::now();
        ctmp_action_space_->getSuccessors(min_state->state_id, succs_, costs_);

        // calculating the greedy preds of the succs of Sa
        SearchState* succ_state_g = nullptr;
        double min_h = INF_DOUBLE;
        std::vector<SearchState*> greedy_succs;
        
        // calculating greedy predecessor of the succ of Sa
        for (size_t i{0}; i < succs_.size(); ++i){
            const std::vector<int> & succs_state_ids = succs_[i];
            int succ_state_id = succs_state_ids.back();
            auto succ_state = getOrCreateSearchState(succ_state_id);
            reinitSearchState(succ_state_id);

            // calculate heuristic dist of the pred of the succ of Sa - sg
            succ_state->h = computeHeuristic(succ_state_id, attractor_state_id);

            if (succ_state->h < min_h){
                min_h = succ_state->h;
                succ_state_g = succ_state;
            }
        }

        // for (const auto &succ_state_id : succs_){
        //     auto succ_state = getOrCreateSearchState(succ_state_id);
        //     reinitSearchState(succ_state_id);

        //     // calculate heuristic dist of the pred of the succ of Sa - sg
        //     succ_state->h = computeHeuristic(succ_state_id, attractor_state_id);

        //     if (succ_state->h < min_h){
        //         min_h = succ_state->h;

        //         // succ_state->greedy = true; // CHECK THIS!!!! - sg
        //         // ctmp_action_space_->VisualizePoint(succ_state->state_id, "greedy");

        //         succ_state_g = succ_state;
        //     }
        // }
        std::chrono::duration<double, std::micro> duration = std::chrono::system_clock::now() - start;
        start = std::chrono::system_clock::now();
        auto min_robot = ctmp_action_space_->getRobotState(min_state->state_id);
        StateType joint_states_min = min_robot->state_mapped;
        
        // check if succ_state_g is valid and the edge connecting succ_state and succ_state_g is valid
        if (succ_state_g != nullptr && !joint_states_min.empty() && succ_state_g->greedy && 
            ctmp_action_space_->isStateToStateValid(ctmp_action_space_->getRobotState(min_state->state_id)->state, ctmp_action_space_->getRobotState(succ_state_g->state_id)->state)){

                min_state->greedy = true;
                ctmp_action_space_->VisualizePoint(min_state->state_id, "greedy");
        }
        else if (!joint_states_min.empty()){
            ctmp_action_space_->VisualizePoint(min_state->state_id, "non_greedy");

            radius = min_state->h;
            
            // since this succ of Sa does not have a valid greedy predecessor, mark this one as not covered
            for (auto s : h_max_states_){
                if (s->h == min_state->h){
                    if (s->covered_this){
                        s->covered = false;
                    }
                }
            }
            break;

//         start = std::chrono::system_clock::now();
//         auto min_robot = ctmp_action_space_->getRobotState(min_state->state_id);
//         StateType joint_states_min = min_robot->state_mapped;
//         if (succ_state_g != nullptr && (succ_state_g->greedy &&
// //            ctmp_action_space_->isStateToStateValid(min_state->getState(), succ_state_g->getState()))) {
//             !joint_states_min.empty())) {

//             min_state->greedy = true;
//             ctmp_action_space_->VisualizePoint(min_state->state_id, "greedy");
//         }
//             ///@}

//             ///@{ Terminating condition --line 10-11
// //        else if (ctmp_action_space_->isStateValid(min_state->getState(), joint_states_min)){
//         else if (!joint_states_min.empty()){   // TODO: Its a hack. I check validity of the state when generating successors and  if valid then I save mappedstate.
//             ctmp_action_space_->VisualizePoint(min_state->state_id, "exited");
//             radius = min_state->h;

//             // unset covered
//             for (auto s : h_max_states_) {
//                 if (s->h == min_state->h) {
//                     if (s->covered_this) {
//                         s->covered = false;
//                     }
//                 }
//             }

//             break;
        }
        duration = std::chrono::system_clock::now() - start;
        ROS_DEBUG_STREAM("Greedy set criteria: " << duration.count());
        
        radius = min_state->h;
        start = std::chrono::system_clock::now();
        ROS_DEBUG_NAMED("reachability", "Inserting preds in Open");

        for (size_t i{0}; i < succs_.size(); ++i){
            const std::vector<int> & preds_state_ids_ = succs_[i];
            int pred_state_id_ = preds_state_ids_.back();
            auto pred_state_ = getSearchState(pred_state_id_);
            reinitSearchState(pred_state_id_);

            if (!pred_state_->greedy){
                if (!pred_state_->in_closed){
                    pred_state_->h = computeHeuristic(pred_state_id_, attractor_state_id);
                    pred_state_->f = pred_state_->h;
                    if (open_.contains(pred_state_)) {
                        open_.decrease(pred_state_);
                    } else {
                        open_.push(pred_state_);
                    }
                }
            }
        }

        // for (const auto &pred_state_id : succs_){
        //     auto pred_state = getSearchState(pred_state_id);
        //     reinitSearchState(pred_state_id);
        //     if (!pred_state->greedy){
        //         if (!pred_state->in_closed){
        //             pred_state->h = computeHeuristic(pred_state_id, attractor_state_id);
        //             pred_state->f = pred_state->h;
        //             if (open_.contains(pred_state)) {
        //                 open_.decrease(pred_state);
        //             } else {
        //                 open_.push(pred_state);
        //             }
        //         }
        //     }
        // }
        duration = std::chrono::system_clock::now() - start;
        ROS_DEBUG_NAMED("REACHABILITY", "----------------------------------------");
    }
    if (open_.empty()){
        printf("Valid open list got empty");
        radius += 0.2;
    }
    return radius;
}

double ims::preprocessMain::search_for_valid_uncovered_states(double r_max, int iv_start_state_ids){
    ++call_number_;
    iteration_ = 1;
    h_max_ = 0;
    h_max_states_.clear();
    state_initiated_.clear();
    open_.clear();

    auto iv_start_state = getSearchState(iv_start_state_ids);
    reinitSearchState(iv_start_state_ids);
    iv_start_state->h = 0;
    iv_start_state->f = iv_start_state->h;
    open_.push(iv_start_state);

    double radius = 0;

    while (radius <= r_max && !open_.empty()){
        preds_.clear();
        costs_.clear();
        auto min_state = open_.min();
        open_.pop();
        min_state->setClosed();

        if (min_state->h > h_max_){
            h_max_states_.clear();
        }
        h_max_ = min_state->h;

        // if not previously covered
        if (!min_state->covered) {
            min_state->covered = true;
            min_state->covered_this = true;
        }

        h_max_states_.push_back(min_state);
        ctmp_action_space_->VisualizePoint(min_state->state_id, "invalid");
        auto min_robot = ctmp_action_space_->getRobotState(min_state->state_id);
        StateType joint_states_min = min_robot->state_mapped;

        // if this state is not in any of the preprocessed regions and is valid
        if (!ctmp_action_space_->IsStateCovered(true, min_state->state_id) && ctmp_action_space_->isStateValid(min_robot->state, joint_states_min)){

            // push the state to open list so that we can compute a neighborhood around it
            open_.push(min_state); // insert this state in V - is it doing this though?? - check if it is getting updated in m_valid_front - the function which does this is FillFrontierLists
            radius = min_state->h; // 0??

            // ???? - DOUBT
            for (auto s : h_max_states_){
                if (s->h == min_state->h){
                    if (s->covered_this){
                        s->covered = false;
                    }
                }
            }
            break;
        }
        // find successors of the invalid state which does not satisfy the above conditions
        ctmp_action_space_->getSuccessors(min_state->state_id, preds_, costs_);
        for (size_t i{0}; i < preds_.size(); ++i){
            const std::vector<int> & m_preds_state_ids = preds_[i];
            int m_pred_id = m_preds_state_ids.back();
            auto pred_state = getOrCreateSearchState(m_pred_id);
            reinitSearchState(m_pred_id);
            if (!pred_state->in_closed){
                pred_state->h = computeHeuristic(m_pred_id, iv_start_state_ids);
                pred_state->f = pred_state->h;
                if (open_.contains(pred_state)){
                    open_.decrease(pred_state);
                }
                else{
                    open_.push(pred_state);
                }
            }
        }

        // ctmp_action_space_->getSuccessors(min_state->state_id, preds_, costs_);
        // for (auto &m_pred_id : preds_){
        //     auto pred_state = getOrCreateSearchState(m_pred_id);
        //     reinitSearchState(m_pred_id);
        //     if (!pred_state->in_closed){
        //         pred_state->h = computeHeuristic(m_pred_id, iv_start_state_ids);
        //         pred_state->f = pred_state->h;
        //         if (open_.contains(pred_state)) {
        //             open_.decrease(pred_state);
        //         } else {
        //             open_.push(pred_state);
        //         }
        //     }
        // }
        radius = min_state->h;
    }
    if (open_.empty()) {
        printf("Invalid Open list got empty\n");
        radius += 0.005;
    }
    return radius;
}

bool ims::preprocessMain::findGreedyPath(int goal_state_id, int attr_state_id, std::vector<int> &path){

    auto goal_state = getSearchState(goal_state_id);
    goal_state->h = computeHeuristic(goal_state_id, attr_state_id);
    int state_ind = goal_state_id;
    
    auto start_time = std::chrono::system_clock::now();
    double max_time = 3; //secs
    while(state_ind != attr_state_id){
        preds_.clear();
        costs_.clear();
        path.push_back(state_ind);
        auto state = getSearchState(state_ind);
        if (state->h < 1.7e-2){
            break;
        }
        auto current_time = std::chrono::system_clock::now();
        std::chrono::duration<double> duration = current_time - start_time;
        if (duration.count() > max_time){
            ROS_WARN("Timeout in finding greedy path");
            ROS_INFO("Current h: %f", state->h);
            return false;
        }

        ctmp_action_space_->getSuccessors(state_ind, preds_, costs_);
        double min_cost = std::numeric_limits<double>::infinity();
        for (size_t i{0}; i < preds_.size(); ++i){
            const std::vector<int> & preds_state_ids = preds_[i];
            int pred_state_id = preds_state_ids.back();
            auto pred_state = getOrCreateSearchState(pred_state_id);
            pred_state->h = computeHeuristic(pred_state_id, attr_state_id);
            if (pred_state->h < min_cost){
                min_cost = pred_state->h;
                state_ind = pred_state->state_id;
            }
        }

        // ctmp_action_space_->getSuccessors(state_ind, preds_, costs_);
        // double min_cost = std::numeric_limits<double>::infinity();
        // for (size_t i = 0; i < preds_.size(); ++i){
        //     auto pred_state_id = preds_[(int)i];
        //     auto pred_state = getOrCreateSearchState(pred_state_id);
        //     pred_state->h = computeHeuristic(pred_state_id, attr_state_id);
        //     if (pred_state->h < min_cost){
        //         min_cost = pred_state->h;
        //         state_ind = pred_state->state_id;
        //     }
        // }
    }
    if (false){
        ROS_WARN("Timeout in finding greedy path");
        return false;
    }
    else{
        std::reverse(path.begin(), path.end());
        return true;
    }
}

void ims::preprocessMain::resetPlanningData(){
    for (auto state_ : states_){
        delete state_;
    }
    states_.clear();
    open_.clear();
    goals_.clear();
    goal_ = -1;
    stats_ = PlannerStats();
}