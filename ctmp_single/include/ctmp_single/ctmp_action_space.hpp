#ifndef CTMP_CTMPACTIONSPACE_HPP
#define CTMP_CTMPACTIONSPACE_HPP

// standard includes
#include <iostream>
#include <vector>
#include <random>
#include <yaml-cpp/yaml.h>

// search includes
#include <search/heuristics/standard_heuristics.hpp>

// manipulation_planning includes
#include <manipulation_planning/common/utils.hpp>
#include "manipulation_planning/action_space/manipulation_action_space.hpp"

// ROS includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// include Axes visualizer
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <manipulation_planning/action_space/manipulation_action_space.hpp>
#include <manipulation_planning/common/moveit_scene_interface.hpp>
#include <search/planners/wastar.hpp>
#include <manipulation_planning/heuristics/manip_heuristics.hpp>
#include <manipulation_planning/common/utils.hpp>

// ROS includes
#include <ros/ros.h>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

#include <ctmp_single/ctmp_utils.hpp>

namespace ims{

    struct ctmpActionType : public ManipulationType{

        ctmpActionType(): ManipulationType("../config/ws.mprim"){
            setSpaceType(ManipulationType::SpaceType::WorkSpace);
        }

        explicit ctmpActionType(const std::string &mprimFile) : ManipulationType(mprimFile){
            setSpaceType(ManipulationType::SpaceType::WorkSpace);
        }
    };

    class ctmpActionSpace : public ManipulationActionSpace{

    public:

        ctmpActionSpace(const MoveitInterface &env,
                        const ctmpActionType &actions_ptr) : ManipulationActionSpace(env, actions_ptr){
            m_pnh = ros::NodeHandle(env.group_name_);
            readGoalRegions();
            m_vis_pub = m_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        }

        /// @brief Get the planning group name
        /// @return The planning group name as a string
        std::string getPlanningGroupName() const {
            return moveit_interface_->group_name_;
        }

        /// @brief Get the moveit interface pointer
        /// @return The moveit interface pointer
        std::shared_ptr<MoveitInterface> getMoveitInterface() const {
            return moveit_interface_;
        }

        /// @brief Reading goal regions from the parameter server as defined by the user
        /// @return True if the goal regions were read successfully, false otherwise
        bool readGoalRegions(){
            std::string region_type;
            ros::param::get("/region_type", region_type);

            // region_type = "place";
            
            std::cout << "region_type: " << region_type << std::endl;
            XmlRpc::XmlRpcValue xlist;
            if (!m_pnh.getParam("/" + moveit_interface_->group_name_ + "/regions/" + region_type + "_region/min_limits", xlist)) {
                std::string msg = moveit_interface_->group_name_ + "/regions/" + region_type + "_region/min_limits";
                ROS_WARN("Could not find start region min limits!!!! Searched for %s", msg.c_str());
                return false;
            }

            if (xlist.size() != 6) {
                ROS_ERROR("min limits: %zu params required, %d provided", 6, xlist.size());
                return false;
            }

            for (size_t i = 0; i < xlist.size(); ++i) {
                m_min_ws_limits.push_back(xlist[i]);
            }

            if (!m_pnh.getParam("regions/" + region_type + "_region/max_limits", xlist)) {
                ROS_WARN("Could not find start region max limits!!!!");
                return false;
            }

            if (xlist.size() != 6 ) {
                ROS_ERROR("max limits: %zu params required, %d provided", 6, xlist.size());
                return false;
            }

            for (size_t i = 0; i < xlist.size(); ++i) {
                m_max_ws_limits.push_back(xlist[i]);
            }

            // normalize [-pi,pi]x[-pi/2,pi/2]x[-pi,pi]
            // center of cell
            normalize_euler_zyx(m_min_ws_limits[5], m_min_ws_limits[4], m_min_ws_limits[3]);
            normalize_euler_zyx(m_max_ws_limits[5], m_max_ws_limits[4], m_max_ws_limits[3]);
            roundStateToDiscretization(m_min_ws_limits, manipulation_type_->state_discretization_);
            roundStateToDiscretization(m_max_ws_limits, manipulation_type_->state_discretization_);

            for (size_t i = 0; i < m_min_ws_limits.size(); ++i) {
                if (m_min_ws_limits[i] > m_max_ws_limits[i]) {
                    ROS_ERROR("Min limit greater than max limit at index %zu", i);
                    return false;
                }
            }

            m_distribution.resize(6);

            for (int i = 0; i < m_distribution.size() ; ++i) {
                m_distribution[i] = std::uniform_real_distribution<double> (m_min_ws_limits[i], m_max_ws_limits[i]);
            }
            ROS_INFO("m_distribution size1: %lu", m_distribution.size());ROS_INFO("m_distribution size: %lu", m_distribution.size());
            return true;

        }

        /// @brief Pass the regions to action space
        /// @param regions_ptr The regions
        /// @param iregions_ptr The invalid regions
        void PassRegions(std::vector<region>* regions_ptr,
                        std::vector<region>* iregions_ptr)
        {
            m_regions_ptr = regions_ptr;
            m_iregions_ptr = iregions_ptr;
        }

        /// @brief Sample a state in the workspace as an attractor state
        /// @param state The sampled state
        /// @param max_tries The maximum number of tries
        /// @return int The sampled state id, -1 if no state was sampled
        int SampleAttractorState(StateType &state, const int max_tries){
            int attractor_state_id;
            int count {0};
            while (count < max_tries){
                count++;
                // std::cout << "count: " << count << std::endl;
                StateType joint_state, ws_state;
                if (!SampleRobotState(joint_state, ws_state)){
                    continue;
                }
                /// TODO: check if graspable
                state = ws_state;
                attractor_state_id = getOrCreateRobotState(state);
                // check if state is covered in the preprocessed goal regions
                if (IsStateCovered(true, attractor_state_id)){
                    continue;
                }
                auto attractor_state = getRobotState(attractor_state_id);
                attractor_state->state_mapped = joint_state;
                m_valid_front.insert(attractor_state_id);

                return attractor_state_id;
            }
            return -1;
        }

        bool SampleRobotState(StateType &joint_state, StateType &workspace_state){
            // ROS_INFO("Number of joints: %ld", moveit_interface_->num_joints_);
            workspace_state.resize(6);
            // ROS_INFO("workspace_state size: %lu", workspace_state.size());
            // ROS_INFO("m_distribution size: %lu", m_distribution.size());
            for (int i = 0; i < workspace_state.size(); ++i){
                workspace_state[i] = m_distribution[i](m_generator);
            }
            normalize_euler_zyx(workspace_state[5], workspace_state[4], workspace_state[3]);
            roundStateToDiscretization(workspace_state, manipulation_type_->state_discretization_);

            if (!isStateValid(workspace_state, joint_state)){
                return false;
            }
            else{
                return true;
            }
        }

        /// @brief Check if the state is in some of the regions which were preprocessed
        /// @param valid True if the state is valid, false otherwise
        /// @param state_id The state id
        /// @return True if the state is in a region, false otherwise
        bool IsStateCovered(bool valid, const int state_id){
            std::vector<region>* regions_ptr;
            if (valid)
                regions_ptr = m_regions_ptr;
            else
                regions_ptr = m_iregions_ptr;

            for (const auto& r : *regions_ptr) {
                int center_state_id = getOrCreateRobotState(r.state); // getOrCreateState(r.state);
                // use getHeuristic function from ims::SE3Heuristic
                double dsum {INT_MAX};
                getSE3RPYdistance(getRobotState(state_id)->state,
                                getRobotState(center_state_id)->state,
                                dsum);
                // if (valid)
                //     printf("dsum %d radius %u\n", dsum, r.radius);
                if (dsum < r.radius || dsum == 0) {
                    return true;
                }
            }
            return false;
        }

        /// @brief Check if the state is in the goal region
        /// @param workspace_state The state
        /// @return True if the state is in the goal region, false otherwise
        bool isStateInGoalRegion(const StateType & workspace_state){
            double eps {0.0001};
            for (int i {0}; i < workspace_state.size(); ++i) {
                if (workspace_state[i] < m_min_ws_limits[i] - eps || workspace_state[i] > m_max_ws_limits[i] + eps) {
                    return false;
                }
            }
            return true;
        }

        /// @brief Pop a state from the valid frontier list and Set the attractor state to the first state in the valid frontier
        /// @return int The state id of the attractor state
        int SetAttractorState()
        {
            auto it = m_valid_front.begin();
            int entry_id = *it;
            auto entry = getRobotState(entry_id);
            m_valid_front.erase(it);

            StateType workspace_state = entry->state;

            return getOrCreateRobotState(workspace_state);

        }

        /// @brief Get  an IK solution for a given state
        /// @param state_val The state to get the IK solution for
        /// @param joint_state The IK solution
        /// @return True if an IK solution was found, false otherwise
        bool getIKSolution(const StateType & state_val, StateType & joint_state) {
            // check if the state is valid
            switch (manipulation_type_->getSpaceType()) {
                case ManipulationType::SpaceType::ConfigurationSpace:
                    // raise error
                    ROS_ERROR("IK solution is irrelevant for configuration space");
                    return false;
                case ManipulationType::SpaceType::WorkSpace:
                    geometry_msgs::Pose pose;
                    pose.position.x = state_val[0]; pose.position.y = state_val[1]; pose.position.z = state_val[2];
                    // Euler angles to quaternion
                    Eigen::Quaterniond q;
                    from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
                    return moveit_interface_->calculateIK(pose, joint_state);
            }
            return false;
        }

        /// @brief Get  an IK solution for a given state
        /// @param state_val The state to get the IK solution for
        /// @param joint_state The IK solution
        /// @return True if an IK solution was found, false otherwise
        bool getIKSolution(const StateType & state_val, StateType & seed, StateType & joint_state) {
            // check if the state is valid
            switch (manipulation_type_->getSpaceType()) {
                case ManipulationType::SpaceType::ConfigurationSpace:
                    // raise error
                    ROS_ERROR("IK solution is irrelevant for configuration space");
                    return false;
                case ManipulationType::SpaceType::WorkSpace:
                    geometry_msgs::Pose pose;
                    pose.position.x = state_val[0]; pose.position.y = state_val[1]; pose.position.z = state_val[2];
                    // Euler angles to quaternion
                    Eigen::Quaterniond q;
                    from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
                    return moveit_interface_->calculateIK(pose, seed, joint_state);
            }
            return false;
        }

        bool getSuccessorsWs(int curr_state_ind,
                            std::vector<std::vector<int>> &seqs_state_ids,
                            std::vector<std::vector<double>> &seqs_transition_costs) {

            seqs_state_ids.clear();
            seqs_transition_costs.clear();

            // get the primitive actions
            std::vector<ActionSequence> prim_action_seqs;
            std::vector<std::vector<double>> prim_action_transition_costs;
            manipulation_type_->getPrimActions(prim_action_seqs, prim_action_transition_costs);

            // get the current state
            auto curr_state = this->getRobotState(curr_state_ind);
            auto curr_state_val = curr_state->state;

            // get the actions
            // auto actions = manipulation_type_->getPrimActions();
            // convert to quaternion
            Eigen::Quaterniond q_curr;
            from_euler_zyx(curr_state_val[5], curr_state_val[4], curr_state_val[3], q_curr);
            
            // get the successors. Each successor is  sequence of state_ids and a sequence of transition costs.
            for (size_t i{0}; i < prim_action_seqs.size(); i++){
                ActionSequence &prim_action_seq = prim_action_seqs[i];
                // Objects for the successor resulting from this action.
                std::vector<int> successor_seq_state_ids{curr_state_ind};
                std::vector<double> successor_seq_transition_costs = prim_action_transition_costs[i];
                // The first state is the current state and the last state is the successor.
                // The primitive action sequences are of for xyz-xyzw. Those start from the origin.
                // The first state is assumed to be valid.
                // Go through all the states in the sequence, compute new quaternion, normalize, discretize, and check for validity.
                for (size_t j{1}; j < prim_action_seq.size(); j++) {
                    Eigen::Quaterniond q_action{prim_action_seq[j][6], prim_action_seq[j][3], prim_action_seq[j][4],
                                                prim_action_seq[j][5]};
                    Eigen::Quaterniond q_new = q_curr * q_action;
                    // convert the quaternion to euler angles
                    double r, p, y;
                    get_euler_zyx(q_new, y, p, r);
                    // Create a new state. Now it is in xyzrpy.
                    StateType new_state_val = {curr_state_val[0] + prim_action_seq[j][0], // Add the action on x.
                                            curr_state_val[1] + prim_action_seq[j][1], // Add the action on y.
                                            curr_state_val[2] + prim_action_seq[j][2], // Add the action on z.
                                            r, p, y};
                    // Normalize the angles.
                    normalize_euler_zyx(new_state_val[5], new_state_val[4], new_state_val[3]);
                    // Discretize the state.
                    roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
                    // Check if the next state is valid.
                    // bool succ;
                    // StateType mapped_state;
                    // if (curr_state->state_mapped.empty()) {
                    //     succ = isStateValid(new_state_val,
                    //                         mapped_state);
                    // }
                    // else
                    //     succ = isStateValid(new_state_val,
                    //                         curr_state->state_mapped,
                    //                         mapped_state);
                    // if (succ) {
                    // create a new state
                    int next_state_ind = getOrCreateRobotState(new_state_val);
                    // Add the state to the successor sequence.
                    successor_seq_state_ids.push_back(next_state_ind);
                    // Add the cost. This overrides the value given from the primitive.
                    double cost{0};
                    for (int dim{0}; dim < 3; dim++) {
                        cost += prim_action_seq[j][dim] * prim_action_seq[j][dim];
                    }
                    // Add the cost of the rotation which is quaternion
                    double r_, p_, y_;
                    get_euler_zyx(q_action, y_, p_, r_);
                    cost += r_ * r_ + p_ * p_ + y_ * y_;
                    successor_seq_transition_costs[j - 1] = cost; // TODO(yoraish): probably a cleaner way to do this.
                    // }
                    // else {
                    //     // The transition is not valid.
                    //     // Clear the successor sequence.
                    //     successor_seq_state_ids.clear();
                    //     successor_seq_transition_costs.clear();
                    //     break; // From iterating through the sequence.
                    // }
                }
                if (!successor_seq_state_ids.empty()) {
                    seqs_state_ids.push_back(successor_seq_state_ids);
                    seqs_transition_costs.push_back(successor_seq_transition_costs);
                }
            }
            assert(seqs_state_ids.size() == seqs_transition_costs.size());
            return true;

//             StateType new_state_val;
//             for (auto action : actions) {
//                 new_state_val.clear();
//                 // create a new state in the length of the current state
//                 new_state_val.resize(curr_state_val.size());
//                 // increment the xyz coordinates
//                 for (int i {0} ; i < 3 ; i++) {
//                     new_state_val[i] = curr_state_val[i] + action[i];
//                 }

//                 Eigen::Quaterniond q_action {action[6], action[3], action[4], action[5]};
//                 auto q_new = q_curr * q_action;

//                 // convert the quaternion to euler angles
//                 get_euler_zyx(q_new, new_state_val[5], new_state_val[4], new_state_val[3]);
//                 normalize_euler_zyx(new_state_val[5], new_state_val[4], new_state_val[3]);
//                 // discretize
//                 roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);

// //                bool succ; StateType mapped_state;
// //                if (curr_state->getMappedState().empty()){
// //                    succ = isStateValid(new_state_val,
// //                                        mapped_state);
// //                }
// //                else
// //                    succ = isStateValid(new_state_val,
// //                                        curr_state->getMappedState(),
// //                                        mapped_state);
// //                if (succ) {
//                     // create a new state
//                 int next_state_ind = getOrCreateRobotState(new_state_val);
// //                new_state->setMappedState(mapped_state);
//                 // add the state to the successors
//                 successors.push_back(next_state_ind);
//                 // add the cost
//                 double cost {0};
//                 for (int i {0} ; i < 3 ; i++) {
//                     cost += action[i]*action[i];
//                 }
//                 // add the cost of the rotation which is quaternion
//                 double r, p, y;
//                 get_euler_zyx(q_action, y, p, r);
//                 cost += r*r + p*p + y*y;
//                 costs.push_back(cost);
// //                }
//             }
//             return true;
        }

        bool getSuccessorsCs(int curr_state_ind,
                            std::vector<std::vector<int>> &seqs_state_ids,
                            std::vector<std::vector<double>> &seqs_transition_costs) {

            seqs_state_ids.clear();
            seqs_transition_costs.clear();

            std::vector<ActionSequence> action_seqs;
            std::vector<std::vector<double>> action_transition_costs;
            getActionSequences(curr_state_ind, action_seqs, action_transition_costs, false);
            // Get the successors. Each successor is a sequence of state_ids and a sequence of transition costs.
            for (size_t i{0}; i < action_seqs.size(); i++) {
                ActionSequence &action_seq = action_seqs[i];
                // Objects for the successor resulting from this action.
                std::vector<int> successor_seq_state_ids{curr_state_ind};
                std::vector<double> successor_seq_transition_costs = action_transition_costs[i];
                // The first state is the current state and the last state is the successor.
                // Go through all the states in the sequence, normalize angles, discretize, and check for validity.
                // Normalize and discretize the first state and then go through all pairs [i, i+1].
                // The first state is assumed to be valid.
                normalizeAngles(action_seq.front(), joint_limits_);
                roundStateToDiscretization(action_seq.front(), manipulation_type_->state_discretization_);
                for (size_t j{0}; j < action_seq.size() - 1; j++) {
                    auto curr_state_val = action_seq[j];
                    auto new_state_val = action_seq[j + 1];
                    // Normalize the angles.
                    normalizeAngles(new_state_val, joint_limits_);
                    // Discretize the state.
                    roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);
                    // Check if the state transition went through discontinuity.
                    bool discontinuity{false};
                    // check for maximum absolute action
                    for (int dim{0}; dim < curr_state_val.size(); dim++) {
                        if (new_state_val[dim] < joint_limits_[dim].first ||
                            new_state_val[dim] > joint_limits_[dim].second) {
                            discontinuity = true;
                            break;
                        }
                    }

                    // if (!discontinuity && isStateToStateValid(curr_state_val, new_state_val)) {
                    //     // Create a new state.
                    //     int next_state_ind = getOrCreateRobotState(new_state_val);
                    //     // Add the state to the successors.
                    //     successor_seq_state_ids.push_back(next_state_ind);
                    //     // Transition costs were already added before.
                    // }
                    // else {
                    //     // The transition is not valid.
                    //     // Clear the successor sequence.
                    //     successor_seq_state_ids.clear();
                    //     successor_seq_transition_costs.clear();
                    //     break; // From iterating through the sequence.
                    // }

                    if (isStateValid(new_state_val)){
                        // create a new state
                        int next_state_ind = getOrCreateRobotState(new_state_val);
                        successor_seq_state_ids.push_back(next_state_ind);
                    }
                }
                if (!successor_seq_state_ids.empty()) {
                    seqs_state_ids.push_back(successor_seq_state_ids);
                    seqs_transition_costs.push_back(successor_seq_transition_costs);
                }
            }
            assert(seqs_state_ids.size() == seqs_transition_costs.size());
            return true;

            // // get the current state
            // auto curr_state = this->getRobotState(curr_state_ind);
            // auto curr_state_val = curr_state->state;
            // // get the actions
            // auto actions = manipulation_type_->getPrimActions();
            // // get the successors
            // for (auto action : actions) {
            //     // create a new state in the length of the current state
            //     StateType new_state_val {};
            //     new_state_val.resize(curr_state_val.size());
            //     std::fill(new_state_val.begin(), new_state_val.end(), 0.0);

            //     for (int i {0} ; i < curr_state_val.size() ; i++) {
            //         new_state_val[i] = curr_state_val[i] + action[i];
            //     }
            //     // normalize the angles
            //     normalizeAngles(new_state_val);
            //     // discretize the state
            //     roundStateToDiscretization(new_state_val, manipulation_type_->state_discretization_);

            //     // if (isStateToStateValid(curr_state_val, new_state_val)) {
            //     if (isStateValid(new_state_val)) {
            //         // create a new state
            //         int next_state_ind = getOrCreateRobotState(new_state_val);
            //         // add the state to the successors
            //         successors.push_back(next_state_ind);
            //         // add the cost
            //         // TODO: change this to the real cost
            //         double norm = 0;
            //         for (double i : action) {
            //             norm += i * i;
            //         }
            //         costs.push_back(sqrt(norm));
            //     }
            // }
            // return true;
        }

        bool getSuccessors(int curr_state_ind,
                           std::vector<std::vector<int>> &seqs_state_ids,
                           std::vector<std::vector<double>> &seqs_transition_costs) override {
            bool success;
            if (manipulation_type_->getSpaceType() == ManipulationType::SpaceType::ConfigurationSpace) {
                success = getSuccessorsCs(curr_state_ind,
                                          seqs_state_ids,
                                          seqs_transition_costs);
            } else {
                success = getSuccessorsWs(curr_state_ind,
                                          seqs_state_ids,
                                          seqs_transition_costs);
            }
            auto curr_state = this->getRobotState(curr_state_ind);
            // if REACHABILITY mode, check if successors in goal region.
            // If not, remove from successors
            if (m_search_mode == REACHABILITY) {
                std::vector<std::vector<int>> pruned_successors;
                std::vector<std::vector<double>> pruned_costs;
                std::vector<int> remove;

                for (size_t i{0}; i < seqs_state_ids.size(); ++i){
                    const std::vector<int> & successors_state_ids = seqs_state_ids[i];
                    std::vector<double> successor_seq_transition_costs = seqs_transition_costs[i];

                    int entry_id = successors_state_ids.back();
                    auto entry = this->getRobotState(entry_id);
                    auto ws_child = entry->state;

                    std::vector<int> entry_id_vec;
                    if (isStateInGoalRegion(ws_child)){
                        bool succ;
                        StateType mapped_state;
                        if (curr_state->state_mapped.empty()) {
                            succ = isStateValid(ws_child,
                                                mapped_state);
                        } else
                            succ = isStateValid(ws_child,
                                                curr_state->state_mapped,
                                                mapped_state);
                        if (succ) {
                            entry->state_mapped = mapped_state;
                        }
                        entry_id_vec.push_back(entry_id);

                        pruned_costs.push_back(successor_seq_transition_costs);
                    }
                    else{
                        remove.push_back(entry_id);
                    }
                    pruned_successors.push_back(entry_id_vec);
                }
                    seqs_state_ids = pruned_successors;
                    seqs_transition_costs = pruned_costs;
                // for (size_t i = 0; i < successors.size(); ++i) {
                //     auto entry_id = successors[i];
                //     auto entry = this->getRobotState(entry_id);
                //     auto ws_child = entry->state;
                //     if (isStateInGoalRegion(ws_child)) {
                //         bool succ;
                //         StateType mapped_state;
                //         if (curr_state->state_mapped.empty()) {
                //             succ = isStateValid(ws_child,
                //                                 mapped_state);
                //         } else
                //             succ = isStateValid(ws_child,
                //                                 curr_state->state_mapped,
                //                                 mapped_state);
                //         if (succ) {
                //             entry->state_mapped = mapped_state;
                //         }
                //         pruned_successors.push_back(entry_id);
                //         pruned_costs.push_back(costs[i]);
                //     } else {
                //         // delete entry;
                //         remove.push_back(entry_id);
                //     }
                // }
                // delete successors
            }
//            } else if (search_mode_ == QUERY){
//                std::vector<ims::state*> pruned_successors;
//                std::vector<double> pruned_costs;
//                std::vector<ims::state*> remove;
//                for (size_t i = 0; i < successors.size(); ++i) {
//                    auto* entry = successors[i];
//                    auto ws_child = entry->getState();
//                    if (isStateInGoalRegion(ws_child)){
//                        pruned_successors.push_back(entry);
//                        pruned_costs.push_back(costs[i]);
//                    }
//                    else {
//                        // delete entry;
//                        remove.push_back(entry);
//                    }
//                }
//                successors = pruned_successors;
//                costs = pruned_costs;
//            }
            return success;
        }

        /// @brief update the search mode (REACHABILITY)
        void UpdateSearchMode(int search_mode){m_search_mode = search_mode;}

        /// @brief Fill the frontier lists with the given state ids
        /// @param state_ids The state ids
        void FillFrontierLists(const std::vector<int>& state_ids){
            for (const auto& state_id : state_ids) {
                auto entry = getRobotState(state_id);
                StateType joint_space_state = entry->state_mapped;
                if (isStateValid(entry->state, joint_space_state)) {
                    m_valid_front.insert(state_id);
                }
                else {
                    m_invalid_front.insert(state_id);
                }
            }
        }

        int SetInvalidStartState() {
            auto it = m_invalid_front.begin();
            int entry_id = *it;
            auto entry = getRobotState(entry_id);
            m_invalid_front.erase(it);

            if (entry->state_mapped.empty()){
                StateType joint_state;
                getIKSolution(entry->state, joint_state);
                entry->state_mapped = joint_state;
            }

            return entry_id;
        }

        ///@brief Prune regions that are contained in other regions
        void PruneRegions(){
            std::vector<region> filtered_regions;
            for (const auto& r1 : *m_regions_ptr){
                bool keep = true;
                for (const auto& r2 : *m_regions_ptr){
                    double epsilon = 1e-4;
                    double dis {INT_MAX};
                    getSE3RPYdistance(r1.state, r2.state, dis);
                    if (dis == 0){
                        continue;
                    } else if (r2.radius - r1.radius > dis){    //  + epsilon
                        keep = false;
                        break;
                    }
                }
                if (keep){
                    filtered_regions.push_back(r1);
                }
            }
            ROS_INFO("Filtered regions from %zu to %zu", m_regions_ptr->size(), filtered_regions.size());
            *m_regions_ptr = filtered_regions;
        }

        /// @brief Find region containing the state
        /// @param workspace_state The state
        /// @param position_only If true, heck only the position values. Default is false
        /// @return int The region id, -1 if no region was found
        int FindRegionContainingState_WS(const StateType & workspace_state,
                                         bool position_only=false){
            int query_state_id = getOrCreateRobotState(workspace_state);
            bool covered = false;
            int reg_idx = 0;
            StateType goal_state;

            for (const auto& r : *m_regions_ptr) {
                int center_state_id = getOrCreateRobotState(r.state);
                // use getHeuristic function from ims::SE3Heuristic
                double dsum {INT_MAX};
                if (!position_only){
                    getSE3RPYdistance(getRobotState(query_state_id)->state,
                                      getRobotState(center_state_id)->state, dsum);
                } else {
                    dsum = 0.0;
                    for (size_t i {0}; i < 3; ++i) {
                        double dj = (workspace_state[i] - r.state[i]);
                        dsum += dj*dj;
                    }
                    dsum = std::sqrt(dsum);
                }

                if (dsum < r.radius || dsum == 0) {
                    goal_state = r.state;
                    covered = true;
                    break;
                }
                reg_idx++;
            }

            if (covered) {
                ROS_DEBUG_NAMED("graph", "Attractor State of Containing Region %d", reg_idx);
                return reg_idx;
            }
            else {
                ROS_INFO_STREAM_NAMED("graph.expands", "  start workspace_state: " << workspace_state[0] << " "
                                                                                   << workspace_state[1] << " " << workspace_state[2] << " " << workspace_state[3] << " " <<
                                                                                   workspace_state[4] << " " << workspace_state[5]);
                return -1;
            }
        }

        /// @brief Visualize a state point in rviz for debugging
        /// @param state_id The state id
        /// @param type The type of state (greedy, attractor, etc)
        void VisualizePoint(int state_id, const std::string& type){
            auto entry = getRobotState(state_id);
            auto ws_parent = entry->state;

            visualization_msgs::Marker marker;
            marker.header.frame_id = moveit_interface_->planning_scene_->getPlanningFrame();
            marker.header.stamp = ros::Time();
            marker.ns = "graph";
            marker.id = m_vis_id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = ws_parent[0]; marker.pose.position.y = ws_parent[1]; marker.pose.position.z = ws_parent[2];
            marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
            ros::NodeHandle nh;
            nh = ros::NodeHandle();
            std::string group_name = "manipulator_1";
            std::string region_type;
            ros::param::get("/region_type", region_type);
            std::vector<double> discretization;
            nh.getParam("/"+ group_name + "/regions/" + region_type + "_region/discretization", discretization);

            marker.scale.x = discretization[0]; marker.scale.y = discretization[1]; marker.scale.z = discretization[2];
            if (type == "greedy"){
                // green
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
                marker.color.a = 0.5;
            } else if (type == "attractor"){
                // blue
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
                marker.color.a = 0.8;
            } else if (type == "exited"){
                // red
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
                marker.color.a = 1.0;
            } else if (type == "invalid"){
                // black
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 0.0;
                marker.color.a = 1.0;
            }
            else if (type == "start"){
                // white
                marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0;
                marker.color.a = 1.0;
            }
            else{
                // yellow
                marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
                marker.color.a = 0.8;
            }
            // visualize
            m_vis_pub.publish(marker);
            m_vis_id++;
        }

        /// @param state_id state id of attractor state
        /// @param r radius
        /// @param group_name - to change the color of the region for each manipulator
        void VisualizeRegion(int state_id, double r){

            auto entry = getRobotState(state_id);
            auto ws_parent = entry->state;

            visualization_msgs::Marker marker;
            marker.header.frame_id = moveit_interface_->planning_scene_->getPlanningFrame();
            marker.header.stamp = ros::Time();
            marker.ns = "graph";
            marker.id = m_vis_id;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = ws_parent[0]; marker.pose.position.y = ws_parent[1]; marker.pose.position.z = ws_parent[2];
            marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;

            marker.scale.x = r; marker.scale.y = r; marker.scale.z = 0;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            // visualize
            m_vis_pub.publish(marker);
            m_vis_id++;
        }

        // valid/invalid uncovered frontier states
        std::set<int> m_valid_front;
        std::set<int> m_invalid_front;

    private:
        ros::NodeHandle m_pnh;
        ros::Publisher m_vis_pub;
        ros::NodeHandle m_nh;
        int m_vis_id = 0;
        int m_search_mode;
        std::vector<region>* m_regions_ptr;
        std::vector<region>* m_iregions_ptr;

        // where are these limits set?? - DOUBT
        std::vector<double> m_min_ws_limits;
        std::vector<double> m_max_ws_limits;

        std::vector<std::uniform_real_distribution<double>> m_distribution; // Inclusive Exclusive uniform distribution
        std::default_random_engine m_generator; // Pseudo random values generator (deterministic)
    };
}

#endif //CTMP_CTMPACTIONSPACE_HPP