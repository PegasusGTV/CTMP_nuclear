#ifndef CTMP_PREPROCESS_MAIN_HPP
#define CTMP_PREPROCESS_MAIN_HPP

#include <ctmp_single/ctmp_action_space.hpp>
#include <search/planners/best_first_search.hpp>
#include <utility>
#include <unordered_set>

namespace ims{

    class preprocessMain : public BestFirstSearch{

        private:

            struct SearchState : public BestFirstSearch::SearchState{

                double h = -1;

                /// @brief bool indicating whether the state is covered
                bool covered = false;
                bool covered_this = false;
                bool greedy {};
            };

            using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
            OpenList open_;

            std::vector<SearchState*> states_;

        public:
             
             /// @brief Get the state by id
            /// @param state_id The id of the state
            /// @return The state
            /// @note Use this function only if you are sure that the state exists
            auto getSearchState(int state_id) -> SearchState*;

            /// @brief Get the state by id or create a new one if it does not exist
            /// @param state_id The id of the state
            /// @return The state
            auto getOrCreateSearchState(int state_id) -> SearchState*;

            explicit preprocessMain(const BestFirstSearchParams &params);

            void initializePlanner(const std::shared_ptr<ctmpActionSpace> &actionSpacePtr,
                                   const StateType &start, const StateType &goal);

            bool is_state_covered(int id);

            double compute_reachability(double r_max, int attractor_state_id);

            void reinitSearchState(int s_ind);

            double search_for_valid_uncovered_states(double r_max, int iv_start_state_id);

            void get_frontier_stateids(std::vector<int> &state_ids);

            /// @brief Find a greedy path from goal state to attractor state and then return the reverse path.
            /// @param goal_state_id The goal state id.
            /// @param attr_state_id The attractor state id.
            /// @param path The path from goal to attractor state.
            /// @return True if a path was found, false otherwise.
            bool findGreedyPath(int goal_state_id, int attr_state_id, std::vector<int>& path); 

            void resetPlanningData() override;
        
        protected:

            std::shared_ptr<ims::ctmpActionSpace> ctmp_action_space_;

            int call_number_;

            int iteration_;

            // std::vector<int> succs_;
            // std::vector<int> preds_;
            // std::vector<double> costs_;
            std::vector<std::vector<int>> succs_;
            std::vector<std::vector<int>> preds_;
            std::vector<std::vector<double>> costs_;

            int reachability_expansions_;

            double h_max_;
            std::vector<SearchState*> h_max_states_;
            std::unordered_set<int> state_initiated_;  // set of reachable states?
    };
}

#endif //CTMP_PREPROCESS_MAIN_HPP