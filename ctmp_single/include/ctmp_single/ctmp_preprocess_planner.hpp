#ifndef CTMP_PREPROCESS_PLANNER_HPP
#define CTMP_PREPROCESS_PLANNER_HPP

#include <ctmp_single/ctmp_action_space.hpp>
#include <ctmp_single/ctmp_utils.hpp>

// search includes
#include <search/common/types.hpp>
#include <search/planners/wastar.hpp>

#include <ctmp_single/ctmp_preprocess_main.hpp>
// #include <manipulation_planning/common/moveit_scene_interface.hpp>
// #include <moveit_msgs/DisplayTrajectory.h>

// struct to store bfsheuristic params
/// TODO: how to store other variables in this (e.g. query_mode)
// struct PreprocessPlannerParams : public ims::wAStarParams {

//     explicit PreprocessPlannerParams(ims::BaseHeuristic* heuristic, double epislon) : wAStarParams(heuristic, epsilon){}
//     ~PreprocessPlannerParams() = default;

//     bool query_mode;
// };

struct PreprocessPlannerParams{

    explicit PreprocessPlannerParams(bool query_mode, const std::string &planner_type) : query_mode(query_mode), planner_type(planner_type){}
    ~PreprocessPlannerParams() = default;

    bool query_mode;
    std::string planner_type;
};

class PreprocessPlanner{

    public:
        PreprocessPlanner(const PreprocessPlannerParams &params_ctmp);
        ~PreprocessPlanner() = default;

        /// @brief Preprocess planner for generating offline paths
        bool initializePlanner(const std::shared_ptr<ims::ctmpActionSpace> &actionSpacePtr,
                               const StateType &start,
                               const StateType &goal,
                               const ims::BestFirstSearchParams &params);

        /// @brief Goal region preprocessing
        void PreProcess(const StateType &full_start_state);

        /// @brief Plan using CTMP
        bool plan(std::vector<StateType> &path);
        
        void Query(std::vector<StateType> &path, std::string grasp_dir = "");

        void compute_path(std::vector<StateType> &path, std::string grasp_dir = "");

        /// @brief Report stats of the planner.
        PlannerStats reportStats() const {
            return stats_;
        }

    private:
        // void InitwAstarIMS();

        void InitMoveitOMPL();

        bool PlanPathFromStartToAttractorOMPL(const StateType &attractor, std::vector<StateType> &path);

        void ReadRegions(std::string path="");

        void WriteRegions(std::string path="");

        ros::NodeHandle nh;
        ros::NodeHandle m_nh;

        StateType start_;
        StateType goal_;
        std::string arm_name_;

        std::string read_write_dir_;
        std::string read_write_path_;

        std::vector<region> regions_;
        std::vector<region> iregions_;

        std::shared_ptr<ims::ctmpActionSpace> task_space_;
        std::unique_ptr<ims::wAStar> wastar_ptr_;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group_;
        std::shared_ptr<ims::preprocessMain> preprocess_main_;

        PreprocessPlannerParams params_ctmp_;

        PlannerStats stats_;
};

#endif //CTMP_PREPROCESS_PLANNER_HPP