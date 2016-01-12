#include "estar_planner.h"

estar::EstarPlanner::EstarPlanner():
    initialized_(false)
{
}

/**
 * @brief Given a goal pose in the world, compute a plan
 * @param start The start pose
 * @param goal The goal pose
 * @param plan The plan... filled by the planner
 * @return True if a valid plan was found, false otherwise
 */
bool estar::EstarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (not initialized_)
        return false;

    return true;
}

/**
 * @brief  Initialization function for the BaseGlobalPlanner
 * @param  name The name of this planner
 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
 */
void estar::EstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialized_ = true;
}
