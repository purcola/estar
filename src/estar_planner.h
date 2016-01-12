#ifndef ESTAR_PLANNER_H
#define ESTAR_PLANNER_H

#include <nav_core/base_global_planner.h>

namespace estar {
class EstarPlanner: public nav_core::BaseGlobalPlanner
{
public:
    EstarPlanner();

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief  Initialization function for the BaseGlobalPlanner
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

private:

    bool initialized_;

};
}

#endif // ESTAR_PLANNER_H
