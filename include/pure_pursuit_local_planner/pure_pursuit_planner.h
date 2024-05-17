#ifndef PURE_PURSUIT_PLANNER_H_
#define PURE_PURSUIT_PLANNER_H_

// Original repo: 
// Ported by Nawab Aditya

#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <dynamic_reconfigure/server.h>
#include <pure_pursuit_local_planner/transform_global_plan.h>
#include <pure_pursuit_local_planner/join_costmap.h>
#include <pure_pursuit_local_planner/PurePursuitPlannerConfig.h>

#include <vector>
#include <Eigen/Core>

using namespace std;

namespace pure_pursuit_local_planner{

class PurePursuitPlannerROS : public nav_core::BaseLocalPlanner {
public:

    PurePursuitPlannerROS();
    PurePursuitPlannerROS(std::string name, tf2_ros::Buffer* tf, 
                          costmap_2d::Costmap2DROS* costmap_ros);

    ~PurePursuitPlannerROS();

    /**
     * @brief Constructs the local planner
     * @param name The name to give this instance of the local planner
     * @param tf A pointer to a transform listener
     * @param costmap_ros The cost map to use for assigning costs to local plans
     */
    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief  Set the plan that the local planner is following
     * @param plan The plan to pass to the local planner
     * @return True if the plan was updated successfully, false otherwise
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
     * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid velocity command was found, false otherwise
     */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
     * @brief  Check if the goal pose has been achieved by the local planner
     * @return True if achieved, false otherwise
     */
    bool isGoalReached();
private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    bool initialized_;

    /** 
    *@brief Reconfigure config_
    */
    void reconfigureCB(PurePursuitPlannerConfig &config, uint32_t level);

    /**
     * @brief Get robot coordinates in local frame
     * @param name The name to give this instance of the local planner
     */
    void getGoalLocalCoordinates(std::vector<double> &localCoordinates,
                                 geometry_msgs::PoseStamped globalCoordinates,
                                 double look_ahead);
    
    void setControls(std::vector<double> look_ahead, geometry_msgs::Twist& cmd_vel, double yaw);

    /**
     * @brief Get euclidean distance
     * @param intial x and y points and end x and y points
     */
    double getEuclideanDistance(const double x_init, const double y_init,
                                const double x_end, const double y_end) const;
    
    //publisher where the local plan for visulatation is published
    ros::Publisher local_plan_publisher_;
    //check if plan first at first time
    bool first_setPlan_;
    // true if the robot should rotate to gobal plan if new global goal set
    bool rotate_to_global_plan_;
    //true if the goal point is reache and orientation of goal is reached
    bool goal_reached_;
    //true if the goal point is reache and orientation of goal isn't reached
    bool stand_at_goal_;
    //rotation velocity of previous round for the rotateToOrientation methode
    double cmd_vel_angular_z_rotate_;
    //x velocity of the previous round
    double cmd_vel_linear_x_;
    //rotation velocity of previous round for the dirveToward methode
    double cmd_vel_angular_z_;
    //for dynamic reconfigure
    dynamic_reconfigure::Server<PurePursuitPlannerConfig> *dsrv_;
    //start config
    pure_pursuit_local_planner::PurePursuitPlannerConfig default_config_;
    //reconfigure config
    pure_pursuit_local_planner::PurePursuitPlannerConfig config_;
    //global plan which we run along
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    //transformed global plan in global frame with only the points with are needed for calculation (max_points)
    std::vector<geometry_msgs::PoseStamped> transformed_global_plan_;
    //last point of the global plan in global frame
    geometry_msgs::PoseStamped goal_pose_;
    // true if the robot should rotate to gobal plan if new global goal set
    geometry_msgs::PoseStamped old_goal_pose_;

    JoinCostmap *joinCostmap_;
};
};

#endif 