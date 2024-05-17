#include <pure_pursuit_local_planner/transform_global_plan.h>

namespace pure_pursuit_local_planner{
    bool getXPose(const tf2_ros::Buffer& tf,
                  const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const std::string& global_frame, geometry_msgs::PoseStamped& goal_pose, int plan_point){
        if(global_plan.empty()){
            ROS_ERROR("Received plan with zero length");
            return false; 
        }
        if(plan_point >= (int)global_plan.size())
        {
            ROS_ERROR("Goal_functions: Plan_point %d to big. Plan size: %lu",plan_point, global_plan.size());
            return false;
        }

        const geometry_msgs::PoseStamped& plan_goal_pose = global_plan.at(plan_point);
        geometry_msgs::TransformStamped tfGeom;
        try
        {
            tfGeom = tf.lookupTransform(global_frame, plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp, ros::Duration(0.5));
            
            geometry_msgs::PoseStamped transformed_goal_pose;
            tf2::doTransform(plan_goal_pose, transformed_goal_pose, tfGeom);
            goal_pose = transformed_goal_pose;
            goal_pose.header.stamp = tfGeom.header.stamp;
            goal_pose.header.frame_id = global_frame;
        }
        catch(tf2::LookupException& ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch(tf2::ConnectivityException& ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch(tf2::ExtrapolationException& ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }
        return true;
    };
};