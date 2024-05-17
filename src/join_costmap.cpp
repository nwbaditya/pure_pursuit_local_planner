#include <ros/ros.h>
#include <pure_pursuit_local_planner/join_costmap.h>

namespace pure_pursuit_local_planner{
    JoinCostmap::JoinCostmap(){};

    void JoinCostmap::initialize(costmap_2d::Costmap2DROS* local_costmap_ros,
                            costmap_2d::Costmap2DROS* global_costmap_ros){
        local_costmap_ros_ = local_costmap_ros;
        global_costmap_ros_ = global_costmap_ros;

        if(local_costmap_ros_->getCostmap()->getResolution() < global_costmap_ros_->getCostmap()->getResolution()){
            ROS_ERROR("JoinCostmap: Resolution of local costmap is higher than global costmap resolution!");
            init = false;
        }

        //Init vector global to right size
        global.resize(global_costmap_ros_->getCostmap()->getSizeInCellsX());
        for (unsigned int i = 0; i < global_costmap_ros_->getCostmap()->getSizeInCellsX(); ++i){
           global[i].resize(global_costmap_ros_->getCostmap()->getSizeInCellsY());
        }

        //Push global costmap in global vector
        for(unsigned int i = 0; i < global_costmap_ros_->getCostmap()->getSizeInCellsX(); i++){
            for(unsigned int j = 0; j < global_costmap_ros_->getCostmap()->getSizeInCellsY(); j++){
                global[i][j] = global_costmap_ros_->getCostmap()->getCost(i,j);
            }
        }
        ROS_INFO("JoinCostmap: Initalize.");
        init = true;
    }

    void JoinCostmap::joinMaps(){
        if(!init){
            ROS_WARN("JoinCostmap: Dont join costmap, because init faild.");
        }else{
            for(unsigned int i = 0; i < global_costmap_ros_->getCostmap()->getSizeInCellsX(); i++){
                for(unsigned int j = 0; j < global_costmap_ros_->getCostmap()->getSizeInCellsY(); j++){
                    double wx = 0;
                    double wy = 0;
                    global_costmap_ros_->getCostmap()->mapToWorld(i,j, wx, wy);

                    unsigned int mx = 0;
                    unsigned int my = 0;
                    local_costmap_ros_->getCostmap()->worldToMap(wx, wy, mx, my);

                    //Copy the highest cost from global vector or local costmap in the global costmap
                    if(global[i][j] < local_costmap_ros_->getCostmap()->getCost(mx, my)){
                        global_costmap_ros_->getCostmap()->setCost(i,j,local_costmap_ros_->getCostmap()->getCost(mx, my));
                    }else{
                        global_costmap_ros_->getCostmap()->setCost(i,j,global[i][j]);
                    }
                }
            }
        }
    }
};