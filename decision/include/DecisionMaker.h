#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <algorithm>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include "state.h"
#include "hybrid_astar/constants.h"
#include "map"

namespace Decision
{
    class DecisionMaker
    {
    public:
        DecisionMaker(param *initParam);
        ~DecisionMaker();

        void process();
        // void updateParam();

    public:
        void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &startPoint);
        void setGoal(const geometry_msgs::PoseStamped::ConstPtr &goalPoint);
        void setMap(const nav_msgs::OccupancyGrid _map);
        void setStartTime(const std_msgs::Float32 time);
        void setReplanFlag(const std_msgs::Bool flag);
        void setGlobalKeypoints(const nav_msgs::Path globalKeypoints_);
        void setCuspIndex(const std_msgs::Int8MultiArray cuspIndex);
        void setControlCmd(const vehicle_msgs::adm_lat controlCmd);
        struct CSV_data
        {
            float x;
            float y;
            float heading;
        };



    private:
        param *DM_Param;
        State *DM_state;
        InitState *init_state;
        HA_PlanningState *HA_state;
        GeneralKeypointState *general_kp_state;
        CuspKeypointState *cusp_kp_state;
        TerminalKeypointState *term_kp_state;
        MissionCompleteState *complete_state;
        WaitingState *waiting_state;
        
        std::vector<CSV_data> up_road_;
        std::vector<CSV_data> down_road_;

        ros::NodeHandle n;
        ros::Subscriber subStart;
        ros::Subscriber subGoal;
        ros::Subscriber subMap;
        ros::Subscriber subPath;
        ros::Subscriber subKeypointIndex;
        ros::Subscriber subStartTime;
        ros::Subscriber subReplanFlag;
        ros::Subscriber subDwaConCmd;

        void setMainRoad(std::string, std::string);
        CSV_data getHandoverPoint(float, float, float, bool, float);

        
        
    };
}
#endif