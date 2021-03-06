#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include "state.h"
#include "hybrid_astar/constants.h"

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

        ros::NodeHandle n;
        ros::Subscriber subStart;
        ros::Subscriber subGoal;
        ros::Subscriber subMap;
        ros::Subscriber subPath;
        ros::Subscriber subKeypointIndex;
        ros::Subscriber subStartTime;
        ros::Subscriber subReplanFlag;
        ros::Subscriber subDwaConCmd;
    };
}
#endif