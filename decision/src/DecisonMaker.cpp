#include "DecisionMaker.h"
#include <jsoncpp/json/json.h>



namespace Decision
{
    Json::Reader json_reader;
    std::ifstream  json_file;
    Json::Value root;
    std::string json_file_path = "/home/rcx/HA_common/src/json_files/start_and_goal.json";

    std::pair<float, float>  handin;

    DecisionMaker::DecisionMaker(param *initParam)
    {
        DM_Param = initParam;
        DM_Param->lastState = INITIAL_STATE;

        init_state = new InitState(DM_Param);
        DM_state = init_state;
        HA_state = new HA_PlanningState(DM_Param);
        general_kp_state = new GeneralKeypointState(DM_Param);
        cusp_kp_state = new CuspKeypointState(DM_Param);
        term_kp_state = new TerminalKeypointState(DM_Param);
        complete_state = new MissionCompleteState(DM_Param);
        waiting_state = new WaitingState(DM_Param);

        init_state->NextStates.push_back(HA_state);

        HA_state->NextStates.push_back(general_kp_state);
        HA_state->NextStates.push_back(cusp_kp_state);
        HA_state->NextStates.push_back(waiting_state);
        // HA_state->NextStates.push_back(term_kp_state);

        general_kp_state->NextStates.push_back(cusp_kp_state);
        general_kp_state->NextStates.push_back(term_kp_state);
        general_kp_state->NextStates.push_back(HA_state);
        general_kp_state->NextStates.push_back(waiting_state);

        // cusp_kp_state->NextStates.push_back(term_kp_state);
        cusp_kp_state->NextStates.push_back(HA_state);
        cusp_kp_state->NextStates.push_back(general_kp_state);
        cusp_kp_state->NextStates.push_back(waiting_state);

        term_kp_state->NextStates.push_back(complete_state);
        term_kp_state->NextStates.push_back(HA_state);
        term_kp_state->NextStates.push_back(waiting_state);

        waiting_state->NextStates.push_back(HA_state);
        waiting_state->NextStates.push_back(general_kp_state);
        waiting_state->NextStates.push_back(cusp_kp_state);
        waiting_state->NextStates.push_back(term_kp_state);
        waiting_state->NextStates.push_back(complete_state);

        complete_state->NextStates.push_back(HA_state);

        // to do
        // waiting state;

        subStart = n.subscribe("/initialpose", 1, &DecisionMaker::setStart, this);
        subGoal = n.subscribe("/move_base_simple/goal", 1, &DecisionMaker::setGoal, this);
        subMap = n.subscribe("/simu/global_map", 1, &DecisionMaker::setMap, this);
        subPath = n.subscribe("/path_target/keypoints_map", 1, &DecisionMaker::setGlobalKeypoints, this);
        subKeypointIndex = n.subscribe("/path_target/cusp_index", 1, &DecisionMaker::setCuspIndex, this);
        subStartTime = n.subscribe("/hybrid_start_time", 1, &DecisionMaker::setStartTime, this);
        subReplanFlag = n.subscribe("/replan_flag", 1, &DecisionMaker::setReplanFlag, this);
        subDwaConCmd = n.subscribe("/dwa_planner/control_cmd", 1, &DecisionMaker::setControlCmd, this);

        json_file.open(json_file_path);
      if (!json_reader.parse(json_file, root))
     {
        std::cout << "Error opening json file  : " << json_file_path << std::endl;
     }
    }

    DecisionMaker::~DecisionMaker()
    {
        delete init_state;
        delete HA_state;
        delete general_kp_state;
        delete cusp_kp_state;
        delete term_kp_state;
        delete complete_state;
    }

    void DecisionMaker::process()
    {
        std::cout << "present state is " << DM_state->StateName << std::endl;
        DM_state->pubControlCmd();
        DM_state->pubPerceptionFlag();
        DM_state = DM_state->getNextState();
        DM_state->updateParam();
    }

    void DecisionMaker::setMap(const nav_msgs::OccupancyGrid _map)
    {
        DM_state->setMapParam(_map);
        // ROS_INFO("GOT A NEW MAP!");
    }

    void DecisionMaker::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &startPoint)
    {
         static bool testflag_out = 0;
        geometry_msgs::PoseStamped tmp_point;
        float x = startPoint->pose.pose.position.x / HybridAStar::Constants::cellSize;
        float y = startPoint->pose.pose.position.y / HybridAStar::Constants::cellSize;
        float yaw = tf::getYaw(startPoint->pose.pose.orientation);

        //  x = 459;
        //  y = -5;
        //  yaw = 0;
    if (bool switch_handle = root["json_switch_handle"].asBool() )
    {
        x = root["start_x"].asFloat();
        y = root["start_y"].asFloat();
        yaw = root["start_yaw"].asFloat();

        x =  handin.first;
        y = handin.second;
    }

    if ( testflag_out  &&   root["json_switch_handle"].asBool() )
     {
        x = root["alpout_start_x"].asFloat();
        y = root["alpout_start_y"].asFloat();
        yaw = root["alpout_start_yaw"].asFloat();
    }
        // yaw =  1.6;


        ROS_INFO("Got a new start at (%.2f, %.2f, %.2f)", x, y, yaw);
        int map_o_x = DM_Param->gridMap.info.origin.position.x;
        int map_o_y = DM_Param->gridMap.info.origin.position.y;
        float map_res = DM_Param->gridMap.info.resolution;
        int height = DM_Param->gridMap.info.height;
        int width = DM_Param->gridMap.info.width;


        if ((y - map_o_y) / map_res >= 0 && (y - map_o_y) / map_res <= height && (x - map_o_x) / map_res >= 0 && (x - map_o_x) / map_res <= width)
        {
            ROS_INFO("Start point is valid!");
            tmp_point.header.frame_id = "map";
            tmp_point.header.stamp = startPoint->header.stamp;
            //tmp_point.pose.position = startPoint->pose.pose.position;
            //tmp_point.pose.orientation = startPoint->pose.pose.orientation;
            tmp_point.pose.position.x = x * HybridAStar::Constants::cellSize;
            tmp_point.pose.position.y = y * HybridAStar::Constants::cellSize;
            tmp_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
           // std::cout << "start push forward 10 m" <<  std::endl;
            DM_state->setStartParam(tmp_point);
            DM_state->setStartValid(true);
            testflag_out = true;
        }
        else
        {
            ROS_ERROR("Start point is invalid(out of the border)!!!!!");
            DM_state->setStartValid(false);
        }
    }

    void DecisionMaker::setGoal(const geometry_msgs::PoseStamped::ConstPtr &goalPoint)
    {
        static bool testflag_out = 0;
       geometry_msgs::PoseStamped tmp_point;
        float x = goalPoint->pose.position.x / HybridAStar::Constants::cellSize;
        float y = goalPoint->pose.position.y / HybridAStar::Constants::cellSize;
        float yaw = tf::getYaw(goalPoint->pose.orientation);
    
    //--------test2022.3.30-------------
        if (  root["json_switch_handle"].asBool() )
        {
            x = root["goal_x"].asFloat();
            y = root["goal_y"].asFloat();
            yaw = root["goal_yaw"].asFloat();

          
            float dist = y;
            std::pair <int, int> projection_point(0,0);
            int backward_dist = root["backward_dist"].asInt();
            std::pair <float, float>   backward_point(projection_point.first - backward_dist, projection_point.second);
            std::pair <float, float> unit_vector(cos(yaw), sin(yaw));
            float vector_weight = root["vector_weight"].asFloat();  
            std::pair <float, float> direction_to_projection( projection_point.first - backward_point.first, projection_point.second - backward_point.second );
            float offset = direction_to_projection.first*vector_weight*unit_vector.first + direction_to_projection.second*vector_weight*unit_vector.second; 
            std::pair <float, float>  handover_in(backward_point.first*(1+  (dist - 30)/(150 - 30))+offset, backward_point.second);
            handin  = handover_in;
            ROS_INFO("Offset is  (%.2f)", offset);
           
        }
        
     if ( testflag_out   &&   root["json_switch_handle"].asBool() )
        {
             x = root["alpout_goal_x"].asFloat();
             y = root["alpout_goal_y"].asFloat();
            yaw = root["alpout_goal_yaw"].asFloat();

    
            x = handin.first - root["handover_down_dist"].asFloat();
            y = handin.second + 20;

        }

        //  x = 512;
        //  y = 35;
        //  yaw =-3.14*0.5;

        // x = -35;
        // y = 175;
        // yaw = 0;

        // x =  -20.25;
        // y = 176.1;
        // yaw =  0.01;
        ROS_INFO("Got a new goal at (%.2f, %.2f, %.2f)", x, y, yaw);
        int map_o_x = DM_Param->gridMap.info.origin.position.x ;
        int map_o_y = DM_Param->gridMap.info.origin.position.y;
        float map_res = DM_Param->gridMap.info.resolution;
        int height = DM_Param->gridMap.info.height;
        int width = DM_Param->gridMap.info.width;
        std::cout << "map_o_x" << map_o_x << "map_o_y" << map_o_y << "map_res" << map_res <<std::endl;
        //--real-map-test--
        if ((y - map_o_y) / map_res >= 0 && (y - map_o_y) / map_res <= height && (x - map_o_x) / map_res >= 0 && (x - map_o_x) / map_res <= width)
        //if (true)
        //--real-map-test--
        {
            ROS_INFO("Goal point is valid!");
            tmp_point.header.frame_id = "map";
            tmp_point.header.stamp = goalPoint->header.stamp;
            //tmp_point.pose.position = goalPoint->pose.position;
            //tmp_point.pose.orientation = goalPoint->pose.orientation;
            tmp_point.pose.position.x = x* HybridAStar::Constants::cellSize;
            tmp_point.pose.position.y = y * HybridAStar::Constants::cellSize;
            //rcx_edit
            tmp_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            //std::cout<<tmp_point.pose.orientation.w;
            // tmp_point.pose.orientation.x = goalPoint->pose.orientation.x;
            // tmp_point.pose.orientation.y = goalPoint->pose.orientation.y;
            // tmp_point.pose.orientation.z = goalPoint->pose.orientation.z;
            // tmp_point.pose.orientation.w = goalPoint->pose.orientation.w;
            tmp_point.pose.position.z = 0.f;
            if (!DM_Param->firstPlanFlag)
            {
                // 不是初次规划；
                ROS_INFO("Goal point has changed!");
                DM_state->setReplanParam(true);
                testflag_out = true;
                // DM_state->resetParam();
            }
            else
            {
                // 是初次规划；
                ROS_INFO("This is first planning!");
                DM_Param->firstPlanFlag = false;
                testflag_out = true;
            }

            DM_state->setGoalParam(tmp_point);
            DM_state->setGoalValid(true);
        }
        else
        {
            ROS_ERROR("Goal point is invalid(out of the border)!!!!!");
            DM_state->setStartValid(false);
        }
    }

    void DecisionMaker::setGlobalKeypoints(const nav_msgs::Path globalKeypoints_)
    {
        DM_state->setKeypointsParam(globalKeypoints_);
        
        // 收到路径的第一时间就判断下个关键点是否是人字形关键点；
        int length1 = DM_Param->globalKeypoints.poses.size();
        int length2 = DM_Param->cuspIndexes.data.size();
        if (length1 >= 3 && length2 >= 1)
        {
            for (int i = 1; i < length2; i++)
            {
                if (DM_Param->cuspIndexes.data[i] == length1 - 1 - 1)
                {
                    DM_Param->turningFlag.data = true;
                    return;
                }
            }
        }
        // 
        else if (length1 == 1)
        {
            // 没考虑交接点发生碰撞的情形，请确保交接点不撞，否则是交接点选取的问题；
            DM_Param->waitingFlag = true;
        }
    }

    void DecisionMaker::setCuspIndex(const std_msgs::Int8MultiArray cuspIndex)
    {
        DM_state->setCuspParam(cuspIndex);

        int length1 = DM_Param->globalKeypoints.poses.size();
        int length2 = DM_Param->cuspIndexes.data.size();
        if (length1 >= 3 && length2 >= 1)
        {
            for (int i = 1; i < length2; i++)
            {
                if (DM_Param->cuspIndexes.data[i] == length1 - 1 - 1)
                {
                    DM_Param->turningFlag.data = true;
                    return;
                }
            }
        }
    }

    void DecisionMaker::setStartTime(const std_msgs::Float32 time)
    {
        DM_state->setStartTimeParam(time);
    }

    void DecisionMaker::setReplanFlag(const std_msgs::Bool flag)
    {
        DM_state->setReplanParam(flag.data);
    }

    void DecisionMaker::setControlCmd(const vehicle_msgs::adm_lat controlCmd)
    {
        DM_state->setFinalCmdMsg(controlCmd);
    }

    // void DecisionMaker::updateParam()
    // {
    //     // 超出60s预测时间则准备进入重规划状态；
    //     float duration = ros::Time::now().sec - DM_Param->HAStartTime.data.sec;
    //     if (duration > 60)
    //         DM_state->setReplanParam(false);
    // }
}