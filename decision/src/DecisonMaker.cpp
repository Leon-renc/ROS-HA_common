#include "DecisionMaker.h"
#include <jsoncpp/json/json.h>
#include <ros/package.h>



namespace Decision
{
    Json::Reader json_reader;
    std::ifstream  json_file;
    Json::Value root;
    std::string json_file_path;

    DecisionMaker::CSV_data  handin;

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
    
        std::string decision_package_path = ros::package::getPath("decision");
        json_file_path = decision_package_path + "/json_files/start_and_goal.json";
        json_file.open(json_file_path);
        if (!json_reader.parse(json_file, root))
        {
            std::cout << "Error opening json file  : " << json_file_path << std::endl;
        }
        setMainRoad(decision_package_path + "/json_files/up.csv", decision_package_path + "/json_files/down.csv");
        //setMainRoad2Obs(); //要将泊人路线设置成为障碍物需要手动设定参数
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
    if ( !testflag_out && root["json_switch_handle"].asBool() )
    {
         if(root["origin_handover_switch"].asBool()) 
                ROS_ERROR("ERROR! origin_handover_switch AND json_switch_handle  ARE BOTH VALID!");    
        x =  handin.x;
        y = handin.y;
        yaw = handin.heading;
    }

    if ( testflag_out  &&   root["json_switch_handle"].asBool() )
     {
           if(root["origin_handover_switch"].asBool()) 
                ROS_ERROR("ERROR! origin_handover_switch AND json_switch_handle  ARE BOTH VALID!");    
            x = root["goal_x"].asFloat();
            y = root["goal_y"].asFloat();
            yaw = root["goal_yaw"].asFloat();
    }
        // yaw =  1.6;
    if ( !testflag_out && root["origin_handover_switch"].asBool() )
    {
        if(root["json_switch_handle"].asBool()) 
            ROS_ERROR("ERROR! origin_handover_switch AND json_switch_handle  ARE BOTH VALID!"); 
        x = root["origin_start_x"].asFloat();
        y = root["origin_start_y"].asFloat();
        yaw = root["origin_start_yaw"].asFloat();
    }

     if ( testflag_out   &&    root["origin_handover_switch"].asBool() )
    {
        if(root["json_switch_handle"].asBool()) 
            ROS_ERROR("ERROR! origin_handover_switch AND json_switch_handle  ARE BOTH VALID!");            
        x = root["origin_goal_x"].asFloat();
        y = root["origin_goal_y"].asFloat();
        yaw = root["origin_goal_yaw"].asFloat();
    }

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
        if (  !testflag_out && root["json_switch_handle"].asBool() )
        {
            if(root["origin_handover_switch"].asBool()) 
                ROS_ERROR("ERROR! origin_handover_switch AND json_switch_handle  ARE BOTH VALID!");            
            x = root["goal_x"].asFloat();
            y = root["goal_y"].asFloat();
            yaw = root["goal_yaw"].asFloat();

            CSV_data handinpoint  = getHandoverPoint(x, y, yaw, testflag_out, root["handover_down_dist"].asFloat());
            handin = handinpoint;
        }
        
     if ( testflag_out   &&   root["json_switch_handle"].asBool() )
        {
           if(root["origin_handover_switch"].asBool()) 
                ROS_ERROR("ERROR! origin_handover_switch AND json_switch_handle  ARE BOTH VALID!");            
            CSV_data handoverpoint  = getHandoverPoint(x, y, yaw, testflag_out, root["handover_down_dist"].asFloat());
            x = handoverpoint.x;
            y = handoverpoint.y;
            yaw = handoverpoint.heading;
        }

        if (  !testflag_out && root["origin_handover_switch"].asBool() )
        {
            if(root["json_switch_handle"].asBool()) 
                ROS_ERROR("ERROR! origin_handover_switch AND json_switch_handle  ARE BOTH VALID!");
            x = root["origin_goal_x"].asFloat();
            y = root["origin_goal_y"].asFloat();
            yaw = root["origin_goal_yaw"].asFloat();
        }

     if ( testflag_out   &&    root["origin_handover_switch"].asBool() )
        {
            if(root["json_switch_handle"].asBool()) 
                ROS_ERROR("ERROR! origin_handover_switch AND json_switch_handle  ARE BOTH VALID!");            
              x = root["origin_alpout_goal_x"].asFloat();
              y = root["origin_alpout_goal_y"].asFloat();
             yaw = root["origin_alpout_goal_yaw"].asFloat();
        }

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

    void DecisionMaker::setMainRoad(std::string up_path, std::string down_path)
    {
        std::ifstream up_file(up_path);
        std::ifstream down_file(down_path);
        std::string one_line;
        CSV_data line;
        while ( getline(up_file, one_line) )
        {
            if ( !one_line.size())
            {
                break;
            }
            
            std::istringstream sin(one_line);
            std::vector<std::string> fields;
            std::string field;
            while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		    {
			    fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
		    }
            line.x = std::stof(fields[0]);
            line.y = std::stof(fields[1]);
            line.heading = std::stof(fields[2]);
           // std::cout<< " line.heading before   " <<  line.heading << std::endl;
            if(line.heading > 90 && line.heading < 270)
            {
                line.heading = -(line.heading-90)/180*3.14;
                //std::cout<< " line.heading after   " <<  line.heading << std::endl;
            }
           else if (line.heading <=90)
            {
                line.heading = (90 - line.heading)/180*3.14;
            }
            else if (line.heading >= 270)
            {
                line.heading = (360-line.heading+90)/180*3.14;
            }
            up_road_.push_back(line);
            //std::cout << "line.heading" <<line.heading <<  std::endl;
            fields.clear();
        }
         while ( getline(down_file, one_line) )
        {
            if ( !one_line.size())
            {
                break;
            }
            std::istringstream sin(one_line);
            std::vector<std::string> fields;
            std::string field;
            while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		    {
			    fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
		    }
            line.x = std::stof(fields[0]);
            line.y = std::stof(fields[1]);
            line.heading = std::stof(fields[2]);
            if(line.heading > 90 && line.heading < 270)
            {
                line.heading = -(line.heading-90)/180*3.14;
                //std::cout<< " line.heading after   " <<  line.heading << std::endl;
            }
           else if (line.heading <=90)
            {
                line.heading = (90 - line.heading)/180*3.14;
            }
            else if (line.heading >= 270)
            {
                line.heading = (360-line.heading+90)/180*3.14;
            }
            down_road_.push_back(line);
            fields.clear();
        }
        if(!up_road_.size() || !down_road_.size()){
            ROS_ERROR("main road infomation fail to get");
        }
    }

    DecisionMaker::CSV_data DecisionMaker::getHandoverPoint(float x, float y ,float yaw, bool alp_out, float down_diff)
    {
        //std::map<int, float> Hashmap;
        static CSV_data handover_in;
        //find handoverOUT
        float dist_min = INFINITY;
        if (alp_out)
        {
            auto pos = down_road_.begin();
                for (auto iter =down_road_.begin(); iter !=down_road_.end(); iter++)
            {
                float dist = sqrt((iter->x -  handover_in.x)*(iter->x -  handover_in.x) + (iter->y -  handover_in.y) * (iter->y -  handover_in.y));
                //Hashmap.insert(std::make_pair(i,dist));
                if ( dist < dist_min)
                {
                    dist_min = dist;
                    pos = iter;
                }
            }
            CSV_data in_projection_point = *pos;

            float dist = 0;
            for (auto iter = pos ; iter <  down_road_.end() ; iter++)
            {
                dist = sqrt((iter->x -  in_projection_point.x)*(iter->x -  in_projection_point.x) + (iter->y -  in_projection_point.y) * (iter->y -  in_projection_point.y));
                // std::cout <<iter->x << ", " << iter->y << std::endl;
                // std::cout<<"down_road_.end()-1 "<< (down_road_.end())->x << std::endl;
                if ( dist > down_diff )
                {
                    if (iter->x == down_road_.begin()->x)
                    {
                        break;
                    }
                    std::cout << "alp out handover point" << iter->x << "," << iter->y << "," << iter->heading << std::endl;
                    return *iter;
                }
            }
            std::cout << "out main road limited "  << std::endl;
            return down_road_.back();
        }

        //find projection_point
        auto pos = up_road_.begin();
        for (auto iter = up_road_.begin(); iter != up_road_.end(); iter++)
        {
            float dist = sqrt((iter->x -  x)*(iter->x -  x) + (iter->y -  y) * (iter->y -  y));
            //Hashmap.insert(std::make_pair(i,dist));
            if ( dist < dist_min)
            {
                dist_min = dist;
                pos = iter;
            }
        }
        CSV_data projection_point = {pos->x, pos->y, pos->heading};
        std::cout << "projection_point" << projection_point.x << "," << projection_point.y << "," <<  "," << projection_point.heading << std::endl;

        int backward_dist = (1+  (dist_min - 30)/(150 - 30))*root["backward_dist"].asInt();
        std::cout << "backward_dist" <<backward_dist << std::endl; 
        //find back_point
        float dist = 0;
        for (auto iter = pos ; iter != up_road_.begin(); iter--)
        {
            dist = sqrt((iter->x -  projection_point.x)*(iter->x -  projection_point.x) + (iter->y -  projection_point.y) * (iter->y -  projection_point.y));
            if ( dist >backward_dist )
            {
                pos = iter;
                break;
            }
        }
        if ( pos == up_road_.begin() ) 
        {
            handover_in = *pos;
            return handover_in;
        }
        
        CSV_data back_point = {pos->x, pos->y, pos->heading};
        std::cout << "back_point" << back_point.x << "," <<back_point.y << "," <<  back_point.heading<< std::endl;

        CSV_data back_point_vector = {projection_point.x - back_point.x,  projection_point.y- back_point.y,  back_point.heading};
        std::pair <float, float> unit_vector(cos(yaw), sin(yaw));
        float vector_weight = root["vector_weight"].asFloat();  
        float offset = back_point_vector.x*vector_weight*unit_vector.first + back_point_vector.y*vector_weight*unit_vector.second; 
        std::cout << "offset" <<offset << std::endl; 
        //find handover_point
        dist = 0;
        for (auto iter = pos ; ; offset > 0 ? iter++:iter--)
        {
            dist = sqrt((iter->x -  back_point.x)*(iter->x -  back_point.x) + (iter->y -  back_point.y) * (iter->y -  back_point.y));
            if ( dist > abs(offset) )
            {
                pos = iter;
                break;
            }
        }
        handover_in = *pos;
        std::cout << "handover_in     " << handover_in.x << "," <<handover_in.y << "," << handover_in.heading <<std::endl;
        return handover_in;
    }

    void DecisionMaker::setMainRoad2Obs()
    {
        int width = 505;
        int map_resolution = 1;
        std::ofstream fpath;
        std::string file_path = "/home/rcx/HA_common/src/json_files/mainroad_path.json";
        Json::Value root;

        for (auto iter = up_road_.begin(); iter != up_road_.end(); iter++)
        {
            int index = static_cast<int>((iter->y+106.52906799316406)/map_resolution)*width + static_cast<int>((iter->x + 265.83874130249023)/map_resolution);
            root["index"].append(index);
        }
        fpath.open(file_path);
        Json::StyledWriter sw;
        fpath << sw.write(root);
        fpath.close();
    
    }

    // void DecisionMaker::updateParam()
    // {
    //     // 超出60s预测时间则准备进入重规划状态；
    //     float duration = ros::Time::now().sec - DM_Param->HAStartTime.data.sec;
    //     if (duration > 60)
    //         DM_state->setReplanParam(false);
    // }
}