#include "Module.hpp"

//函数:
/******************************************************************************

 *****************************************************************************/
//新增函数

vector<Point2f> Module::pointset_input(vector<Point> area,int vehicle_num,bool formation_flag){
    int divide_num;
    if(formation_flag)
        divide_num=vehicle_num/2;
    else
        divide_num=vehicle_num;

    //确定边界
    double max_x=INT_MIN;
    double min_x=INT_MAX;
    double min_y=INT_MAX;
    double max_y=INT_MIN;

    for(auto &pt:area)
    {
        max_x=pt.x>max_x?pt.x:max_x;
        min_x=pt.x<min_x?pt.x:min_x;
        min_y=pt.y<min_y?pt.y:min_y;
        max_y=pt.y>max_y?pt.y:max_y;
    }

    vector<Point2f> pointset;
    Point2f fp;
    //沿着x方向分割
    for(int i=1;i<=divide_num;i++)
    {
        fp.x=min_x+(max_x-min_x)/(divide_num)*(i-0.5);
        fp.y=(min_y+max_y)/2;
        pointset.push_back(fp);
    }

    return pointset;
}

//函数:
/******************************************************************************
     分组函数：尽可能保证每组有一个侧扫
 *****************************************************************************/
std::vector<std::pair<int,int>> Module::Group(vector<bool> LateralPerception){
    int index=0;
    vector<int> vec1,vec2;
    std::vector<std::pair<int,int>> Group_Msg(LateralPerception.size()/2);

    //两个向量分别存有侧扫的索引和无侧扫的索引
    for(;index<LateralPerception.size();index++)
        if(LateralPerception[index]==true)
            vec1.push_back(index);
        else
            vec2.push_back(index);

    //合并两个向量
    int s1=vec1.size();
    int s2=vec2.size();
    if(s1==s2)
    {
        for(int i=0;i<s1;i++)
        {
            Group_Msg[i].first=vec1[i];
            Group_Msg[i].second=vec2[i];
        }
    }
    else if(s1>s2)
    {
        for(int i=0;i<s2;i++)
        {
            Group_Msg[i].first=vec1[i];
            Group_Msg[i].second=vec2[i];
        }

        for(int i=s2;i<s1;i++)
        {
            Group_Msg[(i-s2)/2+s2].first=vec1[i];
            i++;
            if(i==s1)
                break;
            Group_Msg[(i-s2)/2+s2].second=vec1[i];
        }
    } else if (s1<s2)
    {
        for(int i=0;i<s1;i++)
        {
            Group_Msg[i].first=vec1[i];
            Group_Msg[i].second=vec2[i];
        }

        for(int i=s1;i<s2;i++)
        {
            Group_Msg[(i-s1)/2+s1].first=vec2[i];
            i++;
            if(i==s1)
                break;
            Group_Msg[(i-s1)/2+s1].second=vec2[i];
        }
    }

    return Group_Msg;
}


//函数:
/******************************************************************************
//ANCHOR 1.分组算法：
                                                输入：
                                                            分组标志位  formation_flag
                                                            艇型号标志位 LateralPerception
                                                输出：
                                                            分组信息 group_msg
 *****************************************************************************/

std::vector<std::pair<int,int>> Module::Module_Group(bool formation_flag,vector<bool> LateralPerception){
        std::vector<std::pair<int,int>> group_msg;
            // std::cout << "group start" << std::endl;
    if(formation_flag)
        group_msg=Group(LateralPerception);//需要编组时用编组算法
    else
        for(int i=0;i<LateralPerception.size();i++)
        {
            std::pair<int,int> pr=std::make_pair(i,-1); //不需要编组时group_msg只首位有效，首位为index
            group_msg.push_back(pr);
        }
        return group_msg;
}
/******************************************************************************
//ANCHOR 2.分区算法：
                                                输入：
                                                            可视化区域  visual_area
                                                            智能体总数 vehicle_num
                                                            分组标志位  formation_flag
                                                            感知半径 perception_radius
                                                输出：
                                                            分割区域顶点坐标 divided_areas
 *****************************************************************************/

vector<vector<cv::Point>> Module::Module_Divide(vector<Point> visual_area,int vehicle_num,bool formation_flag,double perception_radius)
{
        // std::cout << "devide start" << std::endl;
    auto pts=pointset_input(visual_area,vehicle_num,formation_flag);

    Divider divider(pts);
    
    vector<vector<cv::Point>> divided_areas=divider.Divide_Loop(visual_area, perception_radius);
    
    return divided_areas;
}



/******************************************************************************
//ANCHOR 3.覆盖算法：
                                                输入：
                                                            分割区域顶点坐标  divided_areas
                                                            分组标志位  formation_flag
                                                            可视化图 _vis_map
                                                            分组信息 group_msg
                                                            各智能体起点 start_pts
                                                输出：
                                                            各智能体路径 output_trajs
 *****************************************************************************/

vector<vector<cv::Point2f>> Module::Module_Cover(vector<vector<cv::Point>> &divided_areas,bool formation_flag,cv::Mat3b &_vis_map,\
                                                                                            std::vector<std::pair<int,int>> &group_msg,std::vector<cv::Point> &start_pts, int vehicle_num){      

    /* 计算200米通道范围 */
    cv::Point2f temp_pt;
    /* --- 计算左上点 */
    temp_pt.x = center_start_pt.x;
    temp_pt.y = center_start_pt.y - 100;
    safe_channel.push_back(temp_pt);
    /* --- 计算左下点 */
    temp_pt.x = center_start_pt.y;
    temp_pt.y = center_start_pt.y + 100;
    safe_channel.push_back(temp_pt);
    /* --- 计算右上点 */
    temp_pt.x = center_start_pt.x + 5400;
    temp_pt.y = center_start_pt.y - 100;
    safe_channel.push_back(temp_pt);
    /* --- 计算右下点 */
    temp_pt.x = center_start_pt.x + 5400;
    temp_pt.y = center_start_pt.y + 100;
    safe_channel.push_back(temp_pt);

                                        
    vector<vector<cv::Point2f>>output_trajs(vehicle_num);
    std::vector<cv::Point2f> st_pts;
    for (auto pt : start_pts)
        st_pts.push_back(pt);
    // if ( form_bool) plannerInit(4, st_pts);
    // else plannerInit(4, st_pts);
    for(int index=0;index<divided_areas.size();index++)
    {
        cv::Point leader_pt;
        cv::Point follower_pt;

        if (formation_flag)
        {
            leader_pt = start_pts[2 * index];
            follower_pt = start_pts[2 * index + 1];
        }
        else
        {
            leader_pt = start_pts[index];
        }

        Cover cover(divided_areas[index]);
    //     /* 测试： */
        float perception_area;
        if (index%2 == 0)
        {
            perception_area = 15;
        }
        else
        {
            perception_area = 20;
        }
        

    cover_path_choice_first[index] = cover.CoverLoop(formation_flag, leader_pt, follower_pt, perception_area);
    //     // cover.CoverLoop(formation_flag, leader_pt, follower_pt);
    //     std::vector<std::vector<cv::Point2f>> pair_trajs=cover.trajs;
    //     // cv::add(_vis_map,cover.vis_map,_vis_map);

    //     output_trajs[group_msg[index].first]=pair_trajs[0];
    //     if(formation_flag)
    //         output_trajs[group_msg[index].second]=pair_trajs[1];

    // }


    // planLoop(all_cover_paths, false, 4, 4,received_local_mines_forAll, received_dangerous_radiuses);

    }
    
    /* 200m */
    // Cover cover(divided_areas[0]);    
    // all_cover_paths =  cover.construct_lozenge_paths(cv::Point2f(600, 900), 300, 200, 300);
    /* all */
    Cover cover(divided_areas[0]);
    cover_path_choice_second = cover.construct_full_path(5000, 1800, cv::Point2f(100, 400), 250, 200, 270, 50, 300);


    bool mode = false;
    if (mode)
    {
        all_cover_paths =  cover.construct_lozenge_paths(cv::Point2f(600, 900), 300,350, 200, 250, 300);
    }
    else
    {
        choose_plan();
    }
    
    

    // std::cout << "planloop_poses_now" << poses_now.size() << std::endl;

    for (int i = 0; i < output_trajs.size(); i++)
    {
        for (int j = 0; j < output_trajs[i].size(); j++)
        {
            cv::circle(vis_map, output_trajs[i][j], 1, cv::Scalar(125, 0, 25), cv::FILLED);
        }
    }
    cv::Mat3b v_map;
     v_map = cv::Mat3b(500, 500);
     v_map.setTo(cv::Scalar(255, 255, 255));
     cv::namedWindow("v_map", cv::WINDOW_NORMAL);
    for (int i = 0; i < divided_areas.size(); i++)
     {
          for (int j = 0; j < divided_areas[i].size(); j++)
          {
               cv::circle(v_map, divided_areas[i][j], 5, cv::Scalar(120, 0, 129), cv::FILLED);
          }
     }
     for (int i = 0; i < output_trajs[3].size(); i++)
     {
         if (i<output_trajs[0].size())
         {
             cv::circle(v_map, output_trajs[0][i], 1, cv::Scalar(120, 118, 129), cv::FILLED);
         }
        if (i<output_trajs[1].size())
         {
             cv::circle(v_map, output_trajs[1][i], 1, cv::Scalar(120, 118, 129), cv::FILLED);
         }
        if (i<output_trajs[2].size())
         {
             cv::circle(v_map, output_trajs[2][i], 1, cv::Scalar(120, 118, 129), cv::FILLED);
         }
         cv::circle(v_map, output_trajs[3][i], 1, cv::Scalar(120, 118, 129), cv::FILLED);
        //  cv::imshow("v_map", v_map);
        // cv::waitKey(50);   // 这里的数字应该就是更新时间
     }
    //  for (int i = 0; i < output_trajs.size(); i++)
    //  {
    //       for (int j = 0; j < output_trajs[i].size(); j++)
    //       {
    //            cv::circle(v_map, output_trajs[i][j], 1, cv::Scalar(120, 118, 129), cv::FILLED);
    //       }
    //  }
    //  cv::imshow("v_map", v_map);
    //  cv::waitKey(0);   // 这里的数字应该就是更新时间

    return output_trajs;

}




/******************************************************************************
//ANCHOR 3.重访算法：
                                                输入：
                                                            位置向量  for_revisit
                                                            速度向量  speed               米/秒
                                                            可视化地图  _vis_map
                                                            剩余时间 time_left          秒
                                                            
                                                输出：
                                                            各艇覆盖阶段轨迹 revisit_trajs
 *****************************************************************************/
    vector<vector<cv::Point>> Module::Module_Solve(std::vector<std::vector<double>> for_revisit, std::vector<double> speed, cv::Mat3b &_vis_map, std::vector<int> cover_rest_time, int time_left, int vehicle_num, std::vector<int> capacity, 
                                                                                                            std::vector<int>WorkTime, std::vector<int> Delivery, std::vector<int> Priority, std::vector<std::vector<int>> Skill, double max_lat, double min_lat){
    vector<vector<cv::Point>> revisit_trajs;

    std::vector<std::vector<int>> TimeWindow;
    // int TimeWindow[2] = {0, time_left};

    int job_num =  for_revisit.size() - vehicle_num * 2;
    // std::cout << "job_num: " << job_num <<std::endl;

    // capacity 载弹量在这里赋了平均值，如果需要准确赋值注释掉循环
    // for (int capa_i = 0; capa_i < vehicle_num; capa_i++) capacity.push_back((int)std::ceil((double)job_num / 4));

    while (1)
    {
        if(Delivery.size() == for_revisit.size() - vehicle_num * 2)  
        {
            cout << "It's OK! " << endl;
            break;
        }
        else
        {
            cout << "ADD Point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;
            Delivery.push_back(0);
            Priority.push_back(1);
            WorkTime.push_back(0);
        }
    }

    for (int i = 0; i < for_revisit.size(); i++)
    {
        if(i < vehicle_num)
        {
            if (cover_rest_time.size() > 0)
            {
                std::vector<int> TW_temp = {cover_rest_time[i], time_left};
                TimeWindow.push_back(TW_temp);
            }
            else
            {
                std::vector<int> TW_temp = {0, time_left};
                TimeWindow.push_back(TW_temp);
            }
        }
        else if(i >= vehicle_num * 2)
        {
            std::vector<int> TW_temp = {0, time_left};
            TimeWindow.push_back(TW_temp);
        }
    }
    

    GET_INFO FROM_VEC("Mat.json", "Location.json", vehicle_num, job_num, TimeWindow, capacity, for_revisit, speed,
                                                WorkTime, Delivery, Priority, Skill);
    FROM_VEC.WriteDataToLocationJson();
    FROM_VEC.WriteDataToMatJson();
        
    std::vector<std::vector<int>> VRP_Paths;
    Problem VRP_Problem(FROM_VEC.MatJson, vehicle_num); 
    try {
        VRP_Paths = VRP_Problem.run_with_custom_matrix(); 
    } 
    catch (const vroom::Exception& e) {
        std::cerr << "[Error] " << e.message << std::endl;
    }

    JSON_RW JSON_RW_1;
    LocationTrans _location_trans;
    vector<vector<LatLngPoint>> routes;
    //读取
    JSON_RW_1.ReadLocationFromJson(FROM_VEC.LocationJson, _location_trans);
    JSON_RW_1.ReadOutputFromJson(_location_trans, routes, VRP_Paths);

    Get_Safe_Place(VRP_Problem.unassigned_mines, _location_trans, safe_place_temp, max_lat, min_lat);
    for (int i = 0; i < VRP_Problem.My_Duration.size(); i++)
    {
        cout << "--------------------------" << VRP_Problem.My_Duration[i] <<endl;
    }
    
    //将route转到CV坐标系下
    vector<vector<cv::Point>> cv_pts(routes.size(),vector<cv::Point>());

    for(int index=0;index<routes.size();index++)
    {
        for(auto &point:routes[index])
        {
            //VRP输出经纬度转cv_point
            cv::Point cv_pt=transfer_.LngAndlat2CV(point);
            cv_pts[index].push_back(cv_pt);
            
            
        }
        cv_pts[index][cv_pts[index].size()-1] = cv::Point(0, 0);
    }
    
    // for (int i = 0; i < cv_pts.size(); i++)
    // {
    //     std::cout << "Vehicle [ " << i << " ]: " << std::endl;
    //     for (int j = 0; j < cv_pts[i].size(); j++)
    //     {
    //         std::cout << cv_pts[i][j].x << ", " << cv_pts[i][j].y << std::endl;
    //     }   
    // }
    
    //TODO 在cv_pts之间形成避障路径,cv的显示范围为500*500 pixels，3个顶层vec对应三条路径
    

   visit_plan(cv_pts, _vis_map);

     /* 计算最大的剩余时间 */
    revisit_rest_time = 0;
    for (int i = 0; i < VRP_Problem.My_Duration.size(); i++)
    {
        if (VRP_Problem.My_Duration[i] >= revisit_rest_time)
        {
            revisit_rest_time = VRP_Problem.My_Duration[i];
        }
    }
    
    /* 计算最后一个雷到终点 */
    float goback_time = 0;
    for (int i = 0; i < vehicle_num; i++)
    {
        if (revisit_trajs_out[i].size()>1)
        {
            float single_goback_time  =  sqrt( pow(revisit_trajs_out[i][revisit_trajs_out[i].size()-1].x - revisit_trajs_out[i][revisit_trajs_out[i].size()-2].x, 2) + pow(revisit_trajs_out[i][revisit_trajs_out[i].size()-1].y - revisit_trajs_out[i][revisit_trajs_out[i].size()-2].y, 2)  ) / 2.5;
            if (single_goback_time  > goback_time)
            {
                goback_time = single_goback_time;
            }
        }
    }
    revisit_rest_time = revisit_rest_time+goback_time;
    

    // revisit_trajs.assign(cv_pts.begin(), cv_pts.end());

    // if (!form_bool)
    // {
    //     for (int i = 0; i < 4; i++)
    //     {
    //         if (vehicle_revisiting[i])
    //         {
    //             revisit_trajs_out[i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[i].push_back(cv::Point2f(210, 370));
    //             // all_cover_paths[i] = revisit_trajs_out[i];
    //             // vehicle_cover_finished[i] = false;
    //             //target_pt_idx[i] = 0;
    //             // for (int j = 0; j < revisit_trajs_out[i].size(); j++)
    //             // {
    //             //     cv::circle(visual_graph, revisit_trajs_out[i][j], 2, cv::Scalar(revisit_trajs_out[i].size()%10*20, 0, 0), cv::FILLED);
    //             // }
    //         }
    //     }
    // }
    // else
    // {
    //     for (int i = 0; i < 2; i++)
    //     {
    //         if (vehicle_revisiting[i])
    //         {
    //             revisit_trajs_out[2*i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[2*i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[2*i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[2*i+1].push_back(cv::Point2f(210, 380));
    //             revisit_trajs_out[2*i+1].push_back(cv::Point2f(210, 380));
    //             revisit_trajs_out[2*i+1].push_back(cv::Point2f(210, 380));
    //             all_cover_paths[i] = revisit_trajs_out[2*i];
    //             // all_cover_paths[i+2] = revisit_trajs_out[2*i+1];
    //             all_cover_paths[i+2] = {cv::Point2f(100, 200), cv::Point2f(100, 200), cv::Point2f(150, 300), cv::Point2f(200, 320), cv::Point2f(290, 350)};
    //             vehicle_cover_finished[i] = false;
    //             target_pt_idx[i] = 1;
    //             target_pt_idx[i+2] = 1;
    //             for (int j = 0; j < all_cover_paths[i].size(); j++)
    //             {
    //                 cv::circle(visual_graph, all_cover_paths[i][j], 2, cv::Scalar(revisit_trajs_out[i].size()%10*20, 0, 0), cv::FILLED);
    //             }
    //             for (int j = 0; j < all_cover_paths[i+2].size(); j++)
    //             {
    //                 cv::circle(visual_graph, all_cover_paths[i+2][j], 2, cv::Scalar(0, revisit_trajs_out[2*i+1].size()%10*20, 0), cv::FILLED);
    //             }
    //             // for (int j = 0; j < revisit_trajs_out[i].size(); j++)
    //             // {
    //             //     cv::circle(visual_graph, revisit_trajs_out[i][j], 2, cv::Scalar(revisit_trajs_out[i].size()%10*20, 0, 0), cv::FILLED);
    //             // }
    //             // for (int j = 0; j < revisit_trajs_out[i+2].size(); j++)
    //             // {
    //             //     cv::circle(visual_graph, revisit_trajs_out[i+2][j], 2, cv::Scalar(0, revisit_trajs_out[2*i+1].size()%10*20, 0), cv::FILLED);
    //             // }
    //         }
    //     }
    // }
    std::cout << "Module Solve Finished! " << std::endl;
    return revisit_trajs;
}

/******************************************************************************
//ANCHOR 3.重访算法：
                                                输入：
                                                            位置向量  for_revisit
                                                            速度向量  speed               米/秒
                                                            可视化地图  _vis_map
                                                            剩余时间 time_left          秒
                                                            
                                                输出：
                                                            各艇覆盖阶段轨迹 revisit_trajs
 *****************************************************************************/
    vector<vector<cv::Point>> Module::Module_Solve_New_Scene(std::vector<std::vector<double>> for_revisit, std::vector<double> speed, cv::Mat3b &_vis_map, std::vector<int> cover_rest_time, int time_left, int vehicle_num, std::vector<int> capacity, 
                                                                                                            std::vector<int>WorkTime, std::vector<int> Delivery, std::vector<int> Priority, std::vector<std::vector<int>> Skill, double max_lat, double min_lat){
        vector<vector<cv::Point>> revisit_trajs;
    // int TimeWindow[2] = {0, time_left};
    std::vector<std::vector<int>> TimeWindow;

    int job_num =  for_revisit.size() - vehicle_num * 2;
    // std::cout << "job_num: " << job_num <<std::endl;

    // capacity 载弹量在这里赋了平均值，如果需要准确赋值注释掉循环
    // for (int capa_i = 0; capa_i < vehicle_num; capa_i++) capacity.push_back((int)std::ceil((double)job_num / 4));

    while (1)
    {
        if(Delivery.size() == for_revisit.size() - vehicle_num * 2)  
        {
            cout << "It's OK! " << endl;
            break;
        }
        else
        {
            cout << "ADD Point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;
            Delivery.push_back(0);
            Priority.push_back(1);
            WorkTime.push_back(0);
        }
    }

    for (int i = 0; i < for_revisit.size(); i++)
    {
        if(i < vehicle_num)
        {
            if (cover_rest_time.size() > 0)
            {
                std::vector<int> TW_temp = {cover_rest_time[i], time_left};
                TimeWindow.push_back(TW_temp);
            }
            else
            {
                std::vector<int> TW_temp = {0, time_left};
                TimeWindow.push_back(TW_temp);
            }
        }
        else if(i >= vehicle_num * 2)
        {
            std::vector<int> TW_temp = {0, time_left};
            TimeWindow.push_back(TW_temp);
        }
    }

    GET_INFO FROM_VEC("Mat_New_Scene.json", "Location_New_Scene.json", vehicle_num, job_num, TimeWindow, capacity, for_revisit, speed,
                                                WorkTime, Delivery, Priority, Skill);
    FROM_VEC.WriteDataToLocationJson();
    FROM_VEC.WriteDataToMatJson();
        
    std::vector<std::vector<int>> VRP_Paths;
    Problem VRP_Problem(FROM_VEC.MatJson, vehicle_num); 
    try {
        VRP_Paths = VRP_Problem.run_with_custom_matrix(); 
    } 
    catch (const vroom::Exception& e) {
        std::cerr << "[Error] " << e.message << std::endl;
    }

    JSON_RW JSON_RW_1;
    LocationTrans _location_trans;
    vector<vector<LatLngPoint>> routes;
    //读取
    JSON_RW_1.ReadLocationFromJson(FROM_VEC.LocationJson, _location_trans);
    JSON_RW_1.ReadOutputFromJson(_location_trans, routes, VRP_Paths);

    Get_Safe_Place(VRP_Problem.unassigned_mines, _location_trans, safe_place_temp, max_lat, min_lat);
    for (int i = 0; i < VRP_Problem.My_Duration.size(); i++)
    {
        // cout << "--------------------------" << VRP_Problem.My_Duration[i] <<endl;
    }
    
    //将route转到CV坐标系下
    vector<vector<cv::Point>> cv_pts(routes.size(),vector<cv::Point>());

    for(int index=0;index<routes.size();index++)
    {
        for(auto &point:routes[index])
        {
            //VRP输出经纬度转cv_point
            cv::Point cv_pt=transfer_.LngAndlat2CV(point);
            cv_pts[index].push_back(cv_pt);
            
            
        }
        cv_pts[index][cv_pts[index].size()-1] = cv::Point(0, 0);
    }
    
    // for (int i = 0; i < cv_pts.size(); i++)
    // {
    //     std::cout << "Vehicle [ " << i << " ]: " << std::endl;
    //     for (int j = 0; j < cv_pts[i].size(); j++)
    //     {
    //         std::cout << cv_pts[i][j].x << ", " << cv_pts[i][j].y << std::endl;
    //     }   
    // }
    
    //TODO 在cv_pts之间形成避障路径,cv的显示范围为500*500 pixels，3个顶层vec对应三条路径
    

   visit_plan(cv_pts, _vis_map);

     /* 计算最大的剩余时间 */
    revisit_rest_time = 0;
    for (int i = 0; i < VRP_Problem.My_Duration.size(); i++)
    {
        if (VRP_Problem.My_Duration[i] >= revisit_rest_time)
        {
            revisit_rest_time = VRP_Problem.My_Duration[i];
        }
    }
    
    /* 计算最后一个雷到终点 */
    float goback_time = 0;
    for (int i = 0; i < vehicle_num; i++)
    {
        if (revisit_trajs_out[i].size()>1)
        {
            float single_goback_time  =  sqrt( pow(revisit_trajs_out[i][revisit_trajs_out[i].size()-1].x - revisit_trajs_out[i][revisit_trajs_out[i].size()-2].x, 2) + pow(revisit_trajs_out[i][revisit_trajs_out[i].size()-1].y - revisit_trajs_out[i][revisit_trajs_out[i].size()-2].y, 2)  ) / 2.5;
            if (single_goback_time  > goback_time)
            {
                goback_time = single_goback_time;
            }
        }
    }
    revisit_rest_time = revisit_rest_time+goback_time;
    

    // revisit_trajs.assign(cv_pts.begin(), cv_pts.end());

    // if (!form_bool)
    // {
    //     for (int i = 0; i < 4; i++)
    //     {
    //         if (vehicle_revisiting[i])
    //         {
    //             revisit_trajs_out[i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[i].push_back(cv::Point2f(210, 370));
    //             // all_cover_paths[i] = revisit_trajs_out[i];
    //             // vehicle_cover_finished[i] = false;
    //             //target_pt_idx[i] = 0;
    //             // for (int j = 0; j < revisit_trajs_out[i].size(); j++)
    //             // {
    //             //     cv::circle(visual_graph, revisit_trajs_out[i][j], 2, cv::Scalar(revisit_trajs_out[i].size()%10*20, 0, 0), cv::FILLED);
    //             // }
    //         }
    //     }
    // }
    // else
    // {
    //     for (int i = 0; i < 2; i++)
    //     {
    //         if (vehicle_revisiting[i])
    //         {
    //             revisit_trajs_out[2*i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[2*i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[2*i].push_back(cv::Point2f(210, 370));
    //             revisit_trajs_out[2*i+1].push_back(cv::Point2f(210, 380));
    //             revisit_trajs_out[2*i+1].push_back(cv::Point2f(210, 380));
    //             revisit_trajs_out[2*i+1].push_back(cv::Point2f(210, 380));
    //             all_cover_paths[i] = revisit_trajs_out[2*i];
    //             // all_cover_paths[i+2] = revisit_trajs_out[2*i+1];
    //             all_cover_paths[i+2] = {cv::Point2f(100, 200), cv::Point2f(100, 200), cv::Point2f(150, 300), cv::Point2f(200, 320), cv::Point2f(290, 350)};
    //             vehicle_cover_finished[i] = false;
    //             target_pt_idx[i] = 1;
    //             target_pt_idx[i+2] = 1;
    //             for (int j = 0; j < all_cover_paths[i].size(); j++)
    //             {
    //                 cv::circle(visual_graph, all_cover_paths[i][j], 2, cv::Scalar(revisit_trajs_out[i].size()%10*20, 0, 0), cv::FILLED);
    //             }
    //             for (int j = 0; j < all_cover_paths[i+2].size(); j++)
    //             {
    //                 cv::circle(visual_graph, all_cover_paths[i+2][j], 2, cv::Scalar(0, revisit_trajs_out[2*i+1].size()%10*20, 0), cv::FILLED);
    //             }
    //             // for (int j = 0; j < revisit_trajs_out[i].size(); j++)
    //             // {
    //             //     cv::circle(visual_graph, revisit_trajs_out[i][j], 2, cv::Scalar(revisit_trajs_out[i].size()%10*20, 0, 0), cv::FILLED);
    //             // }
    //             // for (int j = 0; j < revisit_trajs_out[i+2].size(); j++)
    //             // {
    //             //     cv::circle(visual_graph, revisit_trajs_out[i+2][j], 2, cv::Scalar(0, revisit_trajs_out[2*i+1].size()%10*20, 0), cv::FILLED);
    //             // }
    //         }
    //     }
    // }
    return revisit_trajs;
}

/******************************************************************************
//ANCHOR 直接到达目的地：
                                                输入：
                                                           其实位置、终止位置、周边雷区、周边禁止进入区域
                                                输出：
                                                            轨迹点序列
 *****************************************************************************/
std::vector<cv::Point2f> Module::go_to_target(cv::Point start_pt, cv::Point target_pt, std::vector<cv::Point> local_mine_pos, std::vector<cv::Point> forbidden_area_pos)
{
    std::vector<cv::Point2f> part_trajs;
    cv::Point ctrl_pt0, ctrl_pt1, ctrl_pt2,ctrl_pt3;
    ctrl_pt0 = start_pt;
    ctrl_pt1 = start_pt;
    ctrl_pt2 = start_pt;
    while ( pow(target_pt.x - ctrl_pt2.x, 2) + pow(target_pt.y - ctrl_pt2.y, 2) > 3*3)
    {
        ctrl_pt3 = construct_Bspline(ctrl_pt0, ctrl_pt1, ctrl_pt2, target_pt, local_mine_pos, forbidden_area_pos);
        for (float t = 0.0; t < 1.0; t=t+0.2)
        {
            float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
            float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
            part_trajs.push_back(cv::Point2f(traj_pt_x, traj_pt_y));
        }
        ctrl_pt0 = ctrl_pt1;
        ctrl_pt1 = ctrl_pt2;
        ctrl_pt2 = ctrl_pt3;
    }
    ctrl_pt3 = target_pt;
    for (float t = 0; t < 1.0; t = t+0.2)
    {
        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
        part_trajs.push_back(cv::Point2f(traj_pt_x, traj_pt_y));
    }
    ctrl_pt0 = ctrl_pt1;
    ctrl_pt1 = ctrl_pt2;
    ctrl_pt2 = ctrl_pt3;
    for (float t = 0; t < 1.0; t = t+0.2)
    {
        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
        part_trajs.push_back(cv::Point2f(traj_pt_x, traj_pt_y));
    }
    ctrl_pt0 = ctrl_pt1;
    ctrl_pt1 = ctrl_pt2;
    ctrl_pt2 = ctrl_pt3;
    for (float t = 0; t < 1.0; t = t+0.2)
    {
        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
        part_trajs.push_back(cv::Point2f(traj_pt_x, traj_pt_y));
    }

    // cv::Mat3b mod_goto_map;
    // mod_goto_map = cv::Mat3b(500, 500);
    // mod_goto_map.setTo(cv::Scalar(255, 255, 255));

    for (int i = 0; i < part_trajs.size(); i++)
    {
        cv::circle(vis_map, part_trajs[i], 1, cv::Scalar(125, 0, 25), cv::FILLED);
    }
    
    /* 这里后续可以考虑把mod_goto_map拼进大地图里 */
    // cv::namedWindow("part_traj", cv::WINDOW_NORMAL);
    // cv::imshow("part_traj", mod_goto_map);
    // cv::waitKey(0);

    return part_trajs;
}


/******************************************************************************
//ANCHOR 直接到达目的地的调用函数：
 *****************************************************************************/
/* 计算B样条曲线的控制点*/
cv::Point Module::construct_Bspline(cv::Point ctrl0, cv::Point ctrl1, cv::Point ctrl2, cv::Point target, std::vector<cv::Point> local_mines_pos, std::vector<cv::Point> forbidden_area)
{
    std::vector<cv::Point> local_mines; // 障碍区域中心
    std::vector<int> dangerous_area;    // 杀伤范围
    for (int i = 0; i < local_mines_pos.size(); i++)
    {
        local_mines.push_back(local_mines_pos[i]);
        dangerous_area.push_back(5);    // 障碍杀伤范围
    }
    for (int i = 0; i < forbidden_area.size(); i++)
    {
        local_mines.push_back(forbidden_area[i]);
        dangerous_area.push_back(10);   // 禁止驶入范围
    }
    
    /* 首选速度 */
    float pref_vel_x = ( (float)target.x - (float)ctrl2.x ) / sqrt( pow( ((float)target.x - (float)ctrl2.x), 2 ) + pow( ((float)target.y - (float)ctrl2.y), 2 ) );
    float pref_vel_y = ( (float)target.y - (float)ctrl2.y ) / sqrt( pow( ((float)target.x - (float)ctrl2.x), 2 ) + pow( ((float)target.y - (float)ctrl2.y), 2 ) );

    /* 将首选速度推出障碍集 */
    float vel_off_k1_x, vel_off_k1_y;   // 从k1侧推出
    float vel_off_k2_x, vel_off_k2_y;   // 从k2侧推出
    vel_off_k1_x = pref_vel_x;
    vel_off_k1_y = pref_vel_y;
    vel_off_k2_x = pref_vel_x;
    vel_off_k2_y = pref_vel_y;
    for (int i = 0; i < local_mines.size(); i++)
    {
        /* --- 1、计算切线 */
        float dist_boat_mine = sqrt( pow((float)local_mines[i].x - (float)ctrl2.x, 2) + pow((float)local_mines[i].y - (float)ctrl2.y, 2) );
        float angle_beta = asin((float)dangerous_area[i]/dist_boat_mine); //切线1/2夹角
        float angle_alpha;  // 方向角
        if ( (float)local_mines[i].x != (float)ctrl2.x )
        {
            angle_alpha = atan( ((float)local_mines[i].y - (float)ctrl2.y) / ((float)local_mines[i].x - (float)ctrl2.x) );
        }
        else
        {
            angle_alpha = 0.5*PI;
        }
        float k1 = angle_alpha + angle_beta;
        float k2 = angle_alpha - angle_beta;

        
        /* --- 2、判断几何情况 */
        float vel1_on_k1, vel1_on_k2, vel2_on_k1, vel2_on_k2; // 速度在障碍集边界的哪一侧
        float mine_on_k1, mine_on_k2;    // 障碍在障碍集边界的哪一侧
        float k1_tan = tan(k1);
        if ( !isnan(k1_tan) )
        {
            vel1_on_k1 = k1_tan*vel_off_k1_x - vel_off_k1_y;
            vel2_on_k1 = k1_tan*vel_off_k2_x - vel_off_k2_y;
            mine_on_k1 = k1_tan * ( (float)local_mines[i].x - (float)ctrl2.x ) -  ( (float)local_mines[i].y - (float)ctrl2.y );
        }
        else
        {
            // vel1_on_k1 = vel_off_k1_y;
            // vel2_on_k1 = vel_off_k2_y;
            // mine_on_k1 = (float)local_mines[i].y - (float)ctrl2.y;
            k1_tan = tan(1.57);
            vel1_on_k1 = k1_tan*vel_off_k1_x - vel_off_k1_y;
            vel2_on_k1 = k1_tan*vel_off_k2_x - vel_off_k2_y;
            mine_on_k1 = k1_tan * ( (float)local_mines[i].x - (float)ctrl2.x ) -  ( (float)local_mines[i].y - (float)ctrl2.y );
        }
        float k2_tan = tan(k2);
        if ( !isnan(k1_tan) )
        {
            vel1_on_k2 = k2_tan*vel_off_k1_x - vel_off_k1_y;
            vel2_on_k2 = k2_tan*vel_off_k2_x - vel_off_k2_y;
            mine_on_k2 = k2_tan * ( (float)local_mines[i].x - (float)ctrl2.x ) -  ( (float)local_mines[i].y - (float)ctrl2.y );
        }
        else
        {
            // vel1_on_k2 = vel_off_k1_y;
            // vel2_on_k2 = vel_off_k2_y;
            // mine_on_k2 = (float)local_mines[i].y - (float)ctrl2.y;
            k2_tan = tan(1.57);
            vel1_on_k2 = k2_tan*vel_off_k1_x - vel_off_k1_y;
            vel2_on_k2 = k2_tan*vel_off_k2_x - vel_off_k2_y;
            mine_on_k2 = k2_tan * ( (float)local_mines[i].x - (float)ctrl2.x ) -  ( (float)local_mines[i].y - (float)ctrl2.y );
        }

        /* --- 3、推出速度障碍集 */
        if ( (vel1_on_k1*mine_on_k1>0)&&(vel1_on_k2*mine_on_k2>0) )
        {
            float vel_x = ( vel_off_k1_x + k1_tan*vel_off_k1_y )/(1 + k1_tan*k1_tan);
            float vel_y = ( k1_tan*vel_off_k1_x + k1_tan*k1_tan*vel_off_k1_y )/(1 + k1_tan*k1_tan);
            vel_off_k1_x = vel_x / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
            vel_off_k1_y = vel_y / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
        }
        if ( (vel2_on_k1*mine_on_k1>0)&&(vel2_on_k2*mine_on_k2>0) )
        {
            float vel_x = ( vel_off_k2_x + k2_tan*vel_off_k2_y)/(1 + k2_tan*k2_tan);
            float vel_y = ( k1_tan*vel_off_k2_x + k2_tan*k2_tan*vel_off_k2_y)/(1 + k2_tan*k2_tan);
            vel_off_k2_x = vel_x / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
            vel_off_k2_y = vel_y / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
        }
    }
    
    /* 选择较近的速度 */
    float dist_vk1 = pow(vel_off_k1_x - pref_vel_x, 2) + pow(vel_off_k1_y - pref_vel_y, 2);
    float dist_vk2 = pow(vel_off_k2_x - pref_vel_x, 2) + pow(vel_off_k2_y - pref_vel_y, 2);
    float vel_opt_x, vel_opt_y;
    if (dist_vk1 < dist_vk2)
    {
        vel_opt_x = vel_off_k1_x;
        vel_opt_y = vel_off_k1_y;
    }
    else
    {
        vel_opt_x = vel_off_k2_x;
        vel_opt_y = vel_off_k2_y;
    }

    /* 生成B样条曲线 */
    float deltaT = 5;
    cv::Point ctrl3;
    ctrl3.x = ctrl2.x + vel_opt_x*deltaT;
    ctrl3.y = ctrl2.y + vel_opt_y*deltaT;
    return ctrl3;
}






/*******************************************************
 * 
 * 新增函数
 * 
  *******************************************************/

 /* 初始化 */
void Module::plannerInit(int vehicle_num, std::vector<cv::Point2f> start_pos)
{
    /* 变量初始化 */
    target_pt_idx.resize(vehicle_num, 0);
    cover_processing = true;
    control_pts_now.resize(vehicle_num);
    control_pts_buffer.resize(vehicle_num);
    vehicle_cover_finished.resize(vehicle_num, false);
    received_local_mines_forAll.resize(vehicle_num);
    received_dangerous_radiuses.resize(vehicle_num);
    poses_now.resize(vehicle_num);
    vehicle_revisiting.resize(vehicle_num, false);
    all_cover_paths.resize(vehicle_num);
    revisit_trajs_out.resize(4);
    trajectory_output.resize(4);
    going_to_start = false;
    real_poses.resize(vehicle_num);
    cover_path_choice_first.resize(vehicle_num);
    cover_path_choice_second.resize(vehicle_num);
    vehicle_heading.resize(vehicle_num);
    Eliminate_Mines_In_Cover.resize(vehicle_num);
    Eliminate_Cars_In_Cover.resize(vehicle_num, false);
    last_vels_x.resize(vehicle_num, 1.41421/2.0);
    last_vels_y.resize(vehicle_num, 1.41421/2.0);

    /* 可视化初始化 */
    visual_graph = cv::Mat3b(800, 800);
    visual_graph.setTo(cv::Scalar(255, 255, 255));
    cv::namedWindow("visualizaiton", cv::WINDOW_NORMAL);

    /* 出发控制点初始化 */
    for (int i = 0; i < vehicle_num; i++)
    {
        control_pts_now[i].push_back(start_pos[i]);
        control_pts_now[i].push_back(start_pos[i]);
        control_pts_now[i].push_back(start_pos[i]);

        // finished 标志
        vehicle_cover_finished_for_revisit.push_back(false);
         vehicle_revisit_finished_for_end.push_back(false);
    }

    

    gloable_mines = {cv::Point2f(300, 100), cv::Point2f(182, 110), cv::Point2f(264, 100), cv::Point2f(178, 150), cv::Point2f(164, 176), cv::Point2f(351, 115), cv::Point2f(288, 148), cv::Point2f(321, 138), cv::Point2f(164, 81) /*, cv::Point2f(349, 89), cv::Point2f(234, 65)*/ };
    for (int i = 0; i < gloable_mines.size(); i++)
    {
        cv::circle(visual_graph, gloable_mines[i], 5, cv::Scalar(30, 20, 10), cv::FILLED);
        
        gloable_mines_real.push_back(cv::Point2f(gloable_mines[i].x*10, gloable_mines[i].y*10));

        // 重复障碍标志位
        mines_check.push_back(false);
    }
    virtual_mines_check.resize(virtual_gloable_mines_real.size(), false);   // 虚雷重复标志位

} 

/*********************************************************************
 * 判断编队附近是否有雷区：
 *      输入：
 *          leader位置，follower位置；
 *          附近雷区；
**********************************************************************/
bool Module::mines_near(cv::Point2f target_pos,  std::vector<cv::Point2f> mines)
{
    bool flag = false;
    for (int i = 0; i < mines.size(); i++)
    {
        if ( pow(target_pos.x - mines[i].x, 2)+pow(target_pos.y - mines[i].y, 2) < 15*15 )
        {
            flag = true;
        }
    }
    return flag;
}

/*********************************************************************
 * 判断目标点是否在雷区内：
 *      输入：
 *          目标点位置；
 *          附近雷区；
**********************************************************************/
bool Module::inside_mines(cv::Point2f target_pt, std::vector<cv::Point2f> mines)
{
    bool flag = false;
    for (int i = 0; i < mines.size(); i++)
    {
        if ( pow(target_pt.x - mines[i].x, 2)+pow(target_pt.y - mines[i].y, 2) < 160*160 )
        {
            flag = true;
        }
    }
    return flag;
}

/*********************************************************************
 * B样条曲线计算：
 *      输入：
 *          当前控制点0，1，2；
 *          目标点；
 *          障碍点，对应范围；
 *      输出：
 *          控制点3；
 *          一段B样条曲线点；
**********************************************************************/
std::vector<cv::Point2f> Module::construct_Bspline(cv::Point2f ctrl_pt0, cv::Point2f ctrl_pt1, cv::Point2f ctrl_pt2, cv::Point2f target_pt, std::vector<cv::Point2f> local_mines, std::vector<int> dangerous_area, float sim_vel)
{ 
    std::vector<cv::Point2f> Bspline_result;

    /* 首选速度 */
    float pref_vel_x = ( target_pt.x - ctrl_pt2.x ) / sqrt( pow( (target_pt.x - ctrl_pt2.x), 2 ) + pow( (target_pt.y - ctrl_pt2.y), 2 ) );
    float pref_vel_y = ( target_pt.y - ctrl_pt2.y ) / sqrt( pow( (target_pt.x - ctrl_pt2.x), 2 ) + pow( (target_pt.y - ctrl_pt2.y), 2 ) );

    /* 将首选速度推出障碍集 */
    float vel_off_k1_x, vel_off_k1_y;   // 从k1侧推出
    float vel_off_k2_x, vel_off_k2_y;   // 从k2侧推出
    vel_off_k1_x = pref_vel_x;
    vel_off_k1_y = pref_vel_y;
    vel_off_k2_x = pref_vel_x;
    vel_off_k2_y = pref_vel_y;
    for (int i = 0; i < local_mines.size(); i++)
    {

        if (pow(ctrl_pt2.x - local_mines[i].x, 2)+pow(ctrl_pt2.y - local_mines[i].y, 2)<dangerous_area[i])
        {
            dangerous_area[i] = 0.0001;
        }

        /* --- 1、计算切线 */
        float dist_boat_mine = sqrt( pow(local_mines[i].x - ctrl_pt2.x, 2) + pow(local_mines[i].y - ctrl_pt2.y, 2) );
        float angle_beta = asin((float)dangerous_area[i]/dist_boat_mine); //切线1/2夹角
        float angle_alpha;  // 方向角
        if ( local_mines[i].x != ctrl_pt2.x )   
        {
            angle_alpha = atan( (local_mines[i].y - ctrl_pt2.y) / (local_mines[i].x - ctrl_pt2.x) );
        }
        else    // 90度
        {
            angle_alpha = 0.5*PI;
        }
        float k1 = angle_alpha + angle_beta;
        float k2 = angle_alpha - angle_beta;

        
        /* --- 2、判断几何情况 */
        float vel1_on_k1, vel1_on_k2, vel2_on_k1, vel2_on_k2; // 速度在障碍集边界的哪一侧
        float mine_on_k1, mine_on_k2;    // 障碍在障碍集边界的哪一侧
        float k1_tan = tan(k1);
        if ( !isnan(k1_tan) )
        {
            vel1_on_k1 = k1_tan*vel_off_k1_x - vel_off_k1_y;
            vel2_on_k1 = k1_tan*vel_off_k2_x - vel_off_k2_y;
            mine_on_k1 = k1_tan * ( local_mines[i].x - ctrl_pt2.x ) -  ( local_mines[i].y - ctrl_pt2.y );
        }
        else    // 90度
        {
            k1_tan = tan(1.57);
            vel1_on_k1 = k1_tan*vel_off_k1_x - vel_off_k1_y;
            vel2_on_k1 = k1_tan*vel_off_k2_x - vel_off_k2_y;
            mine_on_k1 = k1_tan * ( local_mines[i].x - ctrl_pt2.x ) -  ( local_mines[i].y - ctrl_pt2.y );
        }
        float k2_tan = tan(k2);
        if ( !isnan(k2_tan) )
        {
            vel1_on_k2 = k2_tan*vel_off_k1_x - vel_off_k1_y;
            vel2_on_k2 = k2_tan*vel_off_k2_x - vel_off_k2_y;
            mine_on_k2 = k2_tan * ( local_mines[i].x - ctrl_pt2.x ) -  ( local_mines[i].y - (float)ctrl_pt2.y );
        }
        else
        {
            k2_tan = tan(1.57);
            vel1_on_k2 = k2_tan*vel_off_k1_x - vel_off_k1_y;
            vel2_on_k2 = k2_tan*vel_off_k2_x - vel_off_k2_y;
            mine_on_k2 = k2_tan * ( local_mines[i].x - ctrl_pt2.x ) -  ( local_mines[i].y - ctrl_pt2.y );
        }

        /* --- 3、推出速度障碍集 */
        if ( (vel1_on_k1*mine_on_k1>0)&&(vel1_on_k2*mine_on_k2>0) )
        {
            float vel_x = ( vel_off_k1_x + k1_tan*vel_off_k1_y )/(1 + k1_tan*k1_tan);
            float vel_y = ( k1_tan*vel_off_k1_x + k1_tan*k1_tan*vel_off_k1_y )/(1 + k1_tan*k1_tan);
            vel_off_k1_x = vel_x / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
            vel_off_k1_y = vel_y / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
        }
        if ( (vel2_on_k1*mine_on_k1>0)&&(vel2_on_k2*mine_on_k2>0) )
        {
            float vel_x = ( vel_off_k2_x + k2_tan*vel_off_k2_y)/(1 + k2_tan*k2_tan);
            float vel_y = ( k2_tan*vel_off_k2_x + k2_tan*k2_tan*vel_off_k2_y)/(1 + k2_tan*k2_tan);
            vel_off_k2_x = vel_x / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
            vel_off_k2_y = vel_y / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
        }
    }
    
    /* 选择较近的速度 */
    float dist_vk1 = pow(vel_off_k1_x - pref_vel_x, 2) + pow(vel_off_k1_y - pref_vel_y, 2);
    float dist_vk2 = pow(vel_off_k2_x - pref_vel_x, 2) + pow(vel_off_k2_y - pref_vel_y, 2);
    float vel_opt_x, vel_opt_y;
    if (dist_vk1 < dist_vk2)
    {
        vel_opt_x = vel_off_k1_x;
        vel_opt_y = vel_off_k1_y;
    }
    else
    {
        vel_opt_x = vel_off_k2_x;
        vel_opt_y = vel_off_k2_y;
    }

    /* 限定转向 */
    // double angle_now = acos(vel_opt_x*last_vel_x + vel_opt_y*last_vel_y);
    // double largest_angle = 1.57;
    // double angle_opt_now;
    // if (vel_opt_x!=0)
    // {
    //     angle_opt_now = atan(vel_opt_y/vel_opt_x);
    // }
    // else
    // {
    //     angle_opt_now = 1.57;
    // }
    // if (angle_now > largest_angle)
    // {
    //     std::cout << "1111111111" << std::endl;
    //     float vel_x_1 = ( last_vel_x + tan(angle_opt_now+largest_angle)*last_vel_y)/(1 + tan(angle_opt_now+largest_angle)*tan(angle_opt_now+largest_angle));
    //     float vel_y_1 = ( tan(angle_opt_now+largest_angle)*last_vel_x + tan(angle_opt_now+largest_angle)*tan(angle_opt_now+largest_angle)*last_vel_y)/(1 + tan(angle_opt_now+largest_angle)*tan(angle_opt_now+largest_angle));

    //     float vel_x_2 = ( last_vel_x + tan(angle_opt_now-largest_angle)*last_vel_y)/(1 + tan(angle_opt_now-largest_angle)*tan(angle_opt_now-largest_angle));
    //     float vel_y_2 = ( tan(angle_opt_now-largest_angle)*last_vel_x + tan(angle_opt_now-largest_angle)*tan(angle_opt_now-largest_angle)*last_vel_y)/(1 + tan(angle_opt_now-largest_angle)*tan(angle_opt_now-largest_angle));

    //     if ( pow(vel_opt_x-vel_x_1, 2)+pow(vel_opt_y-vel_y_1, 2) > pow(vel_opt_x-vel_x_2, 2)+pow(vel_opt_y-vel_y_2, 2)  )
    //     {
    //         vel_opt_x = vel_x_2;
    //         vel_opt_y = vel_y_2;
    //     }
    //     else
    //     {
    //         vel_opt_x = vel_x_1;
    //         vel_opt_y = vel_y_1;
    //     }
    // }
    // last_vel_x = vel_opt_x;
    // last_vel_y = vel_opt_y;

    /* 生成B样条曲线 */
    float deltaT = sim_vel;
    cv::Point ctrl_pt3;
    if ( pow(ctrl_pt2.x - target_pt.x, 2)+pow(ctrl_pt2.y - target_pt.y, 2)>sim_vel*sim_vel )
    {
        ctrl_pt3.x = ctrl_pt2.x + vel_opt_x*deltaT;
        ctrl_pt3.y = ctrl_pt2.y + vel_opt_y*deltaT;
    }
    else
    {
        ctrl_pt3 = target_pt;
    }
    
    Bspline_result.push_back(ctrl_pt3);

    for (float t = 0.0; t < 1.0; t=t+0.1)
    {
        float traj_pt_x = (ctrl_pt0.x*(1.0-t)*(1.0-t)*(1.0-t) + ctrl_pt1.x*(3.0*t*t*t - 6.0*t*t + 4.0) + ctrl_pt2.x*(-3.0*t*t*t + 3.0*t*t + 3.0*t + 1.0) + ctrl_pt3.x*t*t*t)/6.0;
        float traj_pt_y = (ctrl_pt0.y*(1.0-t)*(1.0-t)*(1.0-t) + ctrl_pt1.y*(3.0*t*t*t - 6.0*t*t + 4.0) + ctrl_pt2.y*(-3.0*t*t*t + 3.0*t*t + 3.0*t + 1.0) + ctrl_pt3.y*t*t*t)/6.0;
        Bspline_result.push_back(cv::Point2f(traj_pt_x, traj_pt_y));
    }
    
    return Bspline_result;
}

/*********************************************************************
 * 轨迹生成：
 *      输入：
 *          当前控制点0，1，2；
 *          目标点；
 *          障碍点，对应范围；
 *      输出：
 *          一段轨迹点；
 *          一段控制点；
**********************************************************************/
std::vector<std::vector<cv::Point2f>>Module::construct_trajectory(cv::Point2f ctrl_pt0, cv::Point2f ctrl_pt1, cv::Point2f ctrl_pt2, cv::Point2f target_pt, std::vector<cv::Point2f> local_mines, std::vector<int> dangerous_area, float sim_vel)
{
    std::vector<std::vector<cv::Point2f>> trajectory_result;
    trajectory_result.resize(2);
    trajectory_result[1].push_back(ctrl_pt0);
    trajectory_result[1].push_back(ctrl_pt1);
    trajectory_result[1].push_back(ctrl_pt2);

    /* 前往短期终点 */
    int t = 0;   // 更新次数
    while ( pow(target_pt.x - ctrl_pt2.x, 2) + pow(target_pt.y - ctrl_pt2.y, 2) > sim_vel*sim_vel && t < 10)
    {
        std::vector<cv::Point2f> Bspline_result;
        Bspline_result = construct_Bspline(ctrl_pt0, ctrl_pt1, ctrl_pt2, target_pt, local_mines, dangerous_area, sim_vel);
        ctrl_pt0 = ctrl_pt1;
        ctrl_pt1 = ctrl_pt2;
        ctrl_pt2 = Bspline_result[0];
        trajectory_result[1].push_back(Bspline_result[0]);
        for (int i = 1; i < Bspline_result.size(); i++)
        {
            trajectory_result[0].push_back(Bspline_result[i]);
        }
        t++;
        // std::cout << "t=" << t << std::endl;
        // std::cout << "2222222222222222222222222" << std::endl;
    }
    std::vector<cv::Point2f> Bspline_result;
    Bspline_result = construct_Bspline(ctrl_pt0, ctrl_pt1, ctrl_pt2, target_pt, local_mines, dangerous_area, sim_vel);
    ctrl_pt0 = ctrl_pt1;
    ctrl_pt1 = ctrl_pt2;
    ctrl_pt2 = Bspline_result[0];
    trajectory_result[1].push_back(Bspline_result[0]);
    for (int i = 1; i < Bspline_result.size(); i++)
    {
        trajectory_result[0].push_back(Bspline_result[i]);
    }

    return trajectory_result;
}

/*********************************************************************
 * 编队控制：
 *      输入：
 *          当前follower位置，follower目标位置；
 *      输出：
 *          follower一个轨迹点
**********************************************************************/
cv::Point2f Module::follow_leader(cv::Point2f follower_pos, cv::Point2f target_pos)
{
    float vel_x, vel_y;
    cv::Point go_pos;
    vel_x = target_pos.x - follower_pos.x;
    vel_y = target_pos.y - follower_pos.y;
    if (vel_x==0.0 && vel_y==0.0)
    {
        go_pos = follower_pos;
    }
    else
    {
        float follower_vel_x = vel_x / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
        float follower_vel_y = vel_y / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
    
        float update_time = 1.0;
        go_pos.x = (float)follower_pos.x + vel_x * update_time;
        go_pos.y = (float)follower_pos.y + vel_y * update_time;
    }
    return go_pos;  // 下一时刻follower的位置
}



void Module::visit_plan(std::vector<std::vector<cv::Point>> target_pos,cv::Mat3b &vis_map)
{
    
    /* 构建可视化地图 */
    // cv::Mat3b vis_map;
    // vis_map = cv::Mat3b(500, 700);
    // vis_map.setTo(cv::Scalar(255, 255, 255));
        
    /* 对每艘艇进行处理 */
    for (int i = 0; i < target_pos.size(); i++)
    {
        /* 计算参考路径点 */
        
        std::vector<cv::Point2f> traj_result;

        // std::vector<cv::Point> path_pt;
        // int seg_length = 20;    // 每段路径的参考长度
        for (int j = 0; j < target_pos[i].size(); j++)    // 进行路径划分
        {
            // int seg_num = 1 + sqrt( pow( (target_pos[i][j+1].x - target_pos[i][j].x), 2 ) + pow( (target_pos[i][j+1].y - target_pos[i][j].y), 2 ) ) / seg_length;   // 两目标间路径分段
            // for (int m = 1; m < seg_num; m++)   // 生成路径点
            // {
            //     int path_pt_x = target_pos[i][j].x + m*(target_pos[i][j+1].x - target_pos[i][j].x)/seg_num;
            //     int path_pt_y = target_pos[i][j].y + m*(target_pos[i][j+1].y - target_pos[i][j].y)/seg_num;
            //     path_pt.push_back(cv::Point(path_pt_x, path_pt_y));
            //     traj_result.push_back(cv::Point2f(path_pt_x, path_pt_y));
            // }

            traj_result.push_back(cv::Point2f(target_pos[i][j].x, target_pos[i][j].y));
        }
        revisit_trajs_out[i] = traj_result;
    }

    // cv::namedWindow("map1", cv::WINDOW_NORMAL);
    // cv::imshow("map1", vis_map);
}

void Module::test_revist()
{
    if (!form_bool)
    {
        for (int i = 0; i < 4; i++)
        {
            if (vehicle_revisiting[i])
            {
                // all_cover_paths[i] = revisit_trajs_out[i];
                vehicle_cover_finished[i] = false;
                target_pt_idx[i] = 1;                
            }
        }
    }
    else
    {
        // for (int i = 0; i < 2; i++)
        // {
        //     if (vehicle_revisiting[i])
        //     {
        //         all_cover_paths[i] = {cv::Point2f(200, 200), cv::Point2f(240, 170), cv::Point2f(230, 140), cv::Point2f(170, 320), cv::Point2f(248, 335)};;
        //         all_cover_paths[i+2] = {cv::Point2f(100, 200), cv::Point2f(100, 200), cv::Point2f(150, 300), cv::Point2f(200, 320), cv::Point2f(290, 350)};
        //         vehicle_cover_finished[i] = false;
        //         target_pt_idx[i] = 1;
        //         target_pt_idx[i+2] = 1;
        //         for (int j = 0; j < all_cover_paths[i].size(); j++)
        //         {
        //             cv::circle(visual_graph, all_cover_paths[i][j], 3, cv::Scalar(232, 0, 0), cv::FILLED);
        //         }
        //         for (int j = 0; j < all_cover_paths[i+2].size(); j++)
        //         {
        //             cv::circle(visual_graph, all_cover_paths[i+2][j], 3, cv::Scalar(0, 232, 0), cv::FILLED);
        //         }
        //     }
        // }
    }
}

/***********************************************************
 * 到达确定点函数
 *      输入：
 *          目标位置；
 *          船索引；
 *          其他水雷位置及半径；
 ***********************************************************/
void Module::planloop_approach(cv::Point2f target_pos, int vehicle_idx, std::vector<cv::Point2f> other_mines, std::vector<int> other_mines_radius, float sim_vel)
{
    // last_vel_x = last_vels_x[vehicle_idx];
    // last_vel_y = last_vels_y[vehicle_idx];
    // std::cout << "other_mines: " << other_mines.size() << std::endl;		
    int update_times = 10;  // 一段轨迹内轨迹点的更新次数   
    std::vector<cv::Point2f> traj_buffer;   
    for (int i = 0; i < all_cover_paths[0].size(); i++)
    {
        cv::circle(visual_graph, all_cover_paths[0][i], 3, cv::Scalar(0, 240, 0), cv::FILLED);
    }

    if ( pow(poses_now[vehicle_idx].x-real_poses[vehicle_idx].x, 2)+pow(poses_now[vehicle_idx].y-real_poses[vehicle_idx].y, 2)<1 )
    {
         /* 生成一段轨迹 */
        std::vector<std::vector<cv::Point2f>> traj_result = construct_trajectory(control_pts_now[vehicle_idx][0], control_pts_now[vehicle_idx][1], control_pts_now[vehicle_idx][2], target_pos, other_mines, other_mines_radius, sim_vel);
        traj_buffer = traj_result[0];
        control_pts_buffer[vehicle_idx] = traj_result[1];

        /* 更新轨迹点 */  
        trajectory_output[vehicle_idx].clear();     
        for (int i = 0; i < update_times; i++)
        {
            trajectory_output[vehicle_idx].push_back(traj_buffer[i]);
        }
            
        /* 计算下一轮的一段轨迹 */
        control_pts_now[vehicle_idx][0] = control_pts_buffer[vehicle_idx][update_times/10];
        control_pts_now[vehicle_idx][1] = control_pts_buffer[vehicle_idx][update_times/10+1];
        control_pts_now[vehicle_idx][2] = control_pts_buffer[vehicle_idx][update_times/10+2];
    }
    else
    {
        /* 生成一段轨迹 */
        std::vector<std::vector<cv::Point2f>> traj_result = construct_trajectory(real_poses[vehicle_idx], real_poses[vehicle_idx], real_poses[vehicle_idx], target_pos, other_mines, other_mines_radius, sim_vel);
        traj_buffer = traj_result[0];
        control_pts_buffer[vehicle_idx] = traj_result[1];
          
        /* 计算下一轮的一段轨迹 */
        control_pts_now[vehicle_idx][0] = control_pts_buffer[vehicle_idx][update_times/10];
        control_pts_now[vehicle_idx][1] = control_pts_buffer[vehicle_idx][update_times/10+1];
        control_pts_now[vehicle_idx][2] = control_pts_buffer[vehicle_idx][update_times/10+2];

        /* 生成一段轨迹 */
        traj_result = construct_trajectory(control_pts_now[vehicle_idx][0], control_pts_now[vehicle_idx][1], control_pts_now[vehicle_idx][2], target_pos, other_mines, other_mines_radius, sim_vel);
        traj_buffer = traj_result[0];
        control_pts_buffer[vehicle_idx] = traj_result[1];

        /* 更新轨迹点 */  
        trajectory_output[vehicle_idx].clear();     
        for (int i = 0; i < update_times; i++)
        {
            trajectory_output[vehicle_idx].push_back(traj_buffer[i]);
        }
            
        /* 计算下一轮的一段轨迹 */
        control_pts_now[vehicle_idx][0] = control_pts_buffer[vehicle_idx][update_times/10];
        control_pts_now[vehicle_idx][1] = control_pts_buffer[vehicle_idx][update_times/10+1];
        control_pts_now[vehicle_idx][2] = control_pts_buffer[vehicle_idx][update_times/10+2];
    }
    /* 速度指向 */
    vehicle_heading[vehicle_idx].first = trajectory_output[vehicle_idx][trajectory_output[vehicle_idx].size()-1].x - trajectory_output[vehicle_idx][trajectory_output[vehicle_idx].size()-2].x;
    vehicle_heading[vehicle_idx].second = trajectory_output[vehicle_idx][trajectory_output[vehicle_idx].size()-1].y - trajectory_output[vehicle_idx][trajectory_output[vehicle_idx].size()-2].y;
    // last_vels_x[vehicle_idx] = last_vel_x;
    // last_vels_y[vehicle_idx] = last_vel_y;
}

/***********************************************************
 * 单艇覆盖局部规划
 *      输入：
 *          路径点序列；
 *          车辆索引号；
 *          局部水雷位置及禁区范围；
 ***********************************************************/
void Module::planloop_single_cover(std::vector<cv::Point2f> cover_path, int vehicle_idx, std::vector<cv::Point2f> local_mines_pos, std::vector<int> local_mines_radius, float sim_vel)
{
    // last_vel_x = last_vels_x[vehicle_idx];
    // last_vel_y = last_vels_y[vehicle_idx];
    for (int i = 0; i < gloable_mines_real.size(); i++)
    {
        if ( pow(gloable_mines_real[i].x - real_poses[vehicle_idx].x, 2)+pow(gloable_mines_real[i].y - real_poses[vehicle_idx].y, 2)<300*300 )
        {
            local_mines_pos.push_back(gloable_mines_real[i]);
            local_mines_radius.push_back(70);
        }
        
    }
    
    int update_times = 10;
    std::vector<cv::Point2f> traj_buffer;

    /* 判断当前是否还在覆盖进程中 */
    if (!vehicle_cover_finished[vehicle_idx])
    {
        /* 当前路径点在雷区中时则换为下一个路径点 */
        while ( inside_mines(cover_path[target_pt_idx[vehicle_idx]], local_mines_pos) )
        {
            target_pt_idx[vehicle_idx] = target_pt_idx[vehicle_idx]+1;
        }

        if (pow(poses_now[vehicle_idx].x-real_poses[vehicle_idx].x, 2)+pow(poses_now[vehicle_idx].y-real_poses[vehicle_idx].y, 2)<1)
        {
            /* 计算一段轨迹 */
            std::vector<std::vector<cv::Point2f>> traj_result = construct_trajectory(control_pts_now[vehicle_idx][0], control_pts_now[vehicle_idx][1], control_pts_now[vehicle_idx][2], cover_path[target_pt_idx[vehicle_idx]], local_mines_pos, local_mines_radius, sim_vel);
            traj_buffer = traj_result[0];
            control_pts_buffer[vehicle_idx] = traj_result[1];

            /* 更新轨迹 */
            trajectory_output[vehicle_idx].clear();
            for (int i = 0; i < update_times; i++)
            {
                trajectory_output[vehicle_idx].push_back(traj_buffer[i]);
            }
        

            /* 下一轮轨迹的控制点 */
            control_pts_now[vehicle_idx][0] = control_pts_buffer[vehicle_idx][update_times/10];
            control_pts_now[vehicle_idx][1] = control_pts_buffer[vehicle_idx][update_times/10+1];
            control_pts_now[vehicle_idx][2] = control_pts_buffer[vehicle_idx][update_times/10+2];
        }
        else
        {
            /* 生成一段轨迹 */
            std::vector<std::vector<cv::Point2f>> traj_result = construct_trajectory(real_poses[vehicle_idx], real_poses[vehicle_idx], real_poses[vehicle_idx], cover_path[target_pt_idx[vehicle_idx]], local_mines_pos, local_mines_radius, sim_vel);
             traj_buffer = traj_result[0];
            control_pts_buffer[vehicle_idx] = traj_result[1];
          
            /* 计算下一轮的一段轨迹 */
            control_pts_now[vehicle_idx][0] = control_pts_buffer[vehicle_idx][update_times/10];
            control_pts_now[vehicle_idx][1] = control_pts_buffer[vehicle_idx][update_times/10+1];
            control_pts_now[vehicle_idx][2] = control_pts_buffer[vehicle_idx][update_times/10+2];

            /* 生成一段轨迹 */
            traj_result = construct_trajectory(control_pts_now[vehicle_idx][0], control_pts_now[vehicle_idx][1], control_pts_now[vehicle_idx][2], cover_path[target_pt_idx[vehicle_idx]], local_mines_pos, local_mines_radius, sim_vel);
            traj_buffer = traj_result[0];
            control_pts_buffer[vehicle_idx] = traj_result[1];

            /* 更新轨迹点 */  
            trajectory_output[vehicle_idx].clear();     
            for (int i = 0; i < update_times; i++)
            {
                trajectory_output[vehicle_idx].push_back(traj_buffer[i]);
            }
            
            /* 计算下一轮的一段轨迹 */
            control_pts_now[vehicle_idx][0] = control_pts_buffer[vehicle_idx][update_times/10];
            control_pts_now[vehicle_idx][1] = control_pts_buffer[vehicle_idx][update_times/10+1];
            control_pts_now[vehicle_idx][2] = control_pts_buffer[vehicle_idx][update_times/10+2];
        }
        
        /* 如果本段接近终点则改换下一个目标点 */
        if ( pow(control_pts_now[vehicle_idx][2].x-cover_path[target_pt_idx[vehicle_idx]].x, 2)+pow(control_pts_now[vehicle_idx][2].y-cover_path[target_pt_idx[vehicle_idx]].y, 2) < sim_vel*sim_vel )
        {
            target_pt_idx[vehicle_idx] = target_pt_idx[vehicle_idx]+1;
        }

        /* 判断是否完成本次覆盖 */
        if (target_pt_idx[vehicle_idx] >= cover_path.size() )
        {
            vehicle_cover_finished[vehicle_idx] = true;
            vehicle_revisiting[vehicle_idx] = true; 
        }

    /* 速度指向 */
    vehicle_heading[vehicle_idx].first = trajectory_output[vehicle_idx][trajectory_output[vehicle_idx].size()-1].x - trajectory_output[vehicle_idx][trajectory_output[vehicle_idx].size()-2].x;
    vehicle_heading[vehicle_idx].second = trajectory_output[vehicle_idx][trajectory_output[vehicle_idx].size()-1].y - trajectory_output[vehicle_idx][trajectory_output[vehicle_idx].size()-2].y;

    }
    // last_vels_x[vehicle_idx] = last_vel_x;
    // last_vels_y[vehicle_idx] = last_vel_y;
} 


void Module::visualize()
{
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cv::Point2f pos_sim;
            pos_sim.x = trajectory_output[j][i].x/10.0;
            pos_sim.y = trajectory_output[j][i].y/10.0;
            cv::circle(visual_graph, pos_sim, 1, cv::Scalar(0, 50, 125), cv::FILLED);
            poses_now[j] = pos_sim ;
            real_poses[j] = trajectory_output[j][i];
        }
        cv::imshow("visualizaiton", visual_graph);
        cv::waitKey(1);
    }
    
}

/* 人工选择方案 */
void Module::choose_plan()
{
    /* 可视化路径选项 */
    cv::Mat3b choice_1_visual;
    choice_1_visual = cv::Mat3b(800, 800);
    choice_1_visual.setTo(cv::Scalar(255, 255, 255));
    
    cv::Mat3b choice_2_visual;
    choice_2_visual = cv::Mat3b(800, 800);
    choice_2_visual.setTo(cv::Scalar(255, 255, 255));

    for (int i = 0; i < cover_path_choice_first.size(); i++)
    {
        for (int j = 0; j < cover_path_choice_first[i].size(); j++)
        {
            cv::Point2f path_pt;
            path_pt.x =  cover_path_choice_first[i][j].x;
            path_pt.y =  cover_path_choice_first[i][j].y;
            cv::circle(choice_1_visual, path_pt, 2, cv::Scalar(100, 0, 0), cv::FILLED);
        }
    }

    for (int i = 0; i < cover_path_choice_second.size(); i++)
    {
        for (int j = 0; j < cover_path_choice_second[i].size(); j++)
        {
            cv::Point2f path_pt;
            path_pt.x =  cover_path_choice_second[i][j].x/10;
            path_pt.y =  cover_path_choice_second[i][j].y/10;
            cv::circle(choice_2_visual, path_pt, 2, cv::Scalar(100, 0, 0), cv::FILLED);
        }
    }
    
    cv::namedWindow("choice01", cv::WINDOW_NORMAL);
    cv::imshow("choice01", choice_1_visual);
    
    cv::namedWindow("choice02", cv::WINDOW_NORMAL);
    cv::imshow("choice02", choice_2_visual);

    cv::waitKey(0);

    int choice=2;
    // std::cin << choice;
    if (choice == 1)
    {
        for (int i = 0; i < cover_path_choice_first.size(); i++)
        {
            for (int j = 0; j < cover_path_choice_first[i].size(); j++)
            {
                cover_path_choice_first[i][j].x = cover_path_choice_first[i][j].x*10;
                cover_path_choice_first[i][j].y = cover_path_choice_first[i][j].y*10;
            }
            all_cover_paths = cover_path_choice_first;
        }
        
    }
    if (choice == 2)
    {
        all_cover_paths = cover_path_choice_second;
    }
}

void Module::Get_Safe_Place(std::vector<int> unassigned, LocationTrans _location_trans, std::vector<double> &safe_place_temp, double max_lat, double min_lat)
{
    double real_distance = 0;
    std::vector<double> lat_vec;
    LatLngPoint min_lat_point, max_lat_point;
    for (int i = 0; i < unassigned.size(); i++)
    {
        lat_vec.push_back(_location_trans.JobIndex2Location[unassigned[i]].lat);
        // if (_location_trans.JobIndex2Location[unassigned[i]].lat < min_lat) 
        // {
        //     min_lat = _location_trans.JobIndex2Location[unassigned[i]].lat;
        //     min_lat_point = _location_trans.JobIndex2Location[unassigned[i]];
        
        // }
        // if (_location_trans.JobIndex2Location[unassigned[i]].lat > max_lat) 
        // {
        //     max_lat = _location_trans.JobIndex2Location[unassigned[i]].lat;
        //     max_lat_point = _location_trans.JobIndex2Location[unassigned[i]];
        // }
    }
    lat_vec.push_back(max_lat);
    lat_vec.push_back(min_lat);
    std::sort(lat_vec.begin(),lat_vec.end());
    if(lat_vec.size() > 1)
    {
        for (int i = 0; i < lat_vec.size() - 1; i++)
        {
            double real_dis_temp = (lat_vec[i+1] - lat_vec[i]) * 111000;
            if (real_dis_temp > real_distance)
            {
                real_distance = real_dis_temp;
                min_lat = lat_vec[i];
                max_lat = lat_vec[i+1];
            }
        }
    
	    std::cout << "g_BlackBoardAgent->SafeLength: "  << std::endl;

        std::vector<double> safe_place;
        safe_place.push_back(min_lat);
        safe_place.push_back(max_lat);
        safe_place.push_back(real_distance);
        safe_place_temp.assign(safe_place.begin(), safe_place.end());
    }
}

void Module::Output_Safe_Place(std::vector<double> &safe_place)
{
    safe_place.assign(safe_place_temp.begin(), safe_place_temp.end());
}

/* 计算剩余时间 */
void Module::report_time_left()
{
    /* 计算覆盖的时间 */
    rest_time_in_cover.clear();
    int rest_points_num = 0;
    for (int i = 0; i < vehicle_cover_finished.size(); i++)
    {
        if (!vehicle_cover_finished[i])
        {
            int single_rest_pts_num = all_cover_paths[i].size() - target_pt_idx[i] - 1;
            rest_time_in_cover.push_back((int)(single_rest_pts_num * 300/2.5));
            if (single_rest_pts_num > rest_points_num)
            {
                rest_points_num = single_rest_pts_num;
            }
        }
        else
        {
            rest_time_in_cover.push_back(0);
        }
        
    }
    float cover_rest_time = rest_points_num * 300.0 / 2.5;    
    total_time_need = revisit_rest_time * 10 + cover_rest_time;
    // std::cout << "total_time_need: " << total_time_need << std::endl;
}

bool Module::MinesEliminateDecision(cv::Point2f mines_pos, vector<cv::Point2f> other_mines_pos, cv::Point2f desternation, vector<cv::Point2f> area, double w_cost_contain, double w_cost_distance2desternation, \
								double w_cost_distance2centerline, double thresthold = 1)
{
    // std::cout << "In eliminate decision! " << std::endl;
	//参数一 检查此雷是否禁区包含
	bool RegionsContainEachOther=false;
    // other_mines_pos.pop_back();
	for (auto& mines_pt : other_mines_pos)
	{
		if (pow(mines_pt.x - mines_pos.x, 2) + pow(mines_pt.y - mines_pos.y, 2) < pow(jinqujuli, 2))
			RegionsContainEachOther = true;
	}
	//参数二 距离终点距离
	double Dis2Desternation = sqrt(pow(desternation.x - mines_pos.x, 2) + pow(desternation.y - mines_pos.y, 2));
	//参数四 距离中轴线距离
	vector<double> area_y;
    // std::cout << "area size: " << area.size() << std::endl;
	for (auto& area_pt : area)
		area_y.push_back(area_pt.y);
	double max_y = *max_element(area_y.begin(), area_y.end());
	double min_y=  *min_element(area_y.begin(), area_y.end());
	double Dis2Centerline = mines_pos.y - (max_y + min_y) / 2;
	//总的代价值
	double cost = w_cost_contain * RegionsContainEachOther + w_cost_distance2desternation * Dis2Desternation+ w_cost_distance2centerline* Dis2Centerline;

    // std::cout << "Eliminate decision finished! " << std::endl;

	return cost > thresthold ? true : false;

}
//用来确定哪个船处理
//输入：雷的位置
//		所有智能体位置
//		所有智能体航向
//		所有智能体容量
//		容量代价
//		距离代价
//		航向代价
// 
//输出：处理雷的艇的编号
int Module::VehicleEliminateDecision(cv::Point2f mines_pos ,vector<cv::Point2f> vehicle_pos, vector<pair<double,double>> vehicle_heading, vector<int> capacity,double w_cost_capacity, double w_cost_distance2vehicle,double w_cost_heading) {
    
    //参数一 capaticy_cost
	vector<double> capaticy_cost;
	for (int i = 0; i < capacity.size(); i++)
		if (!capacity[i])
			capaticy_cost.push_back(w_cost_capacity);
		else
			capaticy_cost.push_back(0);
	//参数二 distance_cost
	vector<double> distance_cost;
	for (int i = 0; i < vehicle_pos.size(); i++)
		distance_cost.push_back(w_cost_distance2vehicle*sqrt(pow(vehicle_pos[i].x- mines_pos.x,2)+pow(vehicle_pos[i].y - mines_pos.y, 2)));

	//参数三 考虑w_cost_heading
	vector<double> heading_cost;
    heading_cost.resize(vehicle_pos.size());
	for (int i = 0; i < vehicle_pos.size(); i++)
	{
		double dx = vehicle_pos[i].x - mines_pos.x;
		double dy = vehicle_pos[i].y - mines_pos.y;
		double ds = sqrt(pow(dx, 2) + pow(dy, 2));


		pair<double, double> heading = { dx / ds,dy / ds };
		//cos越大cost越小所以取一个负号
		    
        // std::cout << vehicle_heading[i].first << ", " << vehicle_heading[i].second << std::endl;
        // std::cout << heading.first << ", " << heading.second << std::endl;
        
        heading_cost[i] = - w_cost_heading*(vehicle_heading[i].first * heading.first + vehicle_heading[i].second * heading.second) ;
	}

	vector<double> cost(vehicle_pos.size());

	for (int i = 0; i < cost.size(); i++)
		cost[i] = capaticy_cost[i] + distance_cost[i] + heading_cost[i];

    // std::cout << "Vehicle decision finished! " << std::endl;

	//让cost最小的去做
	return min_element(cost.begin(), cost.end())- cost.begin();

}

void Module::Reinit(int vehicle_num)
{
    /* 局规相关变量的重新初始化 */
    target_pt_idx.clear();
    vehicle_cover_finished.clear();
    vehicle_cover_finished_for_revisit.clear();
    vehicle_revisit_finished_for_end.clear();
    // mines_check.clear();

    target_pt_idx.resize(vehicle_num,0);
    vehicle_cover_finished.resize(vehicle_num,false);
    going_to_start = true;

    vehicle_cover_finished_for_revisit.resize(vehicle_num, false);;   // 完成覆盖进入重访的标志
    vehicle_revisit_finished_for_end.resize(vehicle_num, false);; // 完成重访结束任务的标志
    // mines_check.resize(gloable_mines.size(), false);; // 测试：重复障碍标志位

}