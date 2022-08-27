#include "Cover.hpp"


Cover::Cover(std::vector<cv::Point> input_area){
    target_area=input_area;
}
Cover::~Cover()
{
}

/* 轨迹规划升级版 */
cv::Point Cover::construct_Bspline(cv::Point ctrl0, cv::Point ctrl1, cv::Point ctrl2, cv::Point target, std::vector<cv::Point> local_mines_pos, std::vector<cv::Point> forbidden_area, int idx)
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
            // std::cout << "inside1" << std::endl;
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
     cv::circle(vis_map, ctrl3, 1, cv::Scalar(255, 0, 0), cv::FILLED);
    for (float t = 0.0; t < 1.0; t=t+0.2)
    {
        float traj_pt_x = ((float)ctrl0.x*(1-t)*(1-t)*(1-t) + (float)ctrl1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl3.x*t*t*t)/6.0;
        float traj_pt_y = ((float)ctrl0.y*(1-t)*(1-t)*(1-t) + (float)ctrl1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl3.y*t*t*t)/6.0;
        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
        // std::cout << "traj_pt:" << traj_pt_x << "," << traj_pt_y << std::endl;
        // cv::Point follower_target;
        // follower_target.x = traj_pt_x + 10.0;
        // follower_target.y = traj_pt_y + 10.0;
        // follower_now = follow_leader(follower_now, follower_target);
        // std::cout << "follower_now" << follower_now.x << ", " << follower_now.y << std::endl;
        trajs[idx].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
    }
    
    return ctrl3;
}

/* 去向目标点 */
void Cover::go_to_target(cv::Point start_pt, cv::Point target_pt, std::vector<cv::Point> local_mine_pos, std::vector<cv::Point> forbidden_area_pos, int idx)
{
    // cv::circle(vis_map, start_pt, 3, cv::Scalar(255, 255, 0), cv::FILLED);
    // cv::circle(vis_map, target_pt, 3, cv::Scalar(255, 0, 255), cv::FILLED);
    // std::vector<cv::Point> local_mine_pos = {cv::Point(250, 250), cv::Point(250, 270), cv::Point(230, 230)};
    // cv::circle(vis_map, local_mine_pos[0], 16, cv::Scalar(0, 255, 255), cv::FILLED);
    // cv::circle(vis_map, local_mine_pos[1], 16, cv::Scalar(0, 255, 255), cv::FILLED);
    // cv::circle(vis_map, local_mine_pos[2], 16, cv::Scalar(0, 255, 255), cv::FILLED);
    // std::vector<cv::Point> local_mine_pos = {};
    // std::cout << "00001" << std::endl;
    cv::Point ctrl_pt0, ctrl_pt1, ctrl_pt2,ctrl_pt3;
    ctrl_pt0 = start_pt;
    ctrl_pt1 = start_pt;
    ctrl_pt2 = start_pt;
    // follower_now = start_pt;
    // std::cout << "00002" << std::endl;
    while ( pow(target_pt.x - ctrl_pt2.x, 2) + pow(target_pt.y - ctrl_pt2.y, 2) > 3*3)
    {
        ctrl_pt3 = construct_Bspline(ctrl_pt0, ctrl_pt1, ctrl_pt2, target_pt, local_mine_pos, forbidden_area_pos, idx);
        ctrl_pt0 = ctrl_pt1;
        ctrl_pt1 = ctrl_pt2;
        ctrl_pt2 = ctrl_pt3;
    }
    // std::cout << "00003" << std::endl;
    ctrl_pt3 = target_pt;
    for (float t = 0; t < 1.0; t = t+0.2)
    {
        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
        // cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
        trajs[idx].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
    }
    ctrl_pt0 = ctrl_pt1;
    ctrl_pt1 = ctrl_pt2;
    ctrl_pt2 = ctrl_pt3;
    for (float t = 0; t < 1.0; t = t+0.2)
    {
        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
        // cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
        trajs[idx].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
    }
    ctrl_pt0 = ctrl_pt1;
    ctrl_pt1 = ctrl_pt2;
    ctrl_pt2 = ctrl_pt3;
    for (float t = 0; t < 1.0; t = t+0.2)
    {
        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
        // cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
        trajs[idx].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
    }
    pos_now = ctrl_pt3;
    // std::cout << "00004" << std::endl;
}

// /* 地图障碍多边形区域构建 */
// void Cover::construct_contours()
// {
//     // std::vector<cv::Point> target_area = {cv::Point(100,100), cv::Point(400, 100),  cv::Point(300, 300),  cv::Point(100,350)};  // 目标区域顶点
//     target_contours.push_back(target_area);

//     /* 计算OBB包围盒 */
//     OBB_box = cv::minAreaRect(cv::Mat(target_area));
//     OBB_angle = OBB_box.angle;
//     // std::cout << OBB_box.angle << std::endl;
//     // std::cout << PI << std::endl;

//     /* 将OBB包围盒旋转 */
//     for (int i = 0; i < target_area.size(); i++)
//     {
//         float target_area_x_f = (float)target_area[i].x;
//         float target_area_y_f = (float)target_area[i].y;
//         target_area[i].x = cos(OBB_angle*PI/180.0)*(target_area_x_f - OBB_box.center.x) - sin(OBB_angle*PI/180.0)*(target_area_y_f - OBB_box.center.y) + OBB_box.center.x;
//         target_area[i].y = sin(OBB_angle*PI/180.0)*(target_area_x_f - OBB_box.center.x) +cos(OBB_angle*PI/180.0)*(target_area_y_f - OBB_box.center.y) + OBB_box.center.y;
//     }

//     /* 找到边界点 */
//     /* --- 最左 */
//     left_point = target_area[0];
//     for (int i = 1; i < target_area.size(); i++)
//     {
//         if (target_area[i].x < left_point.x)
//         {
//             left_point = target_area[i];
//         }         
//     }
//     /* --- 最右 */
//     right_point = target_area[0];
//     for (int i = 1; i < target_area.size(); i++)
//     {
//         if (target_area[i].x > right_point.x)
//         {
//             right_point = target_area[i];
//         }              
//     }
//     /* --- 最上 */
//     top_point = target_area[0];
//     for (int i = 1; i < target_area.size(); i++)
//     {
//         if (target_area[i].y < top_point.y)
//         {
//             top_point = target_area[i];
//         }         
//     }
//     /* --- 最下 */
//     bottom_point = target_area[0];
//     for (int i = 1; i < target_area.size(); i++)
//     {
//         if (target_area[i].y > bottom_point.y)
//         {
//             bottom_point = target_area[i];
//         }              
//     }

//     /* 是否需要再转90度 */
//     if (abs(top_point.y-bottom_point.y)<abs(left_point.x-right_point.x))
//     {
//         // std::cout << target_area.size() <<std::endl;
//         for (int i = 0; i < target_area.size(); i++)
//         {
//             float target_area_x_f_2 = (float)target_area[i].x;
//             float target_area_y_f_2 = (float)target_area[i].y;
//             // std::cout<< target_area_y_f_2 << std::endl;
//             target_area[i].x = cos(0.5*PI)*(target_area_x_f_2 - OBB_box.center.x) - sin(0.5*PI)*(target_area_y_f_2 - OBB_box.center.y) + OBB_box.center.x;
//             target_area[i].y = sin(0.5*PI)*(target_area_x_f_2 - OBB_box.center.x) +cos(0.5*PI)*(target_area_y_f_2 - OBB_box.center.y) + OBB_box.center.y;
//         }

//         /* 找到边界点 */
//         /* --- 最左 */
//         left_point = target_area[0];
//         for (int i = 1; i < target_area.size(); i++)
//         {
//             if (target_area[i].x < left_point.x)
//             {
//                 left_point = target_area[i];
//             }         
//         }
//         /* --- 最右 */
//         right_point = target_area[0];
//         for (int i = 1; i < target_area.size(); i++)
//         {
//             if (target_area[i].x > right_point.x)
//             {
//                 right_point = target_area[i];
//             }              
//         }
//         /* --- 最上 */
//         top_point = target_area[0];
//         for (int i = 1; i < target_area.size(); i++)
//         {
//             if (target_area[i].y < top_point.y)
//             {
//                 top_point = target_area[i];
//             }         
//         }
//         /* --- 最下 */
//         bottom_point = target_area[0];
//         for (int i = 1; i < target_area.size(); i++)
//         {
//             if (target_area[i].y > bottom_point.y)
//             {
//                 bottom_point = target_area[i];
//             }              
//         }
//         OBB_angle = OBB_angle+90;
//     }
    
//     /* 构建旋转后的任务区轮廓 */
//     target_contours_rotated = {target_area};
// }

/* 编队控制 */
cv::Point Cover::follow_leader(cv::Point follower_pos, cv::Point target_pos)
{
    float vel_x, vel_y;
    cv::Point go_pos;
    vel_x = (float)target_pos.x - (float)follower_pos.x;
    vel_y = (float)target_pos.y - (float)follower_pos.y;
    if (vel_x==0.0 && vel_y==0.0)
    {
        go_pos = follower_pos;
        cv::circle(vis_map, go_pos, 1, cv::Scalar(255, 125, 0), cv::FILLED);
        trajs[1].push_back(go_pos);
    }
    else
    {
        float follower_vel_x = vel_x / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
        float follower_vel_y = vel_y / sqrt( pow(vel_x, 2) + pow(vel_y, 2) );
        // std::cout << "follower_vel: " << follower_vel_x << "," << follower_vel_y << std::endl;
    
        float update_time = 1.0;
        // go_pos.x = (float)follower_pos.x + follower_vel_x * update_time;
        // go_pos.y = (float)follower_pos.y + follower_vel_y * update_time;
        go_pos.x = (float)follower_pos.x + vel_x * update_time;
        go_pos.y = (float)follower_pos.y + vel_y * update_time;
        cv::circle(vis_map, go_pos, 1, cv::Scalar(255, 125, 0), cv::FILLED);
        trajs[1].push_back(go_pos);
    }

    return go_pos;  // 下一时刻follower的位置
}

//更新了find_searching_path
/* 遍历路径 */
void Cover::find_searching_path(cv::Point leader_pt, bool NS_or_WE, float perception_dist)
{
    //考虑当前位置
    cv::Point current_pt = leader_pt;
    int up_down_direction; // 上下扫描方向
    int left_right_direction;   // 左右扫描方向

    /* 找到四角中距离最近的点 */
    cv::Point nearst_pt = target_area[0];
    for (int i = 1; i < target_area.size(); i++)
    {
        if ( (pow(target_area[i].x-current_pt.x, 2)+pow(target_area[i].y-current_pt.y, 2)) < (pow(nearst_pt.x-current_pt.x, 2)+pow(nearst_pt.y-current_pt.y, 2)) )
        {
            nearst_pt = target_area[i];
        }
    }

    /* 找到最大最小值 */
    int left_min = target_area[0].x;
    for (int i = 1; i < target_area.size(); i++)
    {
        if ( target_area[i].x < left_min )
        {
            left_min = target_area[i].x;
        }
    }
    int right_max = target_area[0].x;
    for (int i = 1; i < target_area.size(); i++)
    {
        if ( target_area[i].x > right_max )
        {
            right_max = target_area[i].x;
        }
    }
    int top_min = target_area[0].y;
    for (int i = 1; i < target_area.size(); i++)
    {
        if ( target_area[i].y < top_min )
        {
            top_min = target_area[i].y;
        }
    }
    int bottom_max = target_area[0].y;
    for (int i = 1; i < target_area.size(); i++)
    {
        if ( target_area[i].y > bottom_max )
        {
            bottom_max = target_area[i].y;
        }
    }

    cv::Point last_line_back;  // 上一次上下中最后一个点
    //从最近点出发，方向固定
    bool first_round = true;   // 是否第一次
    cv::Vec3b obs_point_scalar = { 0, 0, 0 };  // 任务区外障碍标记

    if (NS_or_WE)   // 南北走
    {
        if (nearst_pt.x == left_min)    // 最近点在左侧
        {
            if (nearst_pt.y == top_min)
            {
                up_down_direction = 1;
            }
            if (nearst_pt.y == bottom_max)
            {
                up_down_direction = -1;
            }

            /* 从左到右 */
            for (int i = left_min; i < right_max; i = i + perception_dist*2)
            {
                std::deque<cv::Point> line_path;
                std::deque<cv::Point> row_path;

                /* 上下扫描 */
                for (int j = 0; j < map_size_y; j = j + 20)
                {
                    if (vis_map.at<cv::Vec3b>(j, i) != obs_point_scalar)
                    {
                        if (up_down_direction == 1) // 如果是向下则将下方的点放在队尾
                         {
                            line_path.push_back(cv::Point(i, j));
                        }
                        else    // 如果是向上则将下方的点放在队首
                        {
                            line_path.push_front(cv::Point(i, j));
                        }
                    }
                }
                up_down_direction = (-1) * up_down_direction; // 下次更改上下的方向
                /* 向右运动 */  
                if (first_round == false)   
                {
                    cv::Point now_front = line_path[0];
                    for (int x_row = last_line_back.x + perception_dist; x_row < last_line_back.x ; x_row = x_row + perception_dist)
                    {
                        int y_row = ((float)last_line_back.y - (float)now_front.y) / ((float)last_line_back.x - (float)now_front.x) * ((float)x_row - (float)now_front.x) + (float)now_front.y;
                        row_path.push_back(cv::Point(x_row, y_row));
                    }
                    searching_path.push_back(row_path);
                }
                else    // 如果是第一个纵向路径
                {
                    first_round = false;
                }
                last_line_back = line_path[line_path.size() - 1];
                searching_path.push_back(line_path);
            }
        }
        
        if (nearst_pt.x == right_max)    // 最近点在左侧
        {
            if (nearst_pt.y == top_min)
            {
                up_down_direction = 1;
            }
            if (nearst_pt.y == bottom_max)
            {
                up_down_direction = -1;
            }

            /* 从右到左 */
            for (int i = right_max; i > left_min - perception_dist; i = i - perception_dist*2)
            {
                std::deque<cv::Point> line_path;
                std::deque<cv::Point> row_path;

                /* 上下扫描 */
                for (int j = 0; j < map_size_y; j = j + 20)
                {
                    if (vis_map.at<cv::Vec3b>(j, i) != obs_point_scalar)
                    {
                        if (up_down_direction == 1) // 如果是向下则将下方的点放在队尾
                         {
                            line_path.push_back(cv::Point(i, j));
                        }
                        else    // 如果是向上则将下方的点放在队首
                        {
                            line_path.push_front(cv::Point(i, j));
                        }
                    }
                }
                up_down_direction = (-1) * up_down_direction; // 下次更改上下的方向
                /* 向左运动 */  
                if (first_round == false)   
                {
                    cv::Point now_front = line_path[0];
                    for (int x_row = last_line_back.x - perception_dist; x_row > last_line_back.x ; x_row = x_row - perception_dist)
                    {
                        int y_row = ((float)last_line_back.y - (float)now_front.y) / ((float)last_line_back.x - (float)now_front.x) * ((float)x_row - (float)now_front.x) + (float)now_front.y;
                        row_path.push_back(cv::Point(x_row, y_row));
                    }
                    searching_path.push_back(row_path);
                }
                else    // 如果是第一个纵向路径
                {
                    first_round = false;
                }
                last_line_back = line_path[line_path.size() - 1];
                searching_path.push_back(line_path);
            }

        }

    }
    else
    {
        if (nearst_pt.y == top_min)    // 最近点在上侧
        {
            if (nearst_pt.x == left_min)
            {
                left_right_direction = 1;
            }
            if (nearst_pt.x == right_max)
            {
                left_right_direction = -1;
            }

            /* 从上到下 */
            for (int i = top_min; i < bottom_max; i = i + perception_dist*2)
            {
                std::deque<cv::Point> line_path;
                std::deque<cv::Point> row_path;

                /* 上下扫描 */
                for (int j = 0; j < map_size_x; j = j + 20)
                {
                    if (vis_map.at<cv::Vec3b>(i, j) != obs_point_scalar)
                    {
                        if (left_right_direction == 1) // 如果是向右则将右方的点放在队尾
                         {
                            line_path.push_back(cv::Point(j, i));
                        }
                        else    // 如果是向左则将右方的点放在队首
                        {
                            line_path.push_front(cv::Point(j, i));
                        }
                    }
                }
               left_right_direction = (-1) * left_right_direction; // 下次更改上下的方向
                /* 向右运动 */  
                if (first_round == false)   
                {
                    cv::Point now_front = line_path[0];
                    for (int y_row = last_line_back.y + perception_dist; y_row < last_line_back.y ; y_row = y_row + perception_dist)
                    {
                        int x_row = ((float)last_line_back.x - (float)now_front.x) / ((float)last_line_back.y - (float)now_front.y) * ((float)y_row - (float)now_front.y) + (float)now_front.x;
                        row_path.push_back(cv::Point(x_row, y_row));
                    }
                    searching_path.push_back(row_path);
                }
                else    // 如果是第一个纵向路径
                {
                    first_round = false;
                }
                last_line_back = line_path[line_path.size() - 1];
                searching_path.push_back(line_path);
            }
        }
        
        if (nearst_pt.y == bottom_max)    // 最近点在下侧
        {
            if (nearst_pt.x == left_min)
            {
                left_right_direction = 1;
            }
            if (nearst_pt.x == right_max)
            {
                left_right_direction = -1;
            }

            /* 从下到上 */
            for (int i = bottom_max; i > top_min; i = i - perception_dist*2)
            {
                std::deque<cv::Point> line_path;
                std::deque<cv::Point> row_path;

                /* 左右扫描 */
                for (int j = 0; j < map_size_x; j = j + 20)
                {
                    if (vis_map.at<cv::Vec3b>(i, j) != obs_point_scalar)
                    {
                        if (left_right_direction == 1) // 如果是向右则将右方的点放在队尾
                         {
                            line_path.push_back(cv::Point(j, i));
                        }
                        else    // 如果是向左则将右方的点放在队首
                        {
                            line_path.push_front(cv::Point(j, i));
                        }
                    }
                }
               left_right_direction = (-1) * left_right_direction; // 下次更改上下的方向
                /* 上下运动 */  
                if (first_round == false)   
                {
                    cv::Point now_front = line_path[0];
                    for (int y_row = last_line_back.y - perception_dist; y_row > last_line_back.y ; y_row = y_row - perception_dist)
                    {
                        int x_row = ((float)last_line_back.x - (float)now_front.x) / ((float)last_line_back.y - (float)now_front.y) * ((float)y_row - (float)now_front.y) + (float)now_front.x;
                        row_path.push_back(cv::Point(x_row, y_row));
                    }
                    searching_path.push_back(row_path);
                }
                else    // 如果是第一个纵向路径
                {
                    first_round = false;
                }
                last_line_back = line_path[line_path.size() - 1];
                searching_path.push_back(line_path);
            }

        }
    }
    


    /* 生成出发路径 */
    std::deque<cv::Point> start_path;
    float dist_start_path = sqrt(pow(current_pt.x - searching_path[0][0].x, 2) + pow(current_pt.y - searching_path[0][0].y, 2));
    int seg_num = dist_start_path / 20;
    for (int i = 0; i < seg_num - 1; i++)
    {
        float pt_x = current_pt.x + (float)i * (searching_path[0][0].x - current_pt.x) / (float)seg_num;
        float pt_y = current_pt.y + (float)i * (searching_path[0][0].y - current_pt.y) / (float)seg_num;
        start_path.push_back(cv::Point(pt_x, pt_y));
    }
    searching_path.push_front(start_path);
}

/* 生成平滑轨迹 */
void Cover::plan_trajectory(std::vector<Point> &output_trajectory,bool formation_flag,std::vector<cv::Point> mines_local)
{
    // std::cout << "formulation_flag" << formation_flag << std::endl;
    if(!formation_flag)
    {
        /* 前往出发点 */
        

        /* 生成三次B样条曲线 */
        cv::Point ctrl_pt0, ctrl_pt1, ctrl_pt2, ctrl_pt3;   // 一段B样条曲线控制点
        ctrl_pt0 = searching_path[0][0];
        ctrl_pt1 = searching_path[0][0];
        ctrl_pt2 = searching_path[0][0];
        pos_now = searching_path[0][0];

        // std::vector<cv::Point> mines_local;  // 测试：障碍
        for (int i = 0; i < searching_path.size(); i++)
        {
            for (int j = 0; j < searching_path[i].size(); j++)
            {
                /* 测试: 障碍 */
                // if ( (i==2 && j == 5) || (i==4 && j == 6) || (i==6 && j == 4) || (i==8 && j == 4) || (i==10 && j == 6) || (i==3 && j == 5) || (i==5 && j == 6) || (i==7 && j == 4) || (i==9 && j == 4) || (i==11 && j == 6) )
                // {
                //     mines_near = true;
                //     mines_local.clear();
                //     mines_local = {searching_path[i][j]};
                //     mines.push_back(searching_path[i][j]);
                //     cv::circle(vis_map, searching_path[i][j], 5, cv::Scalar(120, 20, 0), cv::FILLED);
                // }
                // else
                // {
                    // mines_local.clear();
                    // mines_near = false;
                // }

                if (mines_local.empty())    // 周围无障碍
                {
                    /* B样条曲线 */
                    ctrl_pt3 = searching_path[i][j];
                    for (float t = 0.0; t < 1.0; t=t+0.05)
                    {
                        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
                        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
                        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
                        trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
                        output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
                    }      

                    /* 迭代控制点 */
                    ctrl_pt0 = ctrl_pt1;
                    ctrl_pt1 = ctrl_pt2;
                    ctrl_pt2 = ctrl_pt3;
                    pos_now = searching_path[i][j];
                }
                else    // 该段轨迹中有障碍
                {
                    /* 测试： */
                    /* 发现障碍后先到雷前面的控制点 */
                    for (float t = 0.0; t < 1.0; t=t+0.05)
                    {
                        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
                        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
                        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
                        trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
                        output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
                    }
                    ctrl_pt0 = ctrl_pt1;
                    ctrl_pt1 = ctrl_pt2;
                    ctrl_pt2 = ctrl_pt3;
                    ctrl_pt3 = searching_path[i][j-1];
                    for (float t = 0.0; t < 1.0; t=t+0.05)
                    {
                        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
                        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
                        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
                        trajs[0].push_back(cv::Point(traj_pt_x, traj_pt_y));
                        output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
                    }
                    ctrl_pt0 = ctrl_pt1;
                    ctrl_pt1 = ctrl_pt2;
                    ctrl_pt2 = ctrl_pt3;
                    ctrl_pt3 = searching_path[i][j-1];
                    for (float t = 0.0; t < 1.0; t=t+0.05)
                    {
                        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
                        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
                        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
                        trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
                        output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
                    }

                    /* 调用go to target */
                    go_to_target(pos_now, searching_path[i][j+1], mines_local, mines_local,0);

                    ctrl_pt0 = searching_path[i][j+1];
                    ctrl_pt1 = searching_path[i][j+1];
                    ctrl_pt2 = searching_path[i][j+1];
                    j = j+1;
                }
            }      
        } 
        for (float t = 0.0; t < 1.0; t=t+0.05)
        {
            float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
            float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
            cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
            trajs[0].push_back(cv::Point(traj_pt_x, traj_pt_y));
            output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
        } 
        ctrl_pt0 = ctrl_pt1;
        ctrl_pt1 = ctrl_pt2;
        ctrl_pt2 = ctrl_pt3;
        ctrl_pt3 = searching_path[searching_path.size()-1][searching_path[searching_path.size()-1].size()-1];
        for (float t = 0.0; t < 1.0; t=t+0.05)
        {
            float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
            float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
            cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
            output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
            trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
        }
        ctrl_pt0 = ctrl_pt1;
        ctrl_pt1 = ctrl_pt2;
        ctrl_pt2 = ctrl_pt3;
        ctrl_pt3 = searching_path[searching_path.size()-1][searching_path[searching_path.size()-1].size()-1];
        for (float t = 0.0; t < 1.0; t=t+0.05)
        {
            float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
            float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
            cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
            trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
            output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
        }
    }
    else
    {
        //TODO this formation code need to be done below:
         /* 生成三次B样条曲线 */
        cv::Point ctrl_pt0, ctrl_pt1, ctrl_pt2, ctrl_pt3;   // 一段B样条曲线控制点
        ctrl_pt0 = searching_path[0][0];
        ctrl_pt1 = searching_path[0][0];
        ctrl_pt2 = searching_path[0][0];
        pos_now = searching_path[0][0];
        follower_pos_now.x = pos_now.x + 10.0;
        follower_pos_now.y = pos_now.y + 10.0;
        // bool mines_near = false;    // 搜索范围内无障碍
        // std::vector<cv::Point> mines_local;  // 测试：障碍
        
        for (int i = 0; i < searching_path.size(); i++)
        {
            for (int j = 0; j < searching_path[i].size(); j++)
            {
                /* 测试: 障碍 */
                // if ( (i==2 && j == 5) || (i==4 && j == 6) || (i==6 && j == 4) || (i==8 && j == 4) || (i==10 && j == 6) || (i==3 && j == 5) || (i==5 && j == 6) || (i==7 && j == 4) || (i==9 && j == 4) || (i==11 && j == 6) )
                // {
                //      mines_near = true;
                //      mines_local.clear();
                //     mines_local = {searching_path[i][j]};
                //     cv::circle(vis_map, searching_path[i][j], 5, cv::Scalar(120, 20, 0), cv::FILLED);
                //     mines.push_back(searching_path[i][j]);
                // }
                // else
                // {
                    // mines_local.clear();
                    // mines_near = false;
                // }

                if (!mines_local.empty())    // 周围无障碍
                {
                    /* B样条曲线 */
                    ctrl_pt3 = searching_path[i][j];
                    for (float t = 0.0; t < 1.0; t=t+0.05)
                    {
                        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
                        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
                        cv::Point follower_target;
                        follower_target.x = traj_pt_x + 10.0;
                        follower_target.y = traj_pt_y + 10.0;
                        follower_pos_now = follow_leader(follower_pos_now, follower_target);
                        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
                        trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
                        output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
                    }      

                    /* 迭代控制点 */
                    ctrl_pt0 = ctrl_pt1;
                    ctrl_pt1 = ctrl_pt2;
                    ctrl_pt2 = ctrl_pt3;
                    pos_now = searching_path[i][j];
                }
                else    // 该段轨迹中有障碍
                {
                    /* ******************************************************* */
                    /* 这里需要根据决策指派不同的艇执行不同任务 */
                    for (float t = 0.0; t < 1.0; t=t+0.05)
                    {
                        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
                        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
                        cv::Point follower_target;
                        follower_target.x = traj_pt_x + 10.0;
                        follower_target.y = traj_pt_y + 10.0;
                        follower_pos_now = follow_leader(follower_pos_now, follower_target);
                        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
                        trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
                        output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
                    }
                    ctrl_pt0 = ctrl_pt1;
                    ctrl_pt1 = ctrl_pt2;
                    ctrl_pt2 = ctrl_pt3;
                    ctrl_pt3 = searching_path[i][j-1];
                    for (float t = 0.0; t < 1.0; t=t+0.05)
                    {
                        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
                        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
                        cv::Point follower_target;
                        follower_target.x = traj_pt_x + 10.0;
                        follower_target.y = traj_pt_y + 10.0;
                        follower_pos_now = follow_leader(follower_pos_now, follower_target);
                        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
                        trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
                        output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
                    }
                    ctrl_pt0 = ctrl_pt1;
                    ctrl_pt1 = ctrl_pt2;
                    ctrl_pt2 = ctrl_pt3;
                    ctrl_pt3 = searching_path[i][j-1];
                    for (float t = 0.0; t < 1.0; t=t+0.05)
                    {
                        float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
                        float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
                        cv::Point follower_target;
                        follower_target.x = traj_pt_x + 10.0;
                        follower_target.y = traj_pt_y + 10.0;
                        follower_pos_now = follow_leader(follower_pos_now, follower_target);
                        cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
                        trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
                        output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
                    }

                    /* 调用go to target */
                    go_to_target(pos_now, searching_path[i][j+1], mines_local, mines_local,0);

                    /* follower */
                    cv::Point follower_target;
                    follower_target.x = searching_path[i][j+1].x + 10.0;
                    follower_target.y = searching_path[i][j+1].y + 10.0;
                    go_to_target(follower_pos_now, follower_target, mines_local, mines_local,1);

                    ctrl_pt0 = searching_path[i][j+1];
                    ctrl_pt1 = searching_path[i][j+1];
                    ctrl_pt2 = searching_path[i][j+1];
                    j = j+1;   
                }
            }      
        }
        for (float t = 0.0; t < 1.0; t=t+0.05)
        {
            float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
            float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
            cv::Point follower_target;
            follower_target.x = traj_pt_x + 10.0;
            follower_target.y = traj_pt_y + 10.0;
            follower_pos_now = follow_leader(follower_pos_now, follower_target);
            cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
            trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
            output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
        }
        ctrl_pt0 = ctrl_pt1;
        ctrl_pt1 = ctrl_pt2;
        ctrl_pt2 = ctrl_pt3;
        ctrl_pt3 = searching_path[searching_path.size()-1][searching_path[searching_path.size()-1].size()-1];
        for (float t = 0.0; t < 1.0; t=t+0.05)
        {
            float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
            float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
            cv::Point follower_target;
            follower_target.x = traj_pt_x + 10.0;
            follower_target.y = traj_pt_y + 10.0;
            follower_pos_now = follow_leader(follower_pos_now, follower_target);
            cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
            trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
            output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
        }
        ctrl_pt0 = ctrl_pt1;
        ctrl_pt1 = ctrl_pt2;
        ctrl_pt2 = ctrl_pt3;
        ctrl_pt3 = searching_path[searching_path.size()-1][searching_path[searching_path.size()-1].size()-1];
        for (float t = 0.0; t < 1.0; t=t+0.05)
        {
            float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
            float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
            cv::Point follower_target;
            follower_target.x = traj_pt_x + 10.0;
            follower_target.y = traj_pt_y + 10.0;
            follower_pos_now = follow_leader(follower_pos_now, follower_target);
            cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
            trajs[0].push_back(cv::Point2f(traj_pt_x, traj_pt_y));
            output_trajectory.push_back(cv::Point(traj_pt_x, traj_pt_y));
        }
    }
}

// /* 流程函数 */
// std::vector<cv::Point2f> Cover::CoverLoop(bool formation_flag, cv::Point leader_pt, cv::Point follower_pt)
// {
//         //  std::cout<<"555555"<<std::endl;
//     /* 构造地图 */
//     map_size_x = 500;
//     map_size_y = 500;
//     map = cv::Mat3b(cv::Size(map_size_x, map_size_y));   // 构造500*500像素区域
    

//     /* 获取轮廓 */
//     construct_contours();
//     // std::cout<<"vis_map::"<<vis_map.cols<<std::endl;
//     /* 构建地图 */
//     map.setTo(cv::Scalar(0, 0, 0)); // 设置全图灰度值为全黑
//     cv::fillPoly(map, target_contours, cv::Scalar(255, 255, 255));    // 将目标区域设为全白
//     cv::cvtColor(map, vis_map, cv::COLOR_GRAY2BGR);
//     // std::cout<<"vis_map"<<vis_map.cols<<std::endl;
//     map.setTo(cv::Scalar(0, 0, 0)); // 设置全图灰度值为全黑
//     cv::fillPoly(map, target_contours_rotated, cv::Scalar(255, 255, 255));    // 将目标区域设为全白
//     cv::cvtColor(map, rotated_map, cv::COLOR_GRAY2BGR);

//     /* 障碍位置 */
//     // mines = {cv::Point(200, 260), cv::Point(180, 140), cv::Point(300, 160)};

//     // for (int i = 0; i < mines.size(); i++)
//     // {
//     //     /* 在原地图中添加杀伤范围 */
//     //     cv::circle(vis_map, mines[i], 16, cv::Scalar(255, 0, 255), cv::FILLED);
//     // }



//     trajs.clear();
//     trajs.resize(2);
//     mines.clear();
//         //  std::cout<<"888888"<<std::endl;
//     /* 遍历路径 */
//     find_searching_path(leader_pt); //have problem
//         //  std::cout<<"999999"<<std::endl;
//     std::deque<std::deque<cv::Point>> temp_path;    // 暂存
//     /* 补盲 */
//     std::deque<cv::Point> blind_area;



//      std::vector<cv::Point> output_trajectory;

//     cv::Mat3b v_map;
//      v_map = cv::Mat3b(500, 500);
//      v_map.setTo(cv::Scalar(255, 255, 255));
//      cv::namedWindow("v_map", cv::WINDOW_NORMAL);
//     for (int i = 0; i < target_contours.size(); i++)
//      {
//           for (int j = 0; j < target_contours[i].size(); j++)
//           {
//                cv::circle(v_map, target_contours[i][j], 5, cv::Scalar(120, 0, 129), cv::FILLED);
//           }
//      }
//      for (int i = 0; i < searching_path.size(); i++)
//      {
//           for (int j = 0; j < searching_path[i].size(); j++)
//           {
//                cv::circle(v_map, searching_path[i][j], 1, cv::Scalar(120, 118, 129), cv::FILLED);
//           }
//      }
//      cv::imshow("v_map", v_map);
//      cv::waitKey(0);   // 这里的数字应该就是更新时间
//     // std::cout << "traj_size:" << trajs[0].size() << std::endl;
    
//     std::vector<cv::Point2f> path_output;
//     for (int i = 0; i < searching_path.size(); i++)
//     {
//         for (int j = 0; j < searching_path[i].size(); j++)
//         {
//             path_output.push_back(cv::Point2f(searching_path[i][j].x, searching_path[i][j].y));
//         }
//     }
    

//     return path_output;
// }


/* 流程函数_更新 */
std::vector<cv::Point2f> Cover::CoverLoop(bool formation_flag, cv::Point leader_pt, cv::Point follower_pt, float percept_area)
{
        //  std::cout<<"555555"<<std::endl;
    /* 构造地图 */
    map_size_x = 800;
    map_size_y = 800;
    map = cv::Mat3b(cv::Size(map_size_x, map_size_y));   // 构造500*500像素区域
    

    /* 获取轮廓 */
    // construct_contours();
    // std::cout<<"vis_map::"<<vis_map.cols<<std::endl;
    /* 构建地图 */
    for (int i = 0; i < target_area.size(); i++)
    {
        target_area[i].x = target_area[i].x/10.0;
        target_area[i].y = target_area[i].y/10.0;
    }
    
    target_contours.push_back(target_area);
    map.setTo(cv::Scalar(0, 0, 0)); // 设置全图灰度值为全黑
    cv::fillPoly(map, target_contours, cv::Scalar(255, 255, 255));    // 将目标区域设为全白
    cv::cvtColor(map, vis_map, cv::COLOR_GRAY2BGR);

    trajs.clear();
    trajs.resize(2);
    mines.clear();

    /* 遍历路径 */
    find_searching_path(leader_pt, true, percept_area); //have problem
    
    std::deque<std::deque<cv::Point>> temp_path;    // 暂存
    /* 补盲 */
    std::deque<cv::Point> blind_area;

    std::vector<cv::Point> output_trajectory;

    std::vector<cv::Point2f> path_output;
    for (int i = 0; i < searching_path.size(); i++)
    {
        for (int j = 0; j < searching_path[i].size(); j++)
        {
            path_output.push_back(cv::Point2f(searching_path[i][j].x, searching_path[i][j].y));
        }
    }

    return path_output;
}

/***********************************************************
 * 构建菱形覆盖路径函数
 *      输入：
 *          中心起始点；
 *          前后左右间距；
 *          路径点间隔；
 *      输出：
 *          前后左右顺序的各艇路径;
  ***********************************************************/
std::vector<std::vector<cv::Point2f>> Cover::construct_lozenge_paths(cv::Point2f center_start_pt, float width_up, float width_down, float length_forward, float length_back, float path_dist)
{
    std::vector<std::vector<cv::Point2f>> paths_result;

    std::vector<cv::Point2f> path_pts;
    
    /* 0号元素：前船路径点 */
    for (float pt_x = center_start_pt.x; pt_x < center_start_pt.x+5400 ; pt_x = pt_x+path_dist)
    {
        path_pts.push_back(cv::Point2f(pt_x, center_start_pt.y));
    }
    paths_result.push_back(path_pts);

    /* 1号元素：后船路径点 */
    path_pts.clear();
    for (int i = 0; i < paths_result[0].size(); i++)
    {
        path_pts.push_back(cv::Point2f(paths_result[0][i].x - length_forward -  length_back, paths_result[0][i].y));
    }
    paths_result.push_back(path_pts);
    
    /* 2号元素：左船路径点 */
    path_pts.clear();
    for (int i = 0; i < paths_result[0].size(); i++)
    {
        path_pts.push_back(cv::Point2f(paths_result[0][i].x-length_forward, paths_result[0][i].y-width_up));
    }
    paths_result.push_back(path_pts);

    /* 3号元素：右船路径点 */
    path_pts.clear();
    for (int i = 0; i < paths_result[0].size(); i++)
    {
        path_pts.push_back(cv::Point2f(paths_result[0][i].x-length_forward, paths_result[0][i].y+width_down));
    }
    paths_result.push_back(path_pts);
    
    

    return paths_result;
}

// std::vector<std::vector<cv::Point2f>> Cover::construct_full_paths(cv::Point2f center_start_pt, float width_up, float width_down, float length_forward, float length_back, float middle_dist, float path_dist)
// {
//     std::vector<std::vector<cv::Point2f>> paths_result;

//     std::vector<cv::Point2f> path_pts;
    
//     /* 计算剩余区域 */
//     float rest_area = 1800 - width_up*2 - width_down*2 - middle_dist;
    
//     /* 0号元素：前船路径点 */
//     for (float pt_x = center_start_pt.x; pt_x < center_start_pt.x+5400 ; pt_x = pt_x+path_dist)
//     {
//         path_pts.push_back(cv::Point2f(pt_x, center_start_pt.y));
//     }
//     for (float pt_x = center_start_pt.x+5400-path_dist; pt_x > center_start_pt.x-path_dist; pt_x = pt_x-path_dist)
//     {
//         path_pts.push_back(cv::Point2f(pt_x, center_start_pt.y+width_up+width_down+middle_dist));
//     }
    
//     paths_result.push_back(path_pts);
    
//     /* 1号元素：左船路径点 */
//     path_pts.clear();
//     for (int i = 0; i < paths_result[0].size(); i++)
//     {
//         path_pts.push_back(cv::Point2f(paths_result[0][i].x-length_back, paths_result[0][i].y-width_up));
//     }
//     paths_result.push_back(path_pts);

//     /* 2号元素：右船路径点 */
//     path_pts.clear();
//     for (int i = 0; i < paths_result[0].size(); i++)
//     {
//         path_pts.push_back(cv::Point2f(paths_result[0][i].x-length_back, paths_result[0][i].y+width_down));
//     }
//     paths_result.push_back(path_pts);

//     /* 3号元素：后船路径点 */
//     path_pts.clear();
//     for (float pt_x = center_start_pt.x; pt_x < center_start_pt.x+5400 ; pt_x = pt_x+path_dist)
//     {
//         path_pts.push_back(cv::Point2f(pt_x,  center_start_pt.y+width_up+width_down*2+middle_dist+rest_area*0.5));
//     }
//     paths_result.push_back(path_pts);

//     return paths_result;
// }


/*************************************************************************
 *  更新编队覆盖路径
 *      输入：
 *          
 *      输出：
 *          各船路径点
 *************************************************************************/
std::vector<std::vector<cv::Point2f>>Cover::construct_full_path(float area_length, float area_width, cv::Point2f center_start_pt, float range_1, float range_2, float range_3, float range_4, float path_dist)
{
    std::vector<std::vector<cv::Point2f>> paths_result;
    std::vector<cv::Point2f> single_path_1, single_path_2, single_path_3, single_path_4;

    /* 计算来回次数 */
    int lines_num = floor(area_width/(range_1+range_2+range_3));

    /* 计算剩余范围 */
    float rest_area = area_width - lines_num*(range_1+range_2+range_3);

    /* 规划四条艇的路径 */
    float search_direction = 1;
    for (int i = 0; i < lines_num; i++)
    {
        /* --- 艇1中间艇, 船2左侧，船3右侧 --- */
        if (search_direction == 1)
        {
            for (float pt_x = center_start_pt.x; pt_x <= area_length+center_start_pt.x; pt_x = pt_x+path_dist)
            {
                cv::Point2f path_pt;
                path_pt.x = pt_x;
                path_pt.y = i*(range_1+range_2+range_3) + range_2 + 0.5*range_1;
                single_path_1.push_back(path_pt);
                path_pt.x = pt_x;
                path_pt.y = i*(range_1+range_2+range_3) + range_2*0.5;
                single_path_2.push_back(path_pt);
                path_pt.x = pt_x;
                path_pt.y = i*(range_1+range_2+range_3) + range_2 + range_1 + range_3*0.5;
                single_path_3.push_back(path_pt);

                /* --- 艇4跟随路径 --- */
                //if (i != lines_num-1)
                //{
                    path_pt.x = pt_x;
                    path_pt.y = i*(range_1+range_2+range_3) + range_2 + range_1 + range_3 + range_4*0.5;
                    single_path_4.push_back(path_pt);
                //}
            }
        }
        else
        {
            for (float pt_x = center_start_pt.x+area_length; pt_x >= center_start_pt.x; pt_x = pt_x-path_dist)
            {
                cv::Point2f path_pt;
                path_pt.x = pt_x;
                path_pt.y = i*(range_1+range_2+range_3) + range_2 + 0.5*range_1;
                single_path_1.push_back(path_pt);
                path_pt.x = pt_x;
                path_pt.y = i*(range_1+range_2+range_3) + range_2*0.5;
                single_path_2.push_back(path_pt);
                path_pt.x = pt_x;
                path_pt.y = i*(range_1+range_2+range_3) + range_2 + range_1 + range_3*0.5;
                single_path_3.push_back(path_pt);

                /* --- 艇4跟随路径 --- */
                // if (i != lines_num-1)
                // {
                    path_pt.x = pt_x;
                    path_pt.y = i*(range_1+range_2+range_3) + range_2 + range_1 + range_3 + range_4*0.5;
                    single_path_4.push_back(path_pt);
                // }
            }
        }

        search_direction = -search_direction;
    }

    /* 剩余区域处理 */
    if (rest_area<range_1)  // 如果只需要一艘艇去处理
    {
        if (search_direction == 1)
        {
            for (float pt_x = center_start_pt.x; pt_x <= area_length+center_start_pt.x; pt_x = pt_x+path_dist)
            {
                cv::Point2f path_pt;
                path_pt.x = pt_x;
                path_pt.y = lines_num*(range_1+range_2+range_3) + 0.5*range_1;
                single_path_1.push_back(path_pt);
            }
        }
        else
        {
            for (float pt_x = center_start_pt.x+area_length; pt_x >= center_start_pt.x; pt_x = pt_x-path_dist)
            {
                cv::Point2f path_pt;
                path_pt.x = pt_x;
                path_pt.y = lines_num*(range_1+range_2+range_3) + 0.5*range_1;
                single_path_1.push_back(path_pt);
            }
        }
    }
    else
    {
        if (search_direction == 1)
        {
            for (float pt_x = center_start_pt.x; pt_x <= area_length+center_start_pt.x; pt_x = pt_x+path_dist)
            {
                cv::Point2f path_pt;
                path_pt.x = pt_x;
                path_pt.y = lines_num*(range_1+range_2+range_3) + range_2 + 0.5*range_1;
                single_path_1.push_back(path_pt);
                path_pt.x = pt_x;
                path_pt.y = lines_num*(range_1+range_2+range_3) + range_2*0.5;
                single_path_2.push_back(path_pt);
            }
        }
        else
        {
            for (float pt_x = center_start_pt.x+area_length; pt_x >= center_start_pt.x; pt_x = pt_x-path_dist)
            {
                cv::Point2f path_pt;
                path_pt.x = pt_x;
                path_pt.y = lines_num*(range_1+range_2+range_3) + range_2 + 0.5*range_1;
                single_path_1.push_back(path_pt);
                path_pt.x = pt_x;
                path_pt.y = lines_num*(range_1+range_2+range_3) + range_2*0.5;
                single_path_2.push_back(path_pt);
            }
        }
    }
    
    /* 输出结果 */
    paths_result.push_back(single_path_1);
    paths_result.push_back(single_path_2);
    paths_result.push_back(single_path_3);
    paths_result.push_back(single_path_4);
    return paths_result;
}