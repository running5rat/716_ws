#ifndef _MODULE_H_
#define _MODULE_H_

#include "Cover.hpp"
#include "Data.hpp"
#include "Divider.hpp"
#include "VRP.hpp"
#include "Transfer.hpp"

extern Transfer transfer_;

struct Divide_Sturct {
    vector<cv::Point> area;
    std::pair<int, int> indexes;
};


class Module
{
    public:
        cv::Mat3b vis_map;
        const int RevisitTime=100;  //处理障碍作业时间，单位：秒
        std::vector<std::pair<int,int>> Module_Group(bool formation_flag,std::vector<bool> LateralPerception);
       vector<vector<cv::Point>> Module_Divide(vector<Point> visual_area, int vehicle_num, bool formation_flag, double perception_radius);
        std::vector<std::vector<cv::Point2f>> Module_Cover(std::vector<std::vector<cv::Point>> &divided_areas,bool formation_flag,cv::Mat3b &_vis_map, \
                                                                                            std::vector<std::pair<int,int>> &group_msg,std::vector<cv::Point> &start_pts, int vehicle_num);
        std::vector<std::vector<cv::Point>> Module_Solve(std::vector<std::vector<double>> for_revisit,  std::vector<double> speed, cv::Mat3b &_vis_map, std::vector<int>cover_rest_time,\
                                                                                                                int time_left, int vehicle_num, std::vector<int> capacity, std::vector<int>WorkTime, std::vector<int> Delivery, std::vector<int> Priority,\
                                                                                                                std::vector<std::vector<int>> Skill, double max_lat, double min_lat);
        std::vector<std::vector<cv::Point>> Module_Solve_New_Scene(std::vector<std::vector<double>> for_revisit,  std::vector<double> speed, cv::Mat3b &_vis_map, std::vector<int>cover_rest_time, \
                                                                                                                int time_left, int vehicle_num, std::vector<int> capacity, std::vector<int>WorkTime, std::vector<int> Delivery, std::vector<int> Priority,\
                                                                                                                std::vector<std::vector<int>> Skill, double max_lat, double min_lat);
        vector<Point2f> pointset_input(vector<Point> area,int vehicle_num,bool formation_flag);
        std::vector<std::pair<int,int>> Group(vector<bool> LateralPerception);


        /* 直接到达指定点 */
        cv::Point construct_Bspline(cv::Point ctrl0, cv::Point ctrl1, cv::Point ctrl2, cv::Point target, std::vector<cv::Point> local_mines_pos, std::vector<cv::Point> forbidden_area); // 轨迹生成升级版
        std::vector<cv::Point2f> go_to_target(cv::Point start_pt, cv::Point target_pt, std::vector<cv::Point> local_mine_pos, std::vector<cv::Point> forbidden_area_pos);

        /* ----- 新增加的变量 ----- */
        cv::Mat3b visual_graph; // 可视化地图
        std::vector<bool> formulated_ok; // 符合编队条件
        std::vector<int> target_pt_idx;     // 各船当前的目标点序列号
        bool cover_processing;  // 是否正在覆盖中
        std::vector<std::vector<cv::Point2f>> control_pts_now;   // 各船当前控制点
        std::vector<std::vector<cv::Point2f>> control_pts_buffer;   // 各船一段控制点
        std::vector<std::vector<cv::Point2f>> follower_control_pts_buffer;   // 跟随者一段控制点
        std::vector<std::vector<cv::Point2f>> follower_control_pts_now;   // 跟随者一段控制点
        std::vector<cv::Point2f> leader_pos_now, follower_pos_now;   // 当前位置
        cv::Point2f formulate_start_pt; // 再编队起点
        std::vector<bool> vehicle_cover_finished;   // 各车是否完成覆盖
        std::vector<bool> vehicle_cover_finished_for_revisit;   // 完成覆盖进入重访的标志
        std::vector<bool> vehicle_revisit_finished_for_end; // 完成重访结束任务的标志
        std::vector< bool> mines_check; // 测试：重复障碍标志位
        std::vector< bool> virtual_mines_check; // 测试：虚雷重复障碍标志位
        std::vector<bool> vehicle_revisiting;   // 各车是否完成覆盖
        std::vector<std::vector<cv::Point2f>> all_cover_paths;
        std::vector<std::vector<cv::Point2f>> received_local_mines_forAll;
        std::vector<std::vector<int>> received_dangerous_radiuses;
        std::vector<cv::Point2f> poses_now; // 当前位置
        std::vector<std::vector<cv::Point2f>> revisit_trajs_out; 
        bool form_bool;
        std::vector<cv::Point2f> gloable_mines; // 测试：全局障碍
        std::vector<cv::Point2f> gloable_mines_real;
        std::vector<cv::Point2f> virtual_gloable_mines_real;   // 新场景：虚雷
        std::vector<std::vector<cv::Point2f>> trajectory_output;
        bool going_to_start;
        std::vector<double> last_vels_x;
        std::vector<double> last_vels_y;
        double last_vel_x, last_vel_y;
        std::vector<int> rest_time_in_cover;

        std::vector<double> safe_place_temp;
        void Get_Safe_Place(std::vector<int> unassigned,  LocationTrans _location_trans, std::vector<double> &safe_place_temp, double max_lat, double min_lat); // 给出安全距离
        void Output_Safe_Place(std::vector<double> &safe_length);

        /* ----- 新增加的函数 ----- */
        
        
        bool mines_near(cv::Point2f target_pos, std::vector<cv::Point2f> mines);    // 编队时周围是否有障碍
        cv::Point2f follow_leader(cv::Point2f follower_pos, cv::Point2f target_pos );   // 编队执行
        void planLoop(std::vector<std::vector<cv::Point2f>> paths_pts, bool formulated, int vehicle_num,  int area_num, std::vector<cv::Point2f> simulation_mines, std::vector<int> simulation_mines_radius);    // 更新定时流程函数
        void visit_plan(std::vector<std::vector<cv::Point>> target_pos,cv::Mat3b &vis_map);
        void test_revist();

        
        inline Module(){
            form_bool = false;  // 编队标志
        } 


        /* 有用的变量 */
        std::vector<cv::Point2f> real_poses;

        /* 有用的函数 */
        void plannerInit(int vehicle_num, std::vector<cv::Point2f> start_pos); // 初始化module里的值
        bool inside_mines(cv::Point2f target_pt, std::vector<cv::Point2f> mines);   // 判断目标点是否在雷区内 
        std::vector<cv::Point2f> construct_Bspline(cv::Point2f ctrl_pt0, cv::Point2f ctrl_pt1, cv::Point2f ctrl_pt2, cv::Point2f target_pt, std::vector<cv::Point2f> local_mines_pos, std::vector<int> forbidden_area, float sim_vel); // 生成一小段B样条曲线
        std::vector<std::vector<cv::Point2f>> construct_trajectory(cv::Point2f ctrl_pt0, cv::Point2f ctrl_pt1, cv::Point2f ctrl_pt2, cv::Point2f target_pt, std::vector<cv::Point2f> local_mines, std::vector<int> dangerous_area, float sim_vel);  // 计算一段轨迹
        void planloop_approach(cv::Point2f target_pos, int vehicle_idx, std::vector<cv::Point2f> other_mines, std::vector<int> other_mines_radius, float sim_vel);    // 到达目标点函数
        void planloop_single_cover(std::vector<cv::Point2f> cover_path, int vehicle_idx, std::vector<cv::Point2f> local_mines_pos, std::vector<int> local_mines_radius, float sim_vel);  // 单艇循迹覆盖
        void visualize();

        /* 人工选择变量与函数 */
        std::vector<std::vector<cv::Point2f>> cover_path_choice_first;
        std::vector<std::vector<cv::Point2f>> cover_path_choice_second;
        void choose_plan();

        /* 200米通道 */
        cv::Point2f center_start_pt;    // 通道起点
        std::vector<cv::Point2f> safe_channel;  // 通道范围点

        /* 时间上报 */
        float revisit_rest_time;
        float total_time_need;
        void report_time_left();

        std::vector<std::pair<double, double>> vehicle_heading;

        // 关于边覆盖边重访的变量和函数
        double jinqujuli = 100;
        bool MinesEliminateDecision(cv::Point2f mines_pos, vector<cv::Point2f> other_mines_pos, cv::Point2f desternation, vector<cv::Point2f> area, double w_cost_contain, double w_cost_distance2desternation, \
								double w_cost_distance2centerline, double thresthold);
        int VehicleEliminateDecision(cv::Point2f mines_pos ,vector<cv::Point2f> vehicle_pos, vector<pair<double,double>> vehicle_heading, vector<int> capacity,double w_cost_capacity, double w_cost_distance2vehicle,double w_cost_heading);

        // 覆盖中需要清除的雷的相关参数
        std::vector<cv::Point2f> Eliminate_Mines_In_Cover;
        std::vector<bool> Eliminate_Cars_In_Cover;

        /* 重新初始化 */
        void Reinit(int vehicle_num);
};




#endif // !_MODULE_H_

