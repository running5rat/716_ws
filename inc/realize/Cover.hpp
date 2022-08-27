#ifndef _COVER_H_
#define _COVER_H_

/* INCLUDE */
#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>
#include <numeric>
#include <cmath>

/* OpenCV头文件 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/* Eigen头文件 */
// #include <Eigen/Core>

using namespace cv;
using namespace std;

#define PI acos(-1)

/* 覆盖搜索类 */
class Cover
{
    private:
        /* 变量 */
        int map_size_x; // 地图x尺寸
        int map_size_y; // 地图y尺寸
        cv::Mat1b map;  // 地图
        // cv::Mat3b vis_map;  // 可视化地图
        cv::Mat3b rotated_map;  // 旋转后地图
        cv::RotatedRect OBB_box;    // 任务区OBB包围盒
        float OBB_angle;    // OBB包围盒的旋转
        cv::Point left_point;   // 左上起始点
        cv::Point right_point; // 右下终止点
        cv::Point top_point;   // 左上起始点
        cv::Point bottom_point; // 右下终止点
        std::vector<std::vector<cv::Point>> target_contours;    // 任务区轮廓
        std::vector<std::vector<cv::Point>> target_contours_rotated;    // 旋转后任务区轮廓
        std::deque<std::deque<cv::Point>> searching_path;   // 覆盖路径
        cv::Point pos_now;  // 当前位置
        cv::Point follower_pos_now; // follower当前位置
        std::vector<cv::Point> target_area;//目标区域

        /* 函数 */
        void construct_contours();   // 地图障碍多边形区域构建
        void find_searching_path(cv::Point leader_pt, bool NS_or_WE, float perception_dist);
        void plan_trajectory(std::vector<Point> &output_trajectory,bool formation_flag,std::vector<cv::Point> mines_local); // 平滑轨迹生成
        void avoid_collision(); // 障碍物避碰
        cv::Point follow_leader(cv::Point follower_pos, cv::Point target_pos);   // 编队控制
        cv::Point construct_Bspline(cv::Point ctrl0, cv::Point ctrl1, cv::Point ctrl2, cv::Point target, std::vector<cv::Point> local_mines_pos, std::vector<cv::Point> forbidden_area, int idx); // 轨迹生成升级版
         void go_to_target(cv::Point start_pt, cv::Point target_pt, std::vector<cv::Point> local_mine_pos, std::vector<cv::Point> forbidden_area_pos, int idx);
         
    public:
        std::vector<std::vector<cv::Point2f>> trajs;
        std::vector<cv::Point> mines;
        /* 流程函数 */
        cv::Mat3b vis_map;  // 可视化地图
        // std::vector<Point> CoverLoop(bool formation_flag=false);
        std::vector<cv::Point2f> CoverLoop(bool formation_flag, cv::Point leader_pt, cv::Point follower_pt, float percept_area);
        Cover(std::vector<cv::Point> input_area);
        ~Cover();

        std::vector<std::vector<cv::Point2f>> construct_lozenge_paths(cv::Point2f center_start_pt, float width_up, float width_down, float length_forward, float length_back, float path_dist);
        std::vector<std::vector<cv::Point2f>> construct_full_path(float area_length, float area_width, cv::Point2f center_start_pt, float range_1, float range_2, float range_3, float range_4, float path_dist);
};
#endif // !_COVER_H_