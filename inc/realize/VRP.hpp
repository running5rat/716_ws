#ifndef _VRP_H_
#define _VRP_H_

#include <iostream>

#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"
#include "structures/vroom/vehicle.h"
#include "utils/exception.h"

#include <jsoncpp/json/json.h>
#include <fstream>

#include <unordered_map>
#include <utility>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #define pi 3.1415926535897932384626433832795
// #define EARTH_RADIUS 6378.137 //地球半径 KM
//NOTE 影响显示分辨率
#define LongEdgePixel 500
//NOTE 必要可视区域与实际可视区域的间隔(经纬度)
#define Interval 0.001

using namespace std;

// std::vector<std::vector<int>> Paths;

class Problem
{
    private:
        unsigned amount_dimension_Scene1 = 0;   //场景一不需要负载
        unsigned amount_dimension_Scene2 = 1;
        int capacity_scene2 = 10;   //场景二对应载弹量
        vroom::Matrix<vroom::Duration> CostMat_Scene1; //场景一邻接代价矩阵
        vroom::Matrix<vroom::Duration> DurationMat;  //场景二邻接代价矩阵
        vroom::Matrix<vroom::Cost> CostMat;

        int veh_num;

        std::vector<vroom::Vehicle> Vehicles;   // 车辆定义向量
        std::vector<vroom::Job> Jobs;

    public:
      std::vector<int> unassigned_mines;
      Problem(std::string FileIn, int SceneNum);    // 从Input.json 中读取数据, 并区分是哪个场景(TODO)
     ~Problem();
     std::vector<std::vector<int>> run_with_custom_matrix();
     std::vector<std::vector<int>> Paths;
     void Unassigned_Mines(const vroom::Solution& sol);

     std::vector<int> My_Duration;
     void Get_Duration(const vroom::Solution& sol);
};




inline void log_solution(const vroom::Solution& sol, bool geometry) {
  std::cout << "Total cost: " << sol.summary.cost << std::endl;
  std::cout << "Unassigned: " << sol.summary.unassigned << std::endl;

  // Log unassigned jobs if any.
  std::cout << "Unassigned job ids: ";
  for (const auto& j : sol.unassigned) {
    std::cout << j.id << ", ";
  }
  std::cout << std::endl;

  // Describe routes in solution.
  for (const auto& route : sol.routes) {
    std::cout << "Steps for vehicle " << route.vehicle
              << " (cost: " << route.cost;
    std::cout << " - duration: " << route.duration;
    std::cout << " - service: " << route.service;
    if (geometry) {
      std::cout << " - distance: " << route.distance;
    }

    std::cout << ")" << std::endl;

    // Describe all route steps.
    for (const auto& step : route.steps) {
      std::string type;
      switch (step.step_type) {
      case vroom::STEP_TYPE::START:
        type = "Start";
        break;
      case vroom::STEP_TYPE::END:
        type = "End";
        break;
      case vroom::STEP_TYPE::BREAK:
        type = "Break";
        break;
      case vroom::STEP_TYPE::JOB:
        switch (step.job_type) {
        case vroom::JOB_TYPE::SINGLE:
          type = "Job";
          break;
        case vroom::JOB_TYPE::PICKUP:
          type = "Pickup";
          break;
        case vroom::JOB_TYPE::DELIVERY:
          type = "Delivery";
          break;
        }
        break;
      }
      std::cout << type;

      // Add job/pickup/delivery/break ids.
      if (step.step_type != vroom::STEP_TYPE::START and
          step.step_type != vroom::STEP_TYPE::END) {
        std::cout << " " << step.id;
      }

      // Add location if known.
      if (step.location.has_coordinates()) {
        std::cout << " - " << step.location.lon() << ";" << step.location.lat();
      }

      std::cout << " - arrival: " << step.arrival;
      std::cout << " - duration: " << step.duration;
      std::cout << " - service: " << step.service;

      // Add extra step info if geometry is required.
      if (geometry) {
        std::cout << " - distance: " << step.distance;
      }
      std::cout << std::endl;
    }
  }
}




inline double rad(double d)
{
    double pi = 3.1415926535897932384626433832795;
    return d * pi /180.0;
}
struct LatLngPoint
{
    double lng;
    double lat;
};

struct CVCoordinate
{
    double x;
    double y;
};


class LocationTrans{
    public:
    std::unordered_map<int,pair<LatLngPoint,LatLngPoint>> VehicleIndex2StartEndPoint;
    std::unordered_map<int,LatLngPoint> JobIndex2Location;
};

//ANCHOR class Json_RW
class JSON_RW
{
    private:
        string ReadFile;
        Json::Value Vehicles, Matrices, Car, Durations, Jobs;
        std::vector<std::vector<int>> DurationMat;
    public:
        JSON_RW()
        {

        }
        void ReadLocationFromJson(string ReadFile,LocationTrans &locationtrans);
        void ReadOutputFromJson(LocationTrans &_location_trans,vector<vector<LatLngPoint>>& transfer_routes, 
                                                                                std::vector<std::vector<int>> Paths);
};

// std::vector<cv::Point2f> visit_plan(std::vector<std::vector<cv::Point>> target_pos,cv::Mat3b &vis_map)
// {
//     std::vector<cv::Point2f> traj_result;
//     /* 构建可视化地图 */
//     // cv::Mat3b vis_map;
//     // vis_map = cv::Mat3b(500, 700);
//     // vis_map.setTo(cv::Scalar(255, 255, 255));
        
//     /* 对每艘艇进行处理 */
//     for (int i = 0; i < target_pos.size(); i++)
//     {
//         /* 计算参考路径点 */
//         std::vector<cv::Point> path_pt;
//         int seg_length = 20;    // 每段路径的参考长度
//         for (int j = 0; j < target_pos[i].size()-1; j++)    // 进行路径划分
//         {
//             int seg_num = 1 + sqrt( pow( (target_pos[i][j+1].x - target_pos[i][j].x), 2 ) + pow( (target_pos[i][j+1].y - target_pos[i][j].y), 2 ) ) / seg_length;   // 两目标间路径分段
//             for (int m = 0; m < seg_num; m++)   // 生成路径点
//             {
//                 int path_pt_x = target_pos[i][j].x + m*(target_pos[i][j+1].x - target_pos[i][j].x)/seg_num;
//                 int path_pt_y = target_pos[i][j].y + m*(target_pos[i][j+1].y - target_pos[i][j].y)/seg_num;
//                 path_pt.push_back(cv::Point(path_pt_x, path_pt_y));
//                 cv::circle(vis_map, cv::Point(path_pt_x, path_pt_y), 3, cv::Scalar(255, 0, 255), cv::FILLED);
//             }
//         }

//         /* 添加雷区杀伤范围可视化 */
//         for (int j = 1; j < target_pos[i].size()-1; j++)
//         {
//             cv::circle(vis_map, target_pos[i][j], 5, cv::Scalar(0, 255, 0), cv::FILLED);
//         }

//         /* 生成轨迹曲线 */
//         cv::Point ctrl_pt0, ctrl_pt1, ctrl_pt2, ctrl_pt3;   // 一段B样条曲线控制点
//         cv::Point pos_now = path_pt[0]; // 当前位置 
//         ctrl_pt0 = path_pt[0];
//         ctrl_pt1 = path_pt[0];
//         ctrl_pt2 = path_pt[0];
//         for (int j = 0; j < path_pt.size(); j++)
//         {
//             /* 检查是否有碰撞 */
//             for (int m = 1; m < target_pos[i].size()-1; m++)
//             {
//                 /* 处理碰撞 */
//                 if ( ( pow((path_pt[j].x - target_pos[i][m].x), 2) + pow((path_pt[j].y - target_pos[i][m].y), 2) ) < (5*5) ) 
//                 {
//                     /* 生成切线 */
//                     double dist_pos_obs = sqrt( pow( (pos_now.x - target_pos[i][m].x), 2 ) + pow( (pos_now.y - target_pos[i][m].y), 2) );
//                     double angle_beta = asin( 6.0/dist_pos_obs );  // 切线与距离夹角
//                     double angle_alpha;
//                     if (!isnan(atan( ((float)pos_now.y - (float)target_pos[i][m].y)/((float)pos_now.x - (float)target_pos[i][m].x) )))  // 若不为90度
//                     {
//                         angle_alpha = atan( ((float)pos_now.y - (float)target_pos[i][m].y)/((float)pos_now.x - (float)target_pos[i][m].x) );
//                     }
//                     else
//                     {
//                         angle_alpha = 0.5*3.1412927;
//                     }
//                     double k1 = angle_alpha + angle_beta; // 障碍边界的倾斜角度1
//                     // std::cout << k1 << std::endl;
//                     double k2 = angle_alpha - angle_beta; // 障碍边界的倾斜角度2
//                     // std::cout << k2 << std::endl;

//                     /* 将路径点从较近一侧推出杀伤范围 */
//                     double dist_on_k1 = pow( (tan(k1) * ((float)path_pt[j+1].x - (float)pos_now.x) - ((float)path_pt[j+1].y - (float)pos_now.y)) , 2)/(1.0+tan(k1)*tan(k1));
//                     double dist_on_k2 = pow( (tan(k2) * ((float)path_pt[j+1].x - (float)pos_now.x) - ((float)path_pt[j+1].y - (float)pos_now.y)) , 2)/(1.0+tan(k2)*tan(k2));
//                     if (dist_on_k1 <= dist_on_k2)
//                     {
//                         /* 将控制点推出障碍集 */
//                         float middle_pt_x = ((float)path_pt[j+1].x + (float)pos_now.x)/2.0;
//                         float middle_pt_y = ((float)path_pt[j+1].y + (float)pos_now.y)/2.0;
//                         float tan_k = -((float)path_pt[j+1].x - (float)pos_now.x)/((float)path_pt[j+1].y - (float)pos_now.y);
//                         // std::cout << "tank:" << tan_k << std::endl;
//                         path_pt[j].x = (tan(k1)*(float)pos_now.x -(float)pos_now.y - tan_k*middle_pt_x + middle_pt_y)/(tan(k1)-tan_k);
//                         path_pt[j].y = tan(k1)*(path_pt[j].x - pos_now.x)+pos_now.y;
//                     }
//                     else
//                     {
//                         /* 将控制点推出障碍集 */
//                         float middle_pt_x = ((float)path_pt[j+1].x + (float)pos_now.x)/2.0;
//                         float middle_pt_y = ((float)path_pt[j+1].y + (float)pos_now.y)/2.0;
//                         float tan_k = -((float)path_pt[j+1].x - (float)pos_now.x)/((float)path_pt[j+1].y - (float)pos_now.y);
//                         // std::cout << "tank:" << tan_k << std::endl;
//                         path_pt[j].x = (tan(k2)*(float)pos_now.x -(float)pos_now.y - tan_k*middle_pt_x + middle_pt_y)/(tan(k2)-tan_k);
//                         path_pt[j].y = tan(k2)*(path_pt[j].x - pos_now.x)+pos_now.y;                        
//                     }
//                     cv::circle(vis_map, path_pt[j], 3, cv::Scalar(255, 255, 0), cv::FILLED);
//                 }
//             } 
//             pos_now = path_pt[j];

//             /* 生成B样条轨迹 */
//             ctrl_pt3 = path_pt[j];
//             for (float t = 0.0; t < 1.0; t=t+0.05)
//             {
//                 float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
//                 float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
//                 cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
//                 traj_result.push_back(cv::Point2f(traj_pt_x, traj_pt_y));
//             }      

//             /* 迭代控制点 */
//             ctrl_pt0 = ctrl_pt1;
//             ctrl_pt1 = ctrl_pt2;
//             ctrl_pt2 = ctrl_pt3;
//         }
        
//         /* 末尾 */
//         for (float t = 0.0; t < 1.0; t=t+0.05)
//         {
//             float traj_pt_x = ((float)ctrl_pt0.x*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.x*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.x*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.x*t*t*t)/6.0;
//             float traj_pt_y = ((float)ctrl_pt0.y*(1-t)*(1-t)*(1-t) + (float)ctrl_pt1.y*(3*t*t*t - 6*t*t + 4) + (float)ctrl_pt2.y*(-3*t*t*t + 3*t*t + 3*t + 1) + (float)ctrl_pt3.y*t*t*t)/6.0;
//             traj_result.push_back(cv::Point2f(traj_pt_x, traj_pt_y));
//             cv::circle(vis_map, cv::Point(traj_pt_x, traj_pt_y), 1, cv::Scalar(255, 0, 0), cv::FILLED);
//         } 
//     }

//     // cv::namedWindow("map1", cv::WINDOW_NORMAL);
//     // cv::imshow("map1", vis_map);
//     return traj_result;
// }
#endif // !_VRP_H_
