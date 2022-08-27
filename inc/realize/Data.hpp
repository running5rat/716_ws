#ifndef _DATA_H_
#define _DATA_H_

#include<iostream>
#include "math.h"
#include <jsoncpp/json/json.h>
#include <fstream>
using namespace std;
// #define pi 3.1415926535897932384626433832795
// #define EARTH_RADIUS 6378.137 //地球半径 KM



class GET_INFO
{
    public:
        struct Point_data
        {
            double lng;
            double lat;
        };

    private:
        Json::Value Vehicles_Mat,  Jobs_Mat, Matrices, Car, Durations, Costs;
        Json::Value Vehicles_Location, Jobs_Location;
        std::vector<Point_data> Location;
        std::vector<std::vector<int>> DurationMat, CostMat;
        
        std::vector<std::vector<double>> JWD;   // 提供了vector标准格式的接口
        /*默认四辆车执行任务，先输入两辆中型艇，再输入两辆小型艇，分别给出车的起始坐标与终止坐标 */
        int VehicleNum, JobNum; // 给出车辆数量和任务数量
       std::vector<std::vector<int>> TimeWindow; //时间窗口
        std::vector<int> ServiceTime; //查证需要的时间
        std::vector<int> Capacity;   // 载弹量
        std::vector<double> Speed;
        std::vector<int> Delivery;
        std::vector<int> Priorty;
        std::vector<std::vector<int>> Skill;

    public:
        double rad_data(double d);
        double RealDistance(double lat1,double lng1,double lat2,double lng2);
        void GetJWDFromFile( std::vector<std::vector<double>> &JWD );

        string MatJson;
        string LocationJson;
        GET_INFO(string MJ, string LJ)
        {
            MatJson = MJ;
            LocationJson = LJ;
        }
        // 目前没有给数据，vector从文件中读取，若直接给定则取消注释
         GET_INFO(string MJ, string LJ, int Veh, int Job, std::vector<std::vector<int>> TW,  std::vector<int> capa, std::vector<std::vector<double>> jwd, std::vector<double> speed, 
                                std::vector<int> Serv, std::vector<int> Deli, std::vector<int> Prior , std::vector<std::vector<int>> skil)
        // GET_INFO(string MJ, string LJ, int Veh, int Job, int *TW, int Serv, int Capa = 0)
        {
            MatJson = MJ;
            LocationJson = LJ;
            VehicleNum =    Veh;
            JobNum = Job;
            TimeWindow.assign(TW.begin(), TW.end());
            ServiceTime.assign(Serv.begin(), Serv.end());
            Capacity.assign(capa.begin(), capa.end());
            JWD.assign(jwd.begin(), jwd.end());
            Speed.assign(speed.begin(), speed.end());
            Delivery.assign(Deli.begin(), Deli.end());
            Priorty.assign(Prior.begin(), Prior.end());
            Skill.assign(skil.begin(), skil.end());
        }
        void GetJWD();  // 读取vector
        void WriteDataToMatJson();
        void WriteDataToLocationJson();
};





#endif // !_DATA_H_
