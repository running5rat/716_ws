#ifndef _TRANSFER_H_
#define _TRANSFER_H_

#include "VRP.hpp"

//ANCHOR: class transfer
class Transfer{
    public:
    Transfer(double max_lat,double min_lat,double min_lng,double max_lng){
            //NOTE 只适用北半球
            //计算原点origin
            origin.lat=max_lat;
            origin.lng=min_lng;
        }

    double  pi = 3.1415926535897932384626433832795;
    double  EARTH_RADIUS = 6378.137; //地球半径 KM

    void Reinit(double max_lat,double min_lat,double min_lng,double max_lng);
    cv::Point2f LngAndlat2CV(LatLngPoint input_pt);
    LatLngPoint  CV2LngAndlat(cv::Point2f CvPt);
    double RealDistance(LatLngPoint p1,LatLngPoint p2);
    //原点：坐标系对齐用
    LatLngPoint origin;
};

#endif // !_TRANSFER_H_

