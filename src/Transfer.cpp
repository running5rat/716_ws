#include "Transfer.hpp"


//NOTE transfer类核心api
cv::Point2f Transfer::LngAndlat2CV(LatLngPoint input_pt)//lat1第一个点纬度,lng1第一个点经度
{
    double lat=input_pt.lat;
    double lng=input_pt.lng;

    double x_dis=(lng-origin.lng)* pi /180.0*(EARTH_RADIUS*cos(lat* pi /180.0))*1000; //忽略了delta_lat
    double y_dis=(origin.lat-lat)* pi /180.0*EARTH_RADIUS*1000;

    cv::Point2f CV_pt;
    CV_pt.x=x_dis;
    CV_pt.y=y_dis;
    return CV_pt;
}

LatLngPoint  Transfer::CV2LngAndlat(cv::Point2f CvPt){
    LatLngPoint ret_pt;
    ret_pt.lat=origin.lat-CvPt.y*180/EARTH_RADIUS/ pi/1000;
    ret_pt.lng=origin.lng+CvPt.x/pi *180.0/(EARTH_RADIUS*cos(ret_pt.lat* pi /180.0))/1000;
    return ret_pt;
}

void Transfer::Reinit(double max_lat,double min_lat,double min_lng,double max_lng){
        //NOTE 只适用北半球
        //计算原点origin
        origin.lat=max_lat;
        origin.lng=min_lng;
    }

double Transfer::RealDistance(LatLngPoint p1,LatLngPoint p2)//lat1第一个点纬度,lng1第一个点经度,lat2第二个点纬度,lng2第二个点经度
{
	
	double a;
   	double b;
    double pi = 3.1415926535897932384626433832795;
   	double radLat1 = p1.lat * pi / 2;
   double radLat2 = p2.lat * pi / 2;
   a = radLat1 - radLat2;
   b = p1.lng * pi / 2 - p2.lng * pi / 2 ;
   double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
   double EARTH_RADIUS = 6378.137;
    s = s * EARTH_RADIUS;
    s = s * 1000;
    return s;
}