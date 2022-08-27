#ifndef _DIVIDER_H_
#define _DIVIDER_H_

#include <bits/stdc++.h>
#include <unordered_set>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class PtHash
{
    public:
    size_t operator()(const Point &key) const
    {
        return hash<int>{}(key.x) ^ hash<int>{}(key.y);
    }
};

class Divider
{
private:
    void drawPointSet(Mat& img, vector<Point2f> pointSet, Scalar color);
    void drawSubdiv(Mat& img, Subdiv2D& subdiv, Scalar delaunay_color);
    void paintVoronoi(Mat& img, Subdiv2D& subdiv, vector<vector<Point2f> > &facets,vector<vector<Point> > &output);
    vector<vector<cv::Point>>  Divide_Area(Rect rect,vector<Point> whole_area,vector<Point2f> pointset,double perception_radius);
    vector<Point2f> preprocess(vector<Point2f> Origin_pointset);


    //intersection
    bool GetCrossPoint(const Point &a,const Point &b,const Point &c,const Point &d,long &x,long &y);
    bool IsPointInPolygon(std::vector<Point> poly,Point pt);
    bool PointCmp(const Point &a,const Point &b,const Point &center);
    void ClockwiseSortPoints(std::vector<Point> &vPoints);
    bool PolygonClip(const vector<Point> &poly1,const vector<Point> &poly2, std::vector<Point> &interPoly);
public:
    Divider( vector<Point2f> pointset_input);
    ~Divider();
    vector<vector<cv::Point>> Divide_Loop(vector<Point> area, double perception_radius);
    //智能体位置
    vector<Point2f> pointset;
    vector<Point2f> pointset_input(vector<Point> area,int vehicle_num,bool formation_flag);
    std::vector<std::pair<int,int>> Group(vector<bool> LateralPerception);
};

#endif // !_DIVIDER_H_