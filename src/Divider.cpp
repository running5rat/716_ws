#include "Divider.hpp"


Divider::Divider( vector<Point2f> pointset_input)
{
    pointset=pointset_input;
}

Divider::~Divider()
{
}

/* ************************************************************************************************************8 */
//划分区域
vector<Point2f> Divider::pointset_input(vector<Point> area,int vehicle_num,bool formation_flag){
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

//分组函数：尽可能保证每组有一个侧扫
std::vector<std::pair<int,int>> Divider::Group(vector<bool> LateralPerception){
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
/* ************************************************************************************************* */

//画出点集
void Divider::drawPointSet(Mat& img, vector<Point2f> pointSet, Scalar color)
{
    for (int i = 0; i < pointSet.size();i++)
    {
        cv::circle(img, pointSet[i], 3, color, cv::FILLED, 8, 0);
    }
}

//画出剖分
void Divider::drawSubdiv(Mat& img, Subdiv2D& subdiv, Scalar delaunay_color)
{
    vector<Vec6f> triangleList;
    //NOTE opencv核心api:计算三角剖分
    subdiv.getTriangleList(triangleList);
    vector<Point> pt(3);

    for (size_t i = 0; i < triangleList.size(); i++)
    {
        Vec6f t = triangleList[i];
        pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
        pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
        pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
        line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
        line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
        line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
    }
}

//画出Voronoi图
void Divider::paintVoronoi(Mat& img, Subdiv2D& subdiv, vector<vector<Point2f> > &facets,vector<vector<Point> > &output)
{
    // vector<vector<Point2f> > facets;
    vector<Point2f> centers;
    //NOTE opencv核心api:计算voronoi图
    subdiv.getVoronoiFacetList(vector<int>(), facets, centers);

    vector<Point> ifacet;
    vector<vector<Point> > ifacets(1);

    for (size_t i = 0; i < facets.size(); i++)
    {
        ifacet.resize(facets[i].size());
        for (size_t j = 0; j < facets[i].size(); j++)
            ifacet[j] = facets[i][j];

        Scalar color;
        color[0] = rand() & 255;
        color[1] = rand() & 255;
        color[2] = rand() & 255;
        fillConvexPoly(img, ifacet, color, 8, 0);

        ifacets[0] = ifacet;

        polylines(img, ifacets, true, Scalar(), 1, CV_AA, 0);
        circle(img, centers[i], 3, Scalar(), CV_FILLED, CV_AA, 0);
        output.push_back(ifacet);
        // for(auto t:ifacet)
        //  cout<<t<<endl;
        // cout<<endl;
    }
}

vector<vector<cv::Point>>  Divider::Divide_Area(Rect rect,vector<Point> whole_area,vector<Point2f> pointset,double recog_radius){
    //新版分区，只针对矩形 by lxl
    vector<vector<cv::Point>> divided_areas;

    int total_id = pointset.size();
    double minx = min(min(whole_area[0].x, whole_area[1].x), whole_area[2].x);
    double maxx = max(max(whole_area[0].x, whole_area[1].x), whole_area[2].x);

    double miny = min(min(whole_area[0].y, whole_area[1].y), whole_area[2].y);
    double maxy = max(max(whole_area[0].y, whole_area[1].y), whole_area[2].y);

    // cout<<"minx"<<minx<<endl;
    // cout<<"maxx"<<maxx<<endl;
    // cout<<"miny"<<minx<<endl;
    // cout<<"maxy"<<maxx<<endl;
    
    double length = maxx- minx;
    double width = maxy- miny;
    double max_leng = length > width ? length : width;

    int times = max_leng / (recog_radius * total_id);
    // cout<<"max_leng"<<max_leng<<endl;
    //     cout<<"total_id"<<total_id<<endl;
    //             cout<<"recog_radius"<<recog_radius<<endl;
    //                 cout<<"times"<<times<<endl;
    double remainder= max_leng -times* (recog_radius * total_id);
    double per_remainder = remainder/pointset.size();
    double per_dis= times * recog_radius + per_remainder;
                        // cout<<"per_dis"<<per_dis<<endl;
    if (length > width)
    {
        for (int index = 1; index <= total_id; index++)
        {
            double temp_x = minx + index * per_dis;

            vector<Point> interPoly;
            Point temp_p;

            temp_p.x = temp_x -  per_dis;
            temp_p.y = miny;
            interPoly.push_back(temp_p);

            temp_p.x = temp_x -  per_dis;
            temp_p.y = maxy;
            interPoly.push_back(temp_p);

            temp_p.x = temp_x;
            temp_p.y = maxy;
            interPoly.push_back(temp_p);

            temp_p.x = temp_x;
            temp_p.y = miny;
            interPoly.push_back(temp_p);
            // cout<<"interPoly"<<endl;
            // for(auto &pt:interPoly)
            //     cout<<"inter_pt"<<pt<<endl;
            ClockwiseSortPoints(interPoly);
            divided_areas.push_back(interPoly);
        }
    }
    else
    {
        for (int index = 1; index <= total_id; index++)
        {
            double temp_y = miny + index * per_dis;

            vector<Point> interPoly;
            Point temp_p;

            temp_p.x = minx;
            temp_p.y = temp_y - per_dis;
            interPoly.push_back(temp_p);

            temp_p.x = maxx; 
            temp_p.y = temp_y -  per_dis;
            interPoly.push_back(temp_p);

            temp_p.x = maxx;
            temp_p.y = temp_y;
            interPoly.push_back(temp_p);

            temp_p.x = minx;
            temp_p.y = temp_y;
            interPoly.push_back(temp_p);

            // cout<<"interPoly"<<endl;
            // for(auto &pt:interPoly)
            //     cout<<"inter_pt"<<pt<<endl;

            ClockwiseSortPoints(interPoly);
            divided_areas.push_back(interPoly);
        }
    }

    //旧版分区

    //Mat img(rect.size(), CV_8UC3);
    //img = Scalar::all(0);

    //Mat img_ptst = img.clone();
    //drawPointSet(img_ptst,pointset,Scalar(0,255,0));
    //imshow("Point set",img_ptst);
    //imwrite("pointSet.jpg",img_ptst);

    ////创建Delaunay剖分
    //Subdiv2D subdiv(rect);
    //for (int i = 0; i < pointset.size();i++)
    //{
    //    subdiv.insert(pointset[i]);
    //}

    ////画出Delaunay剖分三角形
    //Mat img_delaunay = img.clone();
    //drawSubdiv(img_delaunay, subdiv, Scalar(255,255,255));
    //imshow("Delaunay", img_delaunay);
    //imwrite("delaunay.jpg", img_delaunay);

    ////画出Voronoi图
    //Mat img_voronoi = img.clone();
    //vector<vector<cv::Point>> divided_areas;

    //vector<vector<cv::Point2f>> temp_areas;
    //paintVoronoi(img_voronoi, subdiv,temp_areas,divided_areas);


    // for(auto area:divided_areas)
    // {
    //      std::cout<<"origin area:"<<std::endl;
    //      for(auto pt:area)
    //           std::cout<<pt<<std::endl;
    // }

    ////ANCHOR 求与区域交集
    //for(auto &divided_area:divided_areas )
    //{
    //    std::vector<Point> interPoly;

    //    if(PolygonClip(divided_area,whole_area,interPoly))
    //        divided_area=interPoly;
    //    else
    //        cout<<"Intersection Process Error"<<endl;
    //}



    //imshow("Voronoi", img_voronoi);
    //imwrite("voronoi.jpg", img_voronoi);
    return divided_areas;
}

vector<Point2f> Divider::preprocess(vector<Point2f> Origin_pointset){
    vector<Point2f> aft_pointset;

    for(int i=0;i<Origin_pointset.size();i++)
    {
        float x_ori=Origin_pointset[i].x;
        float y_ori=Origin_pointset[i].y;

        bool dist_ok = false;
        float x_aft;
        float y_aft;
        while (! dist_ok)
        {

                x_aft= (float)(rand() % 200 ) + x_ori - 100.0;
                y_aft= (float)(rand() % 200 ) + y_ori - 100.0;

            bool all_in = true;
            for (int j = 0; j < i; j++)
            {
                if (sqrt((aft_pointset[j].x - aft_pointset[i].x) * (aft_pointset[j].x - aft_pointset[i].x) +\
                 (aft_pointset[j].y - aft_pointset[i].y) * (aft_pointset[j].y - aft_pointset[i].y)) < 100)
                {
                    all_in = false;
                    break;
                }
            }
            if (all_in) dist_ok = true;
        }
        Point2f fp(x_aft, y_aft);
        aft_pointset.push_back(fp);

    }
    return aft_pointset;
}

vector<vector<cv::Point>> Divider::Divide_Loop(vector<Point> area, double perception_radius){
    Rect rect(0,0,500,500);
    
    //位置预处理

    //vector<Point2f> aft_preprocess_pointset=preprocess(pointset);
    //划分区域
    vector<vector<cv::Point>> divided_areas;
    // cout<<"area"<<endl;
    // for(auto pt:area)
    //     cout<<pt<<endl;
    divided_areas=Divide_Area(rect,area,pointset, perception_radius);
    return divided_areas;
}

//4 debug:
// int main(){
//     auto pts=pointset_input();
//     Divider divider(pts);
//     auto divided_areas=divider.Divide_Loop();
//     waitKey(0);
//     return 1;
// }




//求线段交点,如果有交点返回坐标
bool Divider::GetCrossPoint(const Point &a,const Point &b,const Point &c,const Point &d,long &x,long &y)
{
/** 1 解线性方程组, 求线段交点. **/  
// 如果分母为0 则平行或共线, 不相交  
    int denominator = (b.y - a.y)*(d.x - c.x) - (a.x - b.x)*(c.y - d.y);  
    if (denominator==0) {  
        return false;  
    }  

// 线段所在直线的交点坐标 (x , y)      
    x = ( (b.x - a.x) * (d.x - c.x) * (c.y - a.y)   
                + (b.y - a.y) * (d.x - c.x) * a.x   
                - (d.y - c.y) * (b.x - a.x) * c.x ) / denominator ;  
    y = -( (b.y - a.y) * (d.y - c.y) * (c.x - a.x)   
                + (b.x - a.x) * (d.y - c.y) * a.y   
                - (d.x - c.x) * (b.y - a.y) * c.y ) / denominator;  

/** 2 判断交点是否在两条线段上 **/  
    if (  
        // 交点在线段1上  
        (x - a.x) * (x - b.x) <= 0 && (y - a.y) * (y - b.y) <= 0  
        // 且交点也在线段2上  
         && (x - c.x) * (x - d.x) <= 0 && (y - c.y) * (y - d.y) <= 0  
        ){  

        // 返回交点p  
        return  true;
    }  
    //否则不相交  
    return false;


}

//判断点在区域内
bool Divider::IsPointInPolygon(std::vector<Point> poly,Point pt)
{
    int i,j;
    bool c = false;
    for (i = 0,j = poly.size() - 1;i < poly.size();j = i++)
    {
        if ((((poly[i].y <= pt.y) && (pt.y < poly[j].y)) ||
            ((poly[j].y <= pt.y) && (pt.y < poly[i].y)))
            && (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y)/(poly[j].y - poly[i].y) + poly[i].x))
        {
            c = !c;
        }
    }
    return c;
}



//若点a大于点b,即点a在点b顺时针方向,返回true,否则返回false
// bool Divider::PointCmp(const Point &a,const Point &b,const Point &center)
// {
//     // if (a.x >= 0 && b.x < 0)
//     //     return true;
//     // if (a.x == 0 && b.x == 0)
//     //     return a.y > b.y;
//     //向量OA和向量OB的叉积
//     int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
//     if (det < 0)
//         return true;
//     if (det > 0)
//         return false;
//     //向量OA和向量OB共线，以距离判断大小
//     int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
//     int d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
//     return d1 > d2;
// }


//按照逆时针排序
void Divider::ClockwiseSortPoints(std::vector<Point> &vPoints)
{
    //去重
    std::vector<Point> NRepeatPt;
    unordered_set<Point,PtHash> set_;
    for(auto &pt:vPoints)
            if(set_.find(pt)==set_.end())
            {
                set_.insert(pt);
                NRepeatPt.push_back(pt);
            }

    //计算重心
    cv::Point center;
    double x = 0,y = 0;
    for (int i = 0;i < NRepeatPt.size();i++)
    {
        x += NRepeatPt[i].x;
        y += NRepeatPt[i].y;
    }
    center.x = (int)x/NRepeatPt.size();
    center.y = (int)y/NRepeatPt.size();


    //ANCHOR 逆时针排序：
    //NOTE center和点重合可能会出现问题;存在合法点斜率大于INT64_MAX可能会出问题
    //排序借助斜率和右侧标志位完成
    std::vector<float> Pt_Slope(NRepeatPt.size());
    std::vector<bool> Pt_Right_Flag(NRepeatPt.size());

    //计算斜率和右侧标志位
    for(int i=0;i < NRepeatPt.size();i++)
    {
        if(NRepeatPt[i].x>center.x)
        {
            Pt_Right_Flag[i]=true;
            Pt_Slope[i]=((float)(NRepeatPt[i].y-center.y))/(NRepeatPt[i].x-center.x);

            if(Pt_Slope[i]>INT64_MAX)
                cout<<"encounter large slope,maybe an Error here!"<<endl;
        }
        else if(NRepeatPt[i].x<center.x)
        {
            Pt_Right_Flag[i]=false;
            Pt_Slope[i]=((float)(NRepeatPt[i].y-center.y))/(NRepeatPt[i].x-center.x);

            if(Pt_Slope[i]>INT64_MAX)
                cout<<"encounter large slope,maybe an Error here!"<<endl;
        }
        else if(NRepeatPt[i].x==center.x)
        {
            //如果斜率不合法，向右偏移一点
            if(NRepeatPt[i].y>center.y)
            {
                Pt_Right_Flag[i]=true;
                Pt_Slope[i]=INT64_MAX;
            }
            else if(NRepeatPt[i].y<center.y)
            {
                Pt_Right_Flag[i]=true;
                Pt_Slope[i]=-INT64_MAX;
            }
            else
            {
                cout<<"Center is Somepoint,there is an Error here!"<<endl;
            }
        }


    }

    //先从左找，斜率从大到小；再从右找，斜率从大到小
    //插入排序:
    std::vector<int> Order_LPt,Order_RPt;
    for(int i=0;i < NRepeatPt.size();i++)
    {
        if(Pt_Right_Flag[i]==true)
        {
            int j=0;
            for(;j<Order_RPt.size();j++)
                if(Pt_Slope[Order_RPt[j]]<Pt_Slope[i])
                    break;
            Order_RPt.insert(Order_RPt.begin()+j,i);
        }
        else
        {
            int j=0;
            for(;j<Order_LPt.size();j++)
                if(Pt_Slope[Order_LPt[j]]<Pt_Slope[i])
                    break;
            Order_LPt.insert(Order_LPt.begin()+j,i);
        }

    }

    //合并两个vector
    vector<int>temp_vec;
    temp_vec.insert(temp_vec.end(),Order_LPt.begin(),Order_LPt.end());
    temp_vec.insert(temp_vec.end(),Order_RPt.begin(),Order_RPt.end());

    //索引转回位置
    vPoints.clear();
    for(auto &index:temp_vec)
        vPoints.push_back(NRepeatPt[index]);
    



    // //冒泡排序
    // for(int i = 0;i < NRepeatPt.size() - 1;i++)
    // {
    //     for (int j = 0;j < NRepeatPt.size() - i - 1;j++)
    //     {
    //         //PointCmp()判断是否顺时针
    //         if (PointCmp(NRepeatPt[j],NRepeatPt[j+1],center))
    //         {
    //             cv::Point tmp = NRepeatPt[j];
    //             NRepeatPt[j] = NRepeatPt[j + 1];
    //             NRepeatPt[j + 1] = tmp;
    //         }
    //     }
    // }
    // vPoints=NRepeatPt;


}

//入口api
bool Divider::PolygonClip(const vector<Point> &poly1,const vector<Point> &poly2, std::vector<Point> &interPoly)
{
    if (poly1.size() < 3 || poly2.size() < 3)
    {
        return false;
    }

    long x,y;
    //计算多边形交点
    for (int i = 0;i < poly1.size();i++)
    {
        int poly1_next_idx = (i + 1) % poly1.size();
        for (int j = 0;j < poly2.size();j++)
        {
            int poly2_next_idx = (j + 1) % poly2.size();
            if (GetCrossPoint(poly1[i],poly1[poly1_next_idx],
                poly2[j],poly2[poly2_next_idx],
                x,y))
            {
                interPoly.push_back(cv::Point(x,y));
            }
        }
    }
    // std::cout<<"interPoly"<<interPoly<<std::endl;
    //计算多边形内部点
    for(int i = 0;i < poly1.size();i++)
    {
        if (IsPointInPolygon(poly2,poly1[i]))
        {
            interPoly.push_back(poly1[i]);
        }
    }
    for (int i = 0;i < poly2.size();i++)
    {
        if (IsPointInPolygon(poly1,poly2[i]))
        {
            interPoly.push_back(poly2[i]);
        }
    }

    if(interPoly.size() <= 0)
        return false;
    //点集排序
    ClockwiseSortPoints(interPoly);

    // for(auto pt:interPoly)
    //     cout<<pt<<endl;
    //     cout<<endl;
    return true;
}
