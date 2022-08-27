#include "Coverage.h"



Coverage::Coverage()
{

}

Coverage::~Coverage()
{

}

void Coverage::DevideArea()
{

     // for(auto pt:g_BlackBoardAgent->CVArea)
     //      cout<<pt<<endl;
    divided_areas=module.Module_Divide(g_BlackBoardAgent->CVArea,g_BlackBoardAgent->VehiclesNum,g_BlackBoardAgent->formation_flag, g_BlackBoardAgent->perception_radius);
     // for(auto area:divided_areas)
     // {
     //      std::cout<<"area:"<<std::endl;
     //      for(auto pt:area)
     //           std::cout<<pt<<std::endl;
     // }

}

void Coverage::FindCoveragePath()
{
     //添加保护
     if(!g_CoverageAgent->VehicleID.size())
          return;
     // std::cout << "Cover vehicle num: "<< g_CoverageAgent->VehicleID.size() << std::endl;
     // std::cout << "Cover vehicle flag: "<< g_CoverageAgent->Replan << std::endl;

     cv::Mat3b _vis_map;
    std:: vector<cv::Point>current_cv_pts;
//     std::cout << "find coverage path" << std::endl;
     for(auto pt:g_BlackBoardAgent->current_pt)
     {
          
          auto current_cv_pt=transfer_.LngAndlat2CV(pt);
          // std::cout << "pt:" << current_cv_pt.x << "," << current_cv_pt.y << std::endl;
          // auto current_cv_pt = cv::Point(230, 300);
          current_cv_pts.push_back(current_cv_pt);

          cv::Point2f start_pt;
          // start_pt.x = current_cv_pt.x;
          // start_pt.y = current_cv_pt.y;
          start_pt.x = 0;
          start_pt.y = 0;
          g_BlackBoardAgent->start_pts.push_back(start_pt);
     }
     // std::cout << "current_cv_pts_size:" << current_cv_pts.size() << std::endl;
     module.plannerInit(g_BlackBoardAgent->VehiclesNum, g_BlackBoardAgent->start_pts);
     CoveragePath=module.Module_Cover(divided_areas,g_BlackBoardAgent->formation_flag,_vis_map,group_msg,current_cv_pts, g_BlackBoardAgent->VehiclesNum);
	// for(auto index:VehicleID)
	// 	std::cout<<  "VehicleID" << index<<' ';
	// std::cout<<std::endl;
     // std::cout << "cover_finished" << std::endl;
}

void Coverage::SetGroup()
{
     // std::cout << "Replan flag: "<< Replan << std::endl;
     // std::cout<<" A new cover !!!!!!!!!!!!!"<<std::endl;
    group_msg=module.Module_Group(g_BlackBoardAgent->formation_flag, g_BlackBoardAgent->LateralPerception);
}


