
#include "Interact.h"


Interact::Interact()
{
	InteractTask = TaskType::SetTargetType;
}

Interact::~Interact()
{

}

void Interact::MoveTo()
{
//cv::Point desternation;
auto cv_start_pt=transfer_.LngAndlat2CV(g_BlackBoardAgent->current_pt[MoveVehicleIndex]);
auto cv_target_pt=transfer_.LngAndlat2CV(MoveToPt);
//局部雷位置，没有需要考虑的则置空
std::vector<cv::Point> local_mine_pos;
//局部不可通行位置，没有需要考虑的则置空
std::vector<cv::Point> forbidden_area_pos;

std::vector<cv::Point2f> traj=module.go_to_target(cv_start_pt,cv_target_pt,local_mine_pos, forbidden_area_pos);
//TODO 移动到新位置后如何处理

}

void Interact::Resume()
{
//弃用

}

void Interact::SetTargetType()
{
	//根据信息赋值CanBeRecognized NeedEliminated
	std::cout << "set target started!" <<std::endl;
	if (!g_RecognizeAgent->CanBeRecognized)
	{
		if(!g_BlackBoardAgent->new_scene)
			g_RecognizeAgent->AddToRevisit();
		else
			g_RecognizeAgent->AddRevisitForNewScene();
	}
	else 
	{
		if (g_RecognizeAgent->NeedEliminated)
		{
			if (g_BlackBoardAgent->capacity[ManualIndex] > 0)
				g_RecognizeAgent->Eliminate();//SetTypeCarIndex
			else
				g_RecognizeAgent->AddToEliminate();
		}
	}
	//返回原先的模式
	Resume();
}



void Interact::TurnBack()
{
	//1.返回规划
	//局部雷位置，没有需要考虑的则置空
	std::vector<cv::Point> local_mine_pos;
	//局部不可通行位置，没有需要考虑的则置空
	std::vector<cv::Point> forbidden_area_pos;
	auto cv_start_pt=transfer_.LngAndlat2CV(g_BlackBoardAgent->current_pt[MoveVehicleIndex]);
	auto cv_end_pt=transfer_.LngAndlat2CV(g_BlackBoardAgent->end_pt);
	manual_traj[ManualIndex]=module.go_to_target(cv_start_pt,cv_end_pt,local_mine_pos, forbidden_area_pos);
	//2.车辆失能流程
	//读当前任务，更新当前任务状态，更新重规划标志位
	if(g_BlackBoardAgent->StateVector[MoveVehicleIndex]==g_BlackBoardAgent->CoverFlag)
	{
		//删除覆盖任务中的此艇
		for(int i=0;i<g_CoverageAgent->VehicleID.size();i++)
		{
			if(g_CoverageAgent->VehicleID[i]==MoveVehicleIndex)
				g_CoverageAgent->VehicleID.erase(g_CoverageAgent->VehicleID.begin()+i);
		}
		//覆盖重规划
		g_CoverageAgent->Replan=true;
	}
	else
	{
		//删除重访任务中的此艇
		for(int i=0;i<g_RevisitAgent->VehicleID.size();i++)
		{
			if(g_RevisitAgent->VehicleID[i]==MoveVehicleIndex)
				g_RevisitAgent->VehicleID.erase(g_CoverageAgent->VehicleID.begin()+i);
		}
		//重访重规划
		g_RevisitAgent->Replan=true;
	}
}


