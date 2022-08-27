#include "Revisit.h"


std::vector<int>  Solution::find_channel_pos() {
	//预处理在通道上下位置加两个雷,不应该改变index和结果
	int max_y=INT_MIN, min_y=INT_MAX;
	for (auto pt : area)
	{
		max_y = max(pt.y, max_y);
		min_y = min(pt.y, min_y);
	}	
	//double top_mine_y = max_y-200;
	//double bottom_mine_y = min_y+200;
	mines.push_back(Point{ 0, max_y });
	mines.push_back(Point{ 0, min_y });
	//如果只生成一个通道滑动窗口就行
	//增序排序
	auto cmp = [&](Point a, Point b) {return a.y < b.y; };
	sort(mines.begin(), mines.end(), cmp);
	if (one_channel) {
		int num = static_cast<int>(mines.size());
		int max_channel_length = INT_MIN;
		int storer_index=0;
		for (int index = 0; index < num - weapon_num-1; index++)
		{
			int channel_length = mines[index + weapon_num + 1].y - mines[index].y;
			std::cout << "channel_length " << channel_length << std::endl;
			if (channel_length > max_channel_length)
			{
				storer_index = index;
				max_channel_length = channel_length;
			}
		}
		priority.resize(num);
		for (int i = storer_index+1; i <= storer_index + weapon_num; i++)
			priority[i] = 1;
	}
	return priority;
}


Revisit::Revisit()
{
	IgnoreBlindSpot = false;
}

Revisit::~Revisit()
{

}

void Revisit::AskForBlindSpot()
{
//输入接口:得到盲区位置locs
	std::vector<LatLngPoint> locs;
	for(auto &loc:locs)
	{
		cv::Point cv_pt=transfer_.LngAndlat2CV(loc);
		mine_pos.push_back(cv_pt);
	}
}

void Revisit::FindRevisitOrder()
{
	//弃用
}

void Revisit::FindRevisitPath()
{
	// for(auto index:VehicleID)
	// 	std::cout<<index<<' ';
	// std::cout<<std::endl;
	// std::cout << "Revisit started" << std::endl;
	// std::cout << "left_time: " << g_BlackBoardAgent->LeftTime << std::endl;
	cv::Mat3b	_vis_map;
	std::vector<std::vector<double>> for_revisit;
	// std::cout << "poses_now_size: " << module.poses_now.size() << std::endl;
	for (int i = 0; i < module.poses_now.size(); i++)
	{
		std::vector<double> veh_start_pt, veh_end_pt;
		if (module.vehicle_cover_finished_for_revisit[i])
		{
			if (module.vehicle_revisit_finished_for_end[i])
			{
				vehicle_start_pt = g_BlackBoardAgent->end_pt;
				// std::cout << "Vehicle [" << i << "] finished! " << std::endl;
				// std::cout << "FINISHED! " << std::endl;
			}
			else
			{
				vehicle_start_pt =  transfer_.CV2LngAndlat(module.real_poses[i]);
			}
			veh_start_pt.push_back(vehicle_start_pt.lng);
			veh_start_pt.push_back(vehicle_start_pt.lat);
			for_revisit.push_back(veh_start_pt);
			veh_end_pt.push_back(g_BlackBoardAgent->end_pts[i].lng);
			veh_end_pt.push_back(g_BlackBoardAgent->end_pts[i].lat);
			for_revisit.push_back(veh_end_pt);
			car_num++;
		}
		else
		{
			veh_start_pt.push_back(transfer_.CV2LngAndlat(module.all_cover_paths[i][module.all_cover_paths[i].size() - 1]).lng);
			veh_start_pt.push_back(transfer_.CV2LngAndlat(module.all_cover_paths[i][module.all_cover_paths[i].size() - 1]).lat);
			for_revisit.push_back(veh_start_pt);
			veh_end_pt.push_back(g_BlackBoardAgent->end_pts[i].lng);
			veh_end_pt.push_back(g_BlackBoardAgent->end_pts[i].lat);
			for_revisit.push_back(veh_end_pt);
		}
	}
	// 可用于重访的车的数量
	// std::cout << "car_num: " << for_revisit.size() << std::endl;
	for (int m = 0; m < mine_pos.size(); m++)
    {
        // for_revisit.push_back(mine_pos[m].x);
		// for_revisit.push_back(mine_pos[m].y);
		std::vector<double> revisit_pt;
		LatLngPoint mine_pos_jwd;
		mine_pos_jwd = transfer_.CV2LngAndlat(mine_pos[m]);
		revisit_pt.push_back(mine_pos_jwd.lng);
		revisit_pt.push_back(mine_pos_jwd.lat);
		for_revisit.push_back(revisit_pt);
    }
	// std::cout << "total_num: " << for_revisit.size() << std::endl;
	
	// std::cout << "Capacity: ";
	// for (int i = 0; i < g_BlackBoardAgent->capacity.size(); i++) std::cout << g_BlackBoardAgent->capacity[i] << ", ";
	// std::cout << std::endl;

	// 增加重访停止逻辑
	bool revisit_finished = false;
	for (int i = 0; i < module.vehicle_revisit_finished_for_end.size(); i++)
	{
		if (!module.vehicle_revisit_finished_for_end[i]) break;
		if (i == module.vehicle_revisit_finished_for_end.size() - 1) 
		{
			revisit_finished = true;
			// std::cout << "ALL Finished! " << std::endl;
			g_RevisitAgent->VehicleID.clear();
			g_CoverageAgent->VehicleID.clear();
		}

	}

	for (int i = 0; i < g_BlackBoardAgent->BlindAreas.size(); i++)
	{
		std::vector<double> blind_area_temp;
		blind_area_temp.push_back(g_BlackBoardAgent->BlindAreas[i].lng);
		blind_area_temp.push_back(g_BlackBoardAgent->BlindAreas[i].lat);
		for_revisit.push_back(blind_area_temp);
	}

	std::vector<int> priority_temp;
	int all_capacity = 0;
	for (int i = 0; i < g_BlackBoardAgent->capacity.size(); i++) all_capacity += g_BlackBoardAgent->capacity[i];
	Solution solution(g_RevisitAgent->mine_pos, all_capacity, !g_BlackBoardAgent->new_scene, g_BlackBoardAgent->CVArea);
	priority_temp = solution.find_channel_pos();
	g_BlackBoardAgent->Priority.assign(priority_temp.begin(),priority_temp.end());

	cout << "Priorty size: " << g_BlackBoardAgent->Priority.size() << endl;
	cout << "Mines size: " << g_RevisitAgent->mine_pos.size() << endl;
	
	if ((car_num > 0) || revisit_finished)
		revisit_trajs=module.Module_Solve(for_revisit, g_BlackBoardAgent->speed, _vis_map, module.rest_time_in_cover, g_BlackBoardAgent->LeftTime,\
																					g_BlackBoardAgent->VehiclesNum, g_BlackBoardAgent->capacity, g_BlackBoardAgent->WorkTime, g_BlackBoardAgent->Delivery, \
																					g_BlackBoardAgent->Priority,  g_BlackBoardAgent->Skill, g_BlackBoardAgent->area_max_lat, g_BlackBoardAgent->area_min_lat);
	
	car_num = 0;
}

void Revisit::FindRevisitPath_NewScene()
{
	// for(auto index:VehicleID)
	// 	std::cout<<index<<' ';
	// std::cout<<std::endl;
	// std::cout << "Revisit started" << std::endl;
	// std::cout << "left_time: " << g_BlackBoardAgent->LeftTime << std::endl;
	cv::Mat3b	_vis_map;
	std::vector<std::vector<double>> for_revisit;
	// std::cout << "poses_now_size: " << module.poses_now.size() << std::endl;
	for (int i = 0; i < module.poses_now.size(); i++)
	{
		std::vector<double> veh_start_pt, veh_end_pt;
		if (module.vehicle_cover_finished_for_revisit[i])
		{
			if (module.vehicle_revisit_finished_for_end[i])
			{
				vehicle_start_pt = g_BlackBoardAgent->end_pt;
				// std::cout << "Vehicle [" << i << "] finished! " << std::endl;
				// std::cout << "FINISHED! " << std::endl;
			}
			else
			{
				vehicle_start_pt =  transfer_.CV2LngAndlat(module.real_poses[i]);
			}
			veh_start_pt.push_back(vehicle_start_pt.lng);
			veh_start_pt.push_back(vehicle_start_pt.lat);
			for_revisit.push_back(veh_start_pt);
			veh_end_pt.push_back(g_BlackBoardAgent->end_pts[i].lng);
			veh_end_pt.push_back(g_BlackBoardAgent->end_pts[i].lat);
			for_revisit.push_back(veh_end_pt);
			car_num++;
		}
		else
		{
			veh_start_pt.push_back(transfer_.CV2LngAndlat(module.all_cover_paths[i][module.all_cover_paths[i].size() - 1]).lng);
			veh_start_pt.push_back(transfer_.CV2LngAndlat(module.all_cover_paths[i][module.all_cover_paths[i].size() - 1]).lat);
			for_revisit.push_back(veh_start_pt);
			veh_end_pt.push_back(g_BlackBoardAgent->end_pts[i].lng);
			veh_end_pt.push_back(g_BlackBoardAgent->end_pts[i].lat);
			for_revisit.push_back(veh_end_pt);
		}
	}
	// 可用于重访的车的数量
	// std::cout << "car_num: " << for_revisit.size() << std::endl;
	for (int m = 0; m < mine_pos_new_scene.size(); m++)
    {
        // for_revisit.push_back(mine_pos[m].x);
		// for_revisit.push_back(mine_pos[m].y);
		std::vector<double> revisit_pt;
		LatLngPoint mine_pos_jwd;
		mine_pos_jwd = transfer_.CV2LngAndlat(mine_pos_new_scene[m]);
		revisit_pt.push_back(mine_pos_jwd.lng);
		revisit_pt.push_back(mine_pos_jwd.lat);
		for_revisit.push_back(revisit_pt);
    }
	// std::cout << "total_num: " << for_revisit.size() << std::endl;
	
	// std::cout << "Capacity: ";
	// for (int i = 0; i < g_BlackBoardAgent->capacity.size(); i++) std::cout << g_BlackBoardAgent->capacity[i] << ", ";
	// std::cout << std::endl;

	// 增加重访停止逻辑
	bool revisit_finished = false;
	for (int i = 0; i < module.vehicle_revisit_finished_for_end.size(); i++)
	{
		if (!module.vehicle_revisit_finished_for_end[i]) break;
		if (i == module.vehicle_revisit_finished_for_end.size() - 1) 
		{
			revisit_finished = true;
			// std::cout << "ALL Finished! " << std::endl;
			g_RevisitAgent->VehicleID.clear();
			g_CoverageAgent->VehicleID.clear();
		}

	}

	for (int i = 0; i < g_BlackBoardAgent->BlindAreas.size(); i++)
	{
		std::vector<double> blind_area_temp;
		blind_area_temp.push_back(g_BlackBoardAgent->BlindAreas[i].lng);
		blind_area_temp.push_back(g_BlackBoardAgent->BlindAreas[i].lat);
		for_revisit.push_back(blind_area_temp);
	}
	
	
	if ((car_num > 0) || revisit_finished)
		revisit_trajs=module.Module_Solve_New_Scene(for_revisit, g_BlackBoardAgent->speed, _vis_map, module.rest_time_in_cover, g_BlackBoardAgent->LeftTime, \
																					g_BlackBoardAgent->VehiclesNum, g_BlackBoardAgent->capacity, g_BlackBoardAgent->WorkTime_NewScene, g_BlackBoardAgent->Delivery_NewScene, \
																					g_BlackBoardAgent->Priority_NewScene,  g_BlackBoardAgent->Skill, g_BlackBoardAgent->area_max_lat, g_BlackBoardAgent->area_min_lat);
	
	car_num = 0;
}

