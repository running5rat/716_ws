#include "behaviac_generated/types/behaviac_types.h"


#include "iostream"
#include "time.h"
// #include "blackboard.h"
#include "Module.hpp"
#include "stdio.h"
#include "unistd.h"

/* 变量 */
double perception_distance = 300;	// 感知的范围
double clean_distance = 8;
double end_distance = 10;

//任务实现模块
Module module;
Transfer transfer_(0,0,0,0);
//接口函数
void Update_Msg();
bool dis_check(cv::Point current_pt, cv::Point mine, double distance);

behaviac::Agent* g_MakeTreeAgent=NULL;
blackboard* g_BlackBoardAgent=NULL;
Coverage* g_CoverageAgent=NULL;
Revisit* g_RevisitAgent=NULL;
Interact* g_InteractAgent=NULL;
Recognize* g_RecognizeAgent = NULL;
//初始化加载
bool InitBehavic()
{

	//NOTE 移植时需修改
	char* buffer = getcwd(NULL, 0); 
	std::string file_path = (string)buffer + "/../inc/exported";
	behaviac::Workspace::GetInstance()->SetFilePath(file_path.c_str());//行为树所在的目录

	behaviac::Workspace::GetInstance()->SetFileFormat(behaviac::Workspace::EFF_xml);//加载的行为树格式（xml）

	return true;
}
//初始化实例
bool InitPlayer()
{
	g_MakeTreeAgent = behaviac::Agent::Create<behaviac::Agent>();
	g_BlackBoardAgent = behaviac::Agent::Create<blackboard>();
	g_CoverageAgent=behaviac::Agent::Create<Coverage>();
	g_RevisitAgent=behaviac::Agent::Create<Revisit>();
	g_InteractAgent=behaviac::Agent::Create<Interact>();
	g_RecognizeAgent = behaviac::Agent::Create<Recognize>();
	// 行为树名称
	bool bRet = g_MakeTreeAgent->btload("Behavior");
	g_MakeTreeAgent->btsetcurrent("Behavior");

	return bRet;
}

void PlanLoop()
{
	// std::cout << "Plan Loop In! " << std::endl; 
	// if (module.going_to_start)
	// {
	// 	if( ( pow(module.real_poses[0].x-g_BlackBoardAgent->start_pts[0].x, 2)+pow(module.real_poses[0].y-g_BlackBoardAgent->start_pts[0].y, 2) > 1 )
	// 		|| ( pow(module.real_poses[1].x-g_BlackBoardAgent->start_pts[1].x, 2)+pow(module.real_poses[1].y-g_BlackBoardAgent->start_pts[1].y, 2) > 1 )
	// 		|| ( pow(module.real_poses[2].x-g_BlackBoardAgent->start_pts[2].x, 2)+pow(module.real_poses[2].y-g_BlackBoardAgent->start_pts[2].y, 2) > 1 )
	// 		|| ( pow(module.real_poses[3].x-g_BlackBoardAgent->start_pts[3].x, 2)+pow(module.real_poses[3].y-g_BlackBoardAgent->start_pts[3].y, 2) > 1 )
	// 	)
	// 	{
	// 		module.planloop_approach(g_BlackBoardAgent->start_pts[0], 0, module.received_local_mines_forAll[0], module.received_dangerous_radiuses[0], 90);
	// 		module.planloop_approach(g_BlackBoardAgent->start_pts[1], 1, module.received_local_mines_forAll[1], module.received_dangerous_radiuses[1], 90);
	// 		module.planloop_approach(g_BlackBoardAgent->start_pts[2], 2, module.received_local_mines_forAll[2], module.received_dangerous_radiuses[2], 90);
	// 		module.planloop_approach(g_BlackBoardAgent->start_pts[3], 3, module.received_local_mines_forAll[3], module.received_dangerous_radiuses[3], 90);
	// 	}
	// 	else
	// 	{
	// 		module.going_to_start = false;
	// 	}
	// }
	// else
	{
		for (int i = 0; i < g_BlackBoardAgent->VehiclesNum; i++)
		{
			module.received_local_mines_forAll[i].clear();
			module.received_dangerous_radiuses[i].clear();
			if (module.vehicle_cover_finished[i])
			{
				if (module.revisit_trajs_out[i].size()>0)
				{
					for (int j = 0; j < g_BlackBoardAgent->simulation_mines.size(); j++)
					{
						if ( pow(module.revisit_trajs_out[i][1].x - g_BlackBoardAgent->simulation_mines[j].x, 2)+pow(module.revisit_trajs_out[i][1].y - g_BlackBoardAgent->simulation_mines[j].y, 2) > 20*20 )
						{
							if (pow(g_BlackBoardAgent->simulation_mines[j].x - module.real_poses[i].x, 2)+pow(g_BlackBoardAgent->simulation_mines[j].y - module.real_poses[i].y, 2) < 300*300)
							{
								cv::Point2f mine_pos_real;
								mine_pos_real.x = g_BlackBoardAgent->simulation_mines[j].x;
								mine_pos_real.y = g_BlackBoardAgent->simulation_mines[j].y;
								module.received_local_mines_forAll[i].push_back(mine_pos_real);
								module.received_dangerous_radiuses[i].push_back(g_BlackBoardAgent->simulation_mines_radius[j]);
								// module.received_dangerous_radiuses[0].push_back(15);
							}
						}
					}
					
					for (int j = 0; j < g_BlackBoardAgent->VehiclesNum; j++)
					{
						if (j != i)
						{
							if ( pow(module.real_poses[j].x - module.real_poses[i].x, 2)+pow(module.real_poses[j].y - module.real_poses[i].y, 2)<200*200 )
							{
								module.received_local_mines_forAll[i].push_back(module.real_poses[j]);
								module.received_dangerous_radiuses[i].push_back(50);
							}
						}
					}
					
					cv::Point2f target_pos_real;
					target_pos_real.x = module.revisit_trajs_out[i][1].x;
					target_pos_real.y = module.revisit_trajs_out[i][1].y;
					// std::cout << "-------------------------------------------------------------------" << std::endl;
					// std::cout << "target pos:" << "[x]: " << target_pos_real.x <<", [y]: " << target_pos_real.y << std::endl;
					// std::cout << "vehicle ID: " << i << std::endl;

					// std::cout << "module.received_local_mines_forAll[i]: " << module.received_local_mines_forAll[i].size() << std::endl;
					module.planloop_approach(target_pos_real, i, module.received_local_mines_forAll[i], module.received_dangerous_radiuses[i], 90);
				}
			}
			else
			{
				if (module.Eliminate_Cars_In_Cover[i])
				{
					for (int j = 0; j < g_BlackBoardAgent->simulation_mines.size(); j++)
					{
						if ( pow(module.Eliminate_Mines_In_Cover[i].x - g_BlackBoardAgent->simulation_mines[j].x, 2)+pow(module.Eliminate_Mines_In_Cover[i].y - g_BlackBoardAgent->simulation_mines[j].y, 2) > 20*20 )
						{
							if (pow(g_BlackBoardAgent->simulation_mines[j].x - module.real_poses[i].x, 2)+pow(g_BlackBoardAgent->simulation_mines[j].y - module.real_poses[i].y, 2) < 30*30)
							{
								cv::Point2f mine_pos_real;
								mine_pos_real.x = g_BlackBoardAgent->simulation_mines[j].x;
								mine_pos_real.y = g_BlackBoardAgent->simulation_mines[j].y;
								module.received_local_mines_forAll[i].push_back(mine_pos_real);
								module.received_dangerous_radiuses[i].push_back(g_BlackBoardAgent->simulation_mines_radius[j]);
								// module.received_dangerous_radiuses[0].push_back(15);
							}
						}
					}
					
					for (int j = 0; j < g_BlackBoardAgent->VehiclesNum; j++)
					{
						if (j != i)
						{
							if ( pow(module.real_poses[j].x - module.real_poses[i].x, 2)+pow(module.real_poses[j].y - module.real_poses[i].y, 2)<200*200 )
							{
								module.received_local_mines_forAll[i].push_back(module.real_poses[j]);
								module.received_dangerous_radiuses[i].push_back(50);
							}
						}
					}
					module.planloop_approach(module.Eliminate_Mines_In_Cover[i], i, module.received_local_mines_forAll[i], module.received_dangerous_radiuses[i], 90);
				}
				else
				{
					for (int j = 0; j < g_BlackBoardAgent->VehiclesNum; j++)
					{
						if (j != i)
						{
							if ( pow(module.real_poses[j].x - module.real_poses[i].x, 2)+pow(module.real_poses[j].y - module.real_poses[i].y, 2)<200*200 )
							{
								module.received_local_mines_forAll[i].push_back(module.real_poses[j]);
								module.received_dangerous_radiuses[i].push_back(50);
							}
						}
					}
					module.planloop_single_cover(module.all_cover_paths[i], i, module.received_local_mines_forAll[i], module.received_dangerous_radiuses[i], 90);
				}
			}
		}
		module.report_time_left();
	}
	module.visualize();
	// std::cout  << "module.target_pt_idx[0]: " << module.target_pt_idx[0] << std::endl;
	// std::cout << "Plan Loop End! " << std::endl; 
}

// 更新清理后的不可通行区域信息
void update_mines_state(){

	// 更新不可进入区域的清理时间
	for (int index=0; index<g_RecognizeAgent->eliminate_start_time.size(); index++ )
	{
		if(g_RecognizeAgent->eliminate_start_time[index] != -1)
		{
			clock_t current_t = clock();
			int delta_time = (current_t - g_RecognizeAgent->eliminate_start_time[index]) / CLOCKS_PER_SEC;
			if (delta_time < g_BlackBoardAgent->cleaning_time)
			{
				g_BlackBoardAgent->mines_type[index] = MineType::CLEANING;
				// std::cout << "Detla Time [" << index << "]: " << delta_time << std::endl;
			}
			else if (delta_time < g_BlackBoardAgent->cleaned_time)
			{
				g_BlackBoardAgent->mines_type[index] = MineType::CLEANED;
			}
			else
			{
				g_BlackBoardAgent->mines_type[index] = MineType::DONE;
			}
		}	
	}
	// std::cout << "Simulation Mines Size: " << g_BlackBoardAgent->simulation_mines.size()  << std::endl;
	g_BlackBoardAgent->simulation_mines_radius.resize(g_BlackBoardAgent->simulation_mines.size(), 30);
	for (int i = 0; i < g_BlackBoardAgent->simulation_mines.size(); i++)
	{
		switch (g_BlackBoardAgent->mines_type[i])
		{
		case DETECTED:
			g_BlackBoardAgent->simulation_mines_radius[i] = g_BlackBoardAgent->Detect_Radius;
			break;
		case CLEANING:
			g_BlackBoardAgent->simulation_mines_radius[i] = g_BlackBoardAgent->Clean_Radius;
			break;
		case CLEANED:
			g_BlackBoardAgent->simulation_mines_radius[i] = g_BlackBoardAgent->Eliminate_Radius;
			break;
		default:
			g_BlackBoardAgent->simulation_mines_radius[i] = 0;
			break;
		}
		// std::cout << "Simulation Mines [" << i << "] Redius: " << g_BlackBoardAgent->simulation_mines_radius[i] << std::endl; 
	}

	// 重访后当前船的禁区时间
	for(int i = 0; i < g_BlackBoardAgent->RevisitBanTime.size(); i++)
	{
		for (int j = 0; j < g_BlackBoardAgent->RevisitBanTime[i].size(); j++)
		{
			if(g_BlackBoardAgent->RevisitBanTime[i][j] != 0)
			{
				clock_t current_time =clock();
				if ((current_time - g_BlackBoardAgent->RevisitBanTime[i][j]) / CLOCKS_PER_SEC > 5)
				{
					g_BlackBoardAgent->RevisitBanTime[i][j] = 0;
					g_BlackBoardAgent->Skill[i].push_back(j);
				}
			}
		}
	}
}

// 新场景下，更新清理后的不可通行区域信息与重访后当前船的禁区时间
void update_mines_state_scene2(){

	// 更新不可进入区域的清理时间
	for (int index=0; index<g_RecognizeAgent->eliminate_start_time_new_scene.size(); index++ )
	{
		if(g_RecognizeAgent->eliminate_start_time_new_scene[index] != -1)
		{
			clock_t current_t = clock();
			int delta_time = (current_t - g_RecognizeAgent->eliminate_start_time_new_scene[index]) / CLOCKS_PER_SEC;
			if (delta_time < g_BlackBoardAgent->cleaning_time)
			{
				g_BlackBoardAgent->mines_type_new_scene[index] = MineType::CLEANING;
				// std::cout << "Detla Time [" << index << "]: " << delta_time << std::endl;
			}
			else if (delta_time < g_BlackBoardAgent->cleaned_time)
			{
				g_BlackBoardAgent->mines_type_new_scene[index] = MineType::CLEANED;
			}
			else
			{
				g_BlackBoardAgent->mines_type_new_scene[index] = MineType::DONE;
			}
		}	
	}
	// std::cout << "Simulation Mines Size: " << g_BlackBoardAgent->simulation_mines.size()  << std::endl;
	g_BlackBoardAgent->simulation_mines_radius_new_scene.resize(g_BlackBoardAgent->simulation_mines_new_scene.size(), 30);
	for (int i = 0; i < g_BlackBoardAgent->simulation_mines_new_scene.size(); i++)
	{
		switch (g_BlackBoardAgent->mines_type_new_scene[i])
		{
		case DETECTED:
			g_BlackBoardAgent->simulation_mines_radius_new_scene[i] = g_BlackBoardAgent->Detect_Radius;
			break;
		case CLEANING:
			g_BlackBoardAgent->simulation_mines_radius_new_scene[i] = g_BlackBoardAgent->Clean_Radius;
			break;
		case CLEANED:
			g_BlackBoardAgent->simulation_mines_radius_new_scene[i] = g_BlackBoardAgent->Eliminate_Radius;
			break;
		default:
			g_BlackBoardAgent->simulation_mines_radius_new_scene[i] = 0;
			break;
		}
		// std::cout << "Simulation Mines [" << i << "] Redius: " << g_BlackBoardAgent->simulation_mines_radius[i] << std::endl; 
	}

	// 重访后当前船的禁区时间
	for(int i = 0; i < g_BlackBoardAgent->RevisitBanTime.size(); i++)
	{
		for (int j = 0; j < g_BlackBoardAgent->RevisitBanTime[i].size(); j++)
		{
			if(g_BlackBoardAgent->RevisitBanTime[i][j] != 0)
			{
				clock_t current_time =clock();
				if ((current_time - g_BlackBoardAgent->RevisitBanTime[i][j]) / CLOCKS_PER_SEC > 5)
				{
					g_BlackBoardAgent->RevisitBanTime[i][j] = 0;
					g_BlackBoardAgent->Skill[i].push_back(j);
				}
			}
		}
	}
}

// 判断覆盖阶段发现的雷是否需要清理
void If_Need_Eliminate( cv::Point2f target_point)
{
	std::vector<cv::Point2f> work_area; 
	// 工作区域的四个顶点，分别为左上，左下，右上，右下
	LatLngPoint left_up_point, left_down_point, right_up_point, right_down_point;
		
	left_up_point.lat = g_BlackBoardAgent->area_min_lat;
	left_up_point.lng = g_BlackBoardAgent->area_min_lng;
	work_area.push_back(transfer_.LngAndlat2CV(left_up_point));
			
	left_down_point.lat = g_BlackBoardAgent->area_max_lat;
	left_down_point.lng = g_BlackBoardAgent->area_min_lng;
	work_area.push_back(transfer_.LngAndlat2CV(left_down_point));
			
	right_up_point.lat = g_BlackBoardAgent->area_min_lat;
	right_up_point.lng = g_BlackBoardAgent->area_max_lng;
	work_area.push_back(transfer_.LngAndlat2CV(right_up_point));
			
	right_down_point.lat = g_BlackBoardAgent->area_max_lat;
	right_down_point.lng = g_BlackBoardAgent->area_max_lng;
	work_area.push_back(transfer_.LngAndlat2CV(right_down_point));

	std::vector<cv::Point2f> other_mines_pos;
	for (int ii = 0; ii < g_RevisitAgent->mine_pos.size(); ii++)
	{
		cv::Point2f point_temp(g_RevisitAgent->mine_pos[ii].x, g_RevisitAgent->mine_pos[ii].y);
		if (dis_check(target_point, g_RevisitAgent->mine_pos[ii], 5)) continue;
		else other_mines_pos.push_back(point_temp);
	}
	if (module.MinesEliminateDecision(target_point, other_mines_pos, g_BlackBoardAgent->end_pt_cv, work_area, \
																			g_BlackBoardAgent->MyCost.contain, g_BlackBoardAgent->MyCost.distance2desternation, g_BlackBoardAgent->MyCost.distance2centerline, g_BlackBoardAgent->MyCost.thresthold))
	{
		int eliminate_car = module.VehicleEliminateDecision(target_point, module.real_poses, module.vehicle_heading, \
																														g_BlackBoardAgent->capacity, g_BlackBoardAgent->MyCost.capacity, g_BlackBoardAgent->MyCost.distance2vehicle, g_BlackBoardAgent->MyCost.heading);
					
		// std::cout << "Eliminate Car: " << eliminate_car << std::endl;
		// 补充和局规的接口
		if (g_BlackBoardAgent->capacity[eliminate_car] > 0)
		{
			module.Eliminate_Mines_In_Cover[eliminate_car] =  target_point;
			module.Eliminate_Cars_In_Cover[eliminate_car] = true;
		}
	}
	else
	{
		g_RevisitAgent->Replan = true;
	}
}


bool dis_check(cv::Point current_pt, cv::Point mine, double distance)
{
	double real_dis = sqrt(pow(current_pt.x - mine.x, 2)+pow(current_pt.y - mine.y, 2));
	// if  (distance == clean_distance) std::cout << "real dis: " << real_dis << std::endl;
	if (real_dis < distance) return true;
	else return false;	
}

void simulation_port(){
	//simution_finnish
	// std::cout << "simulation activated!" << std::endl;
	// std::cout << "Cover path num: "<< module.all_cover_paths.size() << std::endl;
	std::cout << "simulation-------------------------------in" << std::endl;
	for(int i = 0; i < g_BlackBoardAgent->VehiclesNum; i++)
	{
		// std::cout << "insideforloop" << std::endl;
		// std::cout <<  module.all_cover_paths[i].size() << std::endl;
		if (g_BlackBoardAgent->formation_flag)
		{
			// module.all_cover_paths[2] = module.revisit_trajs_out[1];
			// module.all_cover_paths[3] = module.revisit_trajs_out[3];

			module.vehicle_revisiting[2] = module.vehicle_revisiting[1];
			module.vehicle_revisiting[3] = module.vehicle_revisiting[1];
			
			module.vehicle_revisiting[1] = module.vehicle_revisiting[0];
		}
		
		// if(dis_check(module.real_poses[i], module.all_cover_paths[i][module.all_cover_paths[i].size() - 1]))
		if (module.vehicle_revisiting[i] && (!module.vehicle_cover_finished_for_revisit[i]))
		{
			module.vehicle_cover_finished_for_revisit[i] =true;
			g_BlackBoardAgent->msg_type = Msg_Type::SomeMissionFinnished;
			g_BlackBoardAgent->FinnishedIndex = i;
			
			// std::cout << "Become Revisit!" << std::endl;		
		}
		if (g_BlackBoardAgent->formation_flag)
		{
			module.vehicle_revisiting[1] = module.vehicle_revisiting[2];
		}

	}
	// std::cout << "-------------------------------------------------------------01" << std::endl;
	//simulation_perception

	// static bool first_time_mines = true;
	// if (first_time_mines)
	// {
	// 	for (int i = 0; i < module.gloable_mines_real.size(); i++)
	// 	{
					
	// 		g_BlackBoardAgent->NewTarget = true;
	// 		g_RecognizeAgent->target_location = transfer_.CV2LngAndlat(module.gloable_mines_real[i]);
	// 		g_RecognizeAgent->car_index = i;
	// 		g_InteractAgent->InteractTask = TaskType::SetTargetType;
	// 		//赋值
	// 		g_RecognizeAgent->NeedEliminated = true;
	// 		g_RecognizeAgent->CanBeRecognized = false;
	// 		// 障碍坐标登记
	// 		g_InteractAgent->SetTargetType();
	// 		// If_Need_Eliminate(module.gloable_mines_real[j]);
	// 		g_RevisitAgent->Replan = true;
	// 	}
	// 	first_time_mines = false;
	// }
	


	// 增加重复障碍判断逻辑
	// std::cout << "middle" << std::endl;
	for (int i = 0; i < module.real_poses.size(); i++)
	{
		for (int j = 0; j < module.gloable_mines_real.size(); j++)
		{
			if (dis_check(module.real_poses[i],  module.gloable_mines_real[j], perception_distance))
			{
				// 若为重复障碍则跳过
				if (module.mines_check[j] ) continue;
				else module.mines_check[j] = true;
				
				g_BlackBoardAgent->NewTarget = true;
				g_RecognizeAgent->target_location = transfer_.CV2LngAndlat(module.gloable_mines_real[j]);
				g_RecognizeAgent->car_index = i;

				//simulation_certain_type
				g_InteractAgent->InteractTask = TaskType::SetTargetType;
				//赋值
				g_RecognizeAgent->NeedEliminated = true;
				g_RecognizeAgent->CanBeRecognized = false;
				// 障碍坐标登记
				g_InteractAgent->SetTargetType();
				// If_Need_Eliminate(module.gloable_mines_real[j]);
				g_RevisitAgent->Replan = true;
			}
		}
		
		// 载弹量减少用于打击障碍
		// std::cout << "g_RevisitAgent->mine_pos.size(): " << g_RevisitAgent->mine_pos.size() << std::endl;
		for (int kk = 0; kk < g_RevisitAgent->mine_pos.size(); kk++)
		{
			if (module.vehicle_cover_finished_for_revisit[i])
			{	
				// std::cout << "inside----------------------------------------------------------00" << std::endl;
				if (dis_check(module.real_poses[i], g_RevisitAgent->mine_pos[kk], clean_distance))
				{
					bool with_others = false;
					for (int index = 0; index < module.real_poses.size(); index++)
					{
						if (index == i) continue;
						if (std::sqrt(pow(g_RevisitAgent->mine_pos[kk].x - module.real_poses[index].x, 2)
												+ pow(g_RevisitAgent->mine_pos[kk].y - module.real_poses[index].y, 2)) 
												< g_BlackBoardAgent->Clean_Radius)
						{
							with_others = true;
							break;
						}
					}	
					if (!with_others)
					{
						g_RecognizeAgent->Eliminate_MineIndex = kk;
						g_RecognizeAgent->Eliminate_CarIndex = i;

						g_BlackBoardAgent->RevisitBanTime[kk][i] = clock();
						for(int ski = 0; ski < g_BlackBoardAgent->Skill[kk].size(); ski++)
						{
							if (g_BlackBoardAgent->Skill[kk][ski] == i)
							{
								g_BlackBoardAgent->Skill[kk].erase(g_BlackBoardAgent->Skill[kk].begin() + ski);
							}
						}
						
						g_RecognizeAgent->Eliminate();
						g_RevisitAgent->FindRevisitPath();

						// if (!module.vehicle_cover_finished_for_revisit[i])
						// {
						// 	module.Eliminate_Cars_In_Cover[g_RecognizeAgent->Eliminate_CarIndex] = false;
						// }


						std::vector<double> safe_place_msg;
						safe_place_msg.resize(3, 0.0);
						module.Output_Safe_Place(safe_place_msg);

						// 清理完雷后实时上报安全距离
						std::cout << "---------------------------" << std::endl;
						std::cout << "min/max " << safe_place_msg[0] << ", " <<  safe_place_msg[1] << std::endl;
						std::cout << "SafeLength: " << safe_place_msg[2] << std::endl;
						// std::cout << "veh " << i << " In here! " << std::endl;
						g_BlackBoardAgent->safe_area.assign(safe_place_msg.begin(), safe_place_msg.end());
					}
					
				}
				// static int debug1 = 0;
				// std::cout << "outside-----------------------------------------------------00: " << debug1++ << std::endl;
			}
		}

		// 增加停止访问逻辑
		// cv::Point2f end_point(0, 0);

		if (dis_check(g_BlackBoardAgent->end_pt_cvs[i], module.real_poses[i], end_distance) 
			&& module.vehicle_cover_finished_for_revisit[i] && (!module.vehicle_revisit_finished_for_end[i]))
		{
			std::cout << "veh " << i << " In here! " << std::endl;
			module.vehicle_revisit_finished_for_end[i] = true;
			g_RevisitAgent->FindRevisitPath();
			
		}	
		// std::cout << "-------------------------------------------------------------02" << std::endl;
	}


	// std::cout << "simulation-------------------------------out" << std::endl;
}

// 针对新场景的策略
void simulation_port_scene2(){
	// std::cout << "simulation-------------------------------in" << std::endl;
	// 判断船只是否完成覆盖进入重访
	for(int i = 0; i < g_BlackBoardAgent->VehiclesNum; i++)
	{
		// std::cout << "insideforloop" << std::endl;
		// std::cout <<  module.all_cover_paths[i].size() << std::endl;	
		if (module.vehicle_revisiting[i] && (!module.vehicle_cover_finished_for_revisit[i]))
		{
			module.vehicle_cover_finished_for_revisit[i] =true;
			g_BlackBoardAgent->msg_type = Msg_Type::SomeMissionFinnished;
			g_BlackBoardAgent->FinnishedIndex = i;
			
			// std::cout << "Become Revisit!" << std::endl;		
		}
	}
	// std::cout << "-------------------------------------------------------------01" << std::endl;
	//simulation_perception

	// 读取上一场景中的雷区
	static bool first_time_mines = true;
	if (first_time_mines)
	{
		for (int i = 0; i < module.gloable_mines_real.size(); i++)
		{
					
			g_BlackBoardAgent->NewTarget = true;
			g_RecognizeAgent->target_location = transfer_.CV2LngAndlat(module.gloable_mines_real[i]);
			g_RecognizeAgent->car_index = i;
			g_InteractAgent->InteractTask = TaskType::SetTargetType;
			//赋值
			g_RecognizeAgent->NeedEliminated = true;
			g_RecognizeAgent->CanBeRecognized = false;
			// 障碍坐标登记
			g_InteractAgent->SetTargetType();
			// If_Need_Eliminate(module.gloable_mines_real[j]);
			g_RevisitAgent->Replan = true;
		}
		first_time_mines = false;
	}
	


	// 这一部分为：探测到虚雷便增加入重访队列
	// TODO：添加虚雷变量 module.virtual_gloable_mines_real
	// std::cout << "middle" << std::endl;
	for (int i = 0; i < module.real_poses.size(); i++)
	{
		for (int j = 0; j < module.virtual_gloable_mines_real.size(); j++)
		{
			if (dis_check(module.real_poses[i],  module.virtual_gloable_mines_real[j], perception_distance))
			{
				// 若为重复障碍则跳过
				if (module.virtual_mines_check[j] ) continue;
				else module.virtual_mines_check[j] = true;
				
				g_BlackBoardAgent->NewTarget = true;
				g_RecognizeAgent->target_location = transfer_.CV2LngAndlat(module.virtual_gloable_mines_real[j]);
				g_RecognizeAgent->car_index = i;

				//simulation_certain_type
				g_InteractAgent->InteractTask = TaskType::SetTargetType;
				//赋值
				g_RecognizeAgent->NeedEliminated = true;
				g_RecognizeAgent->CanBeRecognized = false;
				// 障碍坐标登记
				g_InteractAgent->SetTargetType();
				// If_Need_Eliminate(module.gloable_mines_real[j]);
				g_RevisitAgent->FindRevisitPath_NewScene();
			}
		}
		
		// 执行重访时的相关判断
		// std::cout << "g_RevisitAgent->mine_pos.size(): " << g_RevisitAgent->mine_pos.size() << std::endl;
		for (int kk = 0; kk < g_RevisitAgent->mine_pos_new_scene.size(); kk++)
		{
			// 处在重访状态下
			if (module.vehicle_cover_finished_for_revisit[i])
			{	
				// std::cout << "inside----------------------------------------------------------00" << std::endl;
				// 静态变量记录上一次的艇和雷
				static cv::Point2f mine_pos_temp(9999,9999);
				static int vehicle_temp = 9999;
				// 进入雷威胁半径，执行重访操作
				if (dis_check(module.real_poses[i], g_RevisitAgent->mine_pos_new_scene[kk], clean_distance))
				{
					bool with_others = false;
					for (int index = 0; index < module.real_poses.size(); index++)
					{
						if (index == i) continue;
						if (std::sqrt(pow(g_RevisitAgent->mine_pos_new_scene[kk].x - module.real_poses[index].x, 2)
												+ pow(g_RevisitAgent->mine_pos_new_scene[kk].y - module.real_poses[index].y, 2)) 
												< g_BlackBoardAgent->Clean_Radius)
						{
							with_others = true;
							break;
						}
					}	
					if (!with_others)
					{
						// 此处为了避免对同一艘艇对同一颗雷进行重复操作
						if (dis_check(mine_pos_temp, g_BlackBoardAgent->mines[kk], 5) && vehicle_temp == i) continue;
						mine_pos_temp = g_RevisitAgent->mine_pos_new_scene[kk];
						vehicle_temp = i;

						// 如果重访基数未满，则进行重访操作 DoRevisitAct_NewScene
						if (g_BlackBoardAgent->MinesNewSceneNum[kk].second > 0)
						{
							g_RecognizeAgent->Eliminate_MineIndex = kk;
							g_RecognizeAgent->Eliminate_CarIndex = i;

							g_BlackBoardAgent->RevisitBanTime[kk][i] = clock();
							for(int ski = 0; ski < g_BlackBoardAgent->Skill[kk].size(); ski++)
							{
								if (g_BlackBoardAgent->Skill[kk][ski] == i)
								{
									g_BlackBoardAgent->Skill[kk].erase(g_BlackBoardAgent->Skill[kk].begin() + ski);
								}
							}
							g_RecognizeAgent->DoRevisitAct_NewScene();
							g_RevisitAgent->FindRevisitPath_NewScene();

							g_BlackBoardAgent->MinesNewSceneNum[kk].first -= g_BlackBoardAgent->VehiclesNewSceneNum[i].first;
							g_BlackBoardAgent->MinesNewSceneNum[kk].second -= g_BlackBoardAgent->VehiclesNewSceneNum[i].second;

						}
						// 如果重访基数已满
						else
						{
							g_RecognizeAgent->Eliminate_MineIndex = kk;
							g_RecognizeAgent->Eliminate_CarIndex = i;
							//重访阈值和查证阈值均满足，执行清除操作
							if (g_BlackBoardAgent->MinesNewSceneNum[kk].first <= 0)
							{
								g_RecognizeAgent->Eliminate_NewScene();
								g_RevisitAgent->FindRevisitPath_NewScene();

								std::vector<double> safe_place_msg;
								safe_place_msg.resize(3, 0.0);
								module.Output_Safe_Place(safe_place_msg);

								// 清理完雷后实时上报安全距离
								std::cout << "---------------------------" << std::endl;
								std::cout << "min/max " << safe_place_msg[0] << ", " <<  safe_place_msg[1] << std::endl;
								std::cout << "SafeLength: " << safe_place_msg[2] << std::endl;
								// std::cout << "veh " << i << " In here! " << std::endl;
								g_BlackBoardAgent->safe_area.assign(safe_place_msg.begin(), safe_place_msg.end());
							}
							// 否则，执行其他方案
							else
							{

							}
						}		
					}	
				}
				// std::cout << "outside-----------------------------------------------------00: " << debug1++ << std::endl;
			}
		}

		// 增加停止访问逻辑
		// cv::Point2f end_point(0, 0);

		if (dis_check(g_BlackBoardAgent->end_pt_cvs[i], module.real_poses[i], end_distance) 
			&& module.vehicle_cover_finished_for_revisit[i] && (!module.vehicle_revisit_finished_for_end[i]))
		{
			std::cout << "veh " << i << " In here! " << std::endl;
			module.vehicle_revisit_finished_for_end[i] = true;
			g_RevisitAgent->FindRevisitPath_NewScene();
			
		}	
		// std::cout << "-------------------------------------------------------------02" << std::endl;
	}


	// std::cout << "simulation-------------------------------out" << std::endl;
}


//执行行为树
void UpdateLoop()
{
	behaviac::EBTStatus status = behaviac::BT_RUNNING;

	clock_t behaviac_start = clock();	// 测试用，重新初始化
	while (status == behaviac::BT_RUNNING)
	{
		 status = g_MakeTreeAgent->btexec();//循环节点在没有直到成功时会返回BT_RUNNING，此时继续执行此语句，让程序返回原先位置。
		//NOTE 通信接口函数：此处读收到的信息，在循环行为树前处更新标志位。
		
		// static int debug2 = 0;
		// std::cout << "Debug  num: "<< debug2++ << std::endl;
		std::cout << "Revisit vehicle num: "<< g_RevisitAgent->VehicleID.size() << std::endl;

		Update_Msg();
		//延迟0.1s
		std::cout << "cover_begin!" << std::endl;
		
		// 局部规划函数调用
		PlanLoop();
		std::cout << "cover_end!" << std::endl;
		
		clock_t start=clock();
		clock_t end=clock();
		while(((double)(end-start)/CLOCKS_PER_SEC)<0.1)
		{
			end=clock();
		}
		static bool test_flag = false;
		// if (((double)(clock() - behaviac_start)/CLOCKS_PER_SEC) > 10 && !test_flag)
		// {
		// 	test_flag =true;
		// 	module.Reinit(g_BlackBoardAgent->VehiclesNum);
		// }
		// std::cout<<"--------------------------------Loop end------------------------------"<<std::endl;
		
		if(!g_BlackBoardAgent->new_scene)
		{
			update_mines_state();
			simulation_port();
		}
		else
		{
			update_mines_state_scene2();
			simulation_port_scene2();
		}
		// 补充盲区
		// g_BlackBoardAgent->BlindAreas = ;

		// cout<<"11111  "<<g_RevisitAgent->VehicleVectorEmpty()<<endl;
		// std::cout<<"--------------------------------all end------------------------------"<<std::endl;
	}

}



//销毁实例，释放工作区
static void CleanupPlayer()
{
	g_MakeTreeAgent = NULL;
}

static void CleanupBehaviac()
{
	behaviac::Workspace::GetInstance()->Cleanup();
}

void PubFinishMsg(){
	//发送 	g_BlackBoardAgent->vector<LatLngPoint> safe_area; //安全通道区域
	
}



clock_t mission_start_time;

int main(int argc, char** argv)
{
	BEHAVIAC_UNUSED_VAR(argc);
	BEHAVIAC_UNUSED_VAR(argv);
	//init
	InitBehavic();
	bool ret=InitPlayer();
	// cout<<"ret"<<ret<<endl;
	

	// std::cout<<"InitPlayer"<<ret<<std::endl;
	
	std::cout<<"Behavior Tree Initialized!"<<std::endl;


	//NOTE 第一次执行前的信息传递，如区域大小，车辆参数等信息传递接口
	// InitMission();
	//Loop		
	//任务开始计时
	mission_start_time=clock();
	UpdateLoop();
	PubFinishMsg();

	CleanupPlayer();

	CleanupBehaviac();
	
	return 0;
}

//更新接口:
void Update_Msg(){
	//人工命令输入与输入的任务结束信号更新
	//接受类型

	switch (g_BlackBoardAgent->msg_type)
	{
		case EnableDisable:
		{
			//NOTE 输入使能或失能、使能的艇或失能的艇
			bool EnableFlag;
			int vehicle_index;

			//如果使能艇
			if(EnableFlag)
			{
				//加入覆盖任务
				g_CoverageAgent->VehicleID.push_back(vehicle_index);
				g_CoverageAgent->Replan=true;
			}
			//如果失能艇
			else
			{
				//读当前任务，更新当前任务状态，更新重规划标志位
				if(g_BlackBoardAgent->StateVector[vehicle_index]==g_BlackBoardAgent->CoverFlag)
				{
					//删除覆盖任务中的此艇
					for(int i=0;i<g_CoverageAgent->VehicleID.size();i++)
					{
						if(g_CoverageAgent->VehicleID[i]==vehicle_index)
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
						if(g_RevisitAgent->VehicleID[i]==vehicle_index)
							g_RevisitAgent->VehicleID.erase(g_CoverageAgent->VehicleID.begin()+i);
					}
					//重访重规划
					g_RevisitAgent->Replan=true;
				}
			}
		}
		break;

		case ManualCommand:
			g_BlackBoardAgent->ManualCommand=true;
			//NOTE 输入命令类型
			//TODO如果输入是MoveTo命令
			{
				g_InteractAgent->InteractTask=TaskType::MoveTo;
				//移动车辆
				// g_InteractAgent->MoveVehicleIndex=;
				//移动到的经纬度点
				// g_InteractAgent->MoveToPt=;
			}
			//TODO 如果输入是确定对象类型任务
			{
				g_InteractAgent->InteractTask=TaskType::SetTargetType;

				//对象类型
				// g_InteractAgent->TargetType=;
				//对象索引
				// g_InteractAgent->TargetLoc=;
			}
			//TODO如果输入为返回起点任务
			{
				g_InteractAgent->InteractTask=TaskType::TurnBack;
			}
		break;
		case SomeMissionFinnished:
		{
			//完成智能体的索引

			//更新StateVector
			// std::cout << "SomeMissionFinished started" << std::endl;
			if(g_BlackBoardAgent->StateVector[g_BlackBoardAgent->FinnishedIndex]==g_BlackBoardAgent->CoverFlag)
			{
				// cout << "1111" << endl;
				//在执行覆盖任务的智能体中删除完成智能体
				for(int i=0;i<g_CoverageAgent->VehicleID.size();i++)
				{
					if(g_CoverageAgent->VehicleID[i]==g_BlackBoardAgent->FinnishedIndex)
						g_CoverageAgent->VehicleID.erase(g_CoverageAgent->VehicleID.begin()+i);
					// cout << "2222" << endl;
				}
				//在执行重访任务的智能体中添加完成智能体
				g_RevisitAgent->VehicleID.push_back(g_BlackBoardAgent->FinnishedIndex);
				g_RevisitAgent->Replan = true;
			}
			else if(g_BlackBoardAgent->StateVector[g_BlackBoardAgent->FinnishedIndex]==g_BlackBoardAgent->RevisitFlag)
			{
				//在执行重访任务的智能体中删除完成智能体
				for(int i=0;i<g_RevisitAgent->VehicleID.size();i++)
				{
					if(g_RevisitAgent->VehicleID[i]==g_BlackBoardAgent->FinnishedIndex)
						g_RevisitAgent->VehicleID.erase(g_CoverageAgent->VehicleID.begin()+i);
				}
			}
			//更新该车辆状态
			g_BlackBoardAgent->StateVector[g_BlackBoardAgent->FinnishedIndex]++;
			// std::cout << "SomeMissionFinished Completed! " << std::endl;
			g_BlackBoardAgent->msg_type = Msg_Type::NoMsg;
		}
		break;
			case New_Target:
		{
			//输入需要抵进搜索的位置
			// g_RecognizeAgent->target_location=;
			//1.目标存在VRP vector
			
			//2.转CV::Point赋recognize类target_location

		}
		break;
	}


	//2.位置信息更新
	// g_BlackBoardAgent->current_pt=;
	//3.剩余时间更新
		clock_t current_time=clock();
		// g_BlackBoardAgent->LeftTime= 4.0 * 60 *60*60 - 6.0 * 60 * (double)(current_time-mission_start_time)/CLOCKS_PER_SEC;
	g_BlackBoardAgent->LeftTime = 4.0 * 60 *60 - 5.0 * 60 * (double)(current_time-mission_start_time)/CLOCKS_PER_SEC;
	if (g_BlackBoardAgent->LeftTime <= 0) g_BlackBoardAgent->LeftTime = 100;

	//NOTE 需要更新的
	//1.人工命令输入与输入的任务结束信号

	//2.车辆位置信息

	//3.剩余时间

}

