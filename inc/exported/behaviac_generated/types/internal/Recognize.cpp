
#include "Recognize.h"

Recognize::Recognize()
{
	AmmunitionAvaiable = false;
	CanBeRecognized = false;
	NeedEliminated = false;
	eliminate_start_time = std::vector<clock_t>(100,-1);
	eliminate_start_time_new_scene = std::vector<clock_t>(100,-1);
}

Recognize::~Recognize()
{

}

void Recognize::AddToEliminate()
{
	//暂未区分两者
	AddToRevisit();
}

void Recognize::AddToRevisit()
{
	// TODO: target_location 
	// LatLngPoint target_location = g_BlackBoardAgent->mines[1];
	auto cv_pt=transfer_.LngAndlat2CV(target_location);

	bool In_Group = false; // 雷是否聚集的标志位
	for (int i = 0; i < g_RevisitAgent->mine_pos.size(); i++)
	{
		// std::cout << "Distance: " << sqrt(pow(cv_pt.x - g_RevisitAgent->mine_pos[i].x, 2)+pow(cv_pt.y - g_RevisitAgent->mine_pos[i].y, 2)) << std::endl;
		// 判断雷是否出现聚集
		if (sqrt(pow(cv_pt.x - g_RevisitAgent->mine_pos[i].x, 2)+pow(cv_pt.y - g_RevisitAgent->mine_pos[i].y, 2)) < 7)
		{
			// g_BlackBoardAgent->Delivery[i]++;
			g_BlackBoardAgent->WorkTime[i] = 200;
			g_RevisitAgent->mine_pos.push_back(cv_pt);
			g_BlackBoardAgent->WorkTime.push_back(200);
			In_Group = true;

			// g_BlackBoardAgent->simulation_mines.insert(g_BlackBoardAgent->simulation_mines.begin() + i + 1, cv_pt);
			// g_BlackBoardAgent->mines_type.insert(g_BlackBoardAgent->mines_type.begin() + i + 1, MineType::DETECTED);
			
			// // 增加水雷群索引
			// if (g_BlackBoardAgent->Mine_Group_Index.size() == 0) g_BlackBoardAgent->Mine_Group_Index.push_back(i);
			// // 防止出现重复
			// for (int j = 0; j < g_BlackBoardAgent->Mine_Group_Index.size(); j++)
			// {
			// 	if (i == g_BlackBoardAgent->Mine_Group_Index[j]) break;
			// 	if (j == g_BlackBoardAgent->Mine_Group_Index.size()) g_BlackBoardAgent->Mine_Group_Index.push_back(i);
			// }
			break;
		}
	}
	if (!In_Group)	// 没有聚集，则添加新雷
	{
		g_RevisitAgent->mine_pos.push_back(cv_pt);
		g_BlackBoardAgent->WorkTime.push_back(100);
	}
	g_BlackBoardAgent->Delivery.push_back(1);
	g_BlackBoardAgent->Priority.push_back(1);
	g_BlackBoardAgent->simulation_mines.push_back(cv_pt);
	g_BlackBoardAgent->mines_type.push_back(MineType::DETECTED);

	// 针对带基数的新场景修改
	std::vector<int> Skill_temp = {0,1,2,3};
	g_BlackBoardAgent->Skill.push_back(Skill_temp);
	std::vector<clock_t> BanTime_temp = {0,0,0,0};
	g_BlackBoardAgent->RevisitBanTime.push_back(BanTime_temp);

	std::cout << "WorkTime size: " << g_BlackBoardAgent->WorkTime.size() << std::endl;

	// 同步更新已探查到的水雷


	// 给定新数组存放雷的索引，与水雷同步更新
	if (g_BlackBoardAgent->Mine_Index_ID.size() == 0) g_BlackBoardAgent->Mine_Index_ID.push_back(0);
	else g_BlackBoardAgent->Mine_Index_ID.push_back(g_BlackBoardAgent->Mine_Index_ID[g_BlackBoardAgent->Mine_Index_ID.size() - 1] + 1);


}

void Recognize::AddRevisitForNewScene()
{
	auto cv_pt=transfer_.LngAndlat2CV(target_location);

	bool In_Group = false; // 雷是否聚集的标志位
	for (int i = 0; i < g_RevisitAgent->mine_pos_new_scene.size(); i++)
	{
		// std::cout << "Distance: " << sqrt(pow(cv_pt.x - g_RevisitAgent->mine_pos[i].x, 2)+pow(cv_pt.y - g_RevisitAgent->mine_pos[i].y, 2)) << std::endl;
		// 判断雷是否出现聚集
		if (sqrt(pow(cv_pt.x - g_RevisitAgent->mine_pos_new_scene[i].x, 2)+pow(cv_pt.y - g_RevisitAgent->mine_pos_new_scene[i].y, 2)) < 7)
		{
			// g_BlackBoardAgent->Delivery[i]++;
			g_BlackBoardAgent->WorkTime_NewScene[i] = 200;
			g_RevisitAgent->mine_pos_new_scene.push_back(cv_pt);
			g_BlackBoardAgent->WorkTime_NewScene.push_back(200);
			In_Group = true;

			// g_BlackBoardAgent->simulation_mines.insert(g_BlackBoardAgent->simulation_mines.begin() + i + 1, cv_pt);
			// g_BlackBoardAgent->mines_type.insert(g_BlackBoardAgent->mines_type.begin() + i + 1, MineType::DETECTED);
			
			// // 增加水雷群索引
			// if (g_BlackBoardAgent->Mine_Group_Index.size() == 0) g_BlackBoardAgent->Mine_Group_Index.push_back(i);
			// // 防止出现重复
			// for (int j = 0; j < g_BlackBoardAgent->Mine_Group_Index.size(); j++)
			// {
			// 	if (i == g_BlackBoardAgent->Mine_Group_Index[j]) break;
			// 	if (j == g_BlackBoardAgent->Mine_Group_Index.size()) g_BlackBoardAgent->Mine_Group_Index.push_back(i);
			// }
			break;
		}
	}
	if (!In_Group)	// 没有聚集，则添加新雷
	{
		g_RevisitAgent->mine_pos_new_scene.push_back(cv_pt);
		g_BlackBoardAgent->WorkTime_NewScene.push_back(100);
	}
	g_BlackBoardAgent->Delivery_NewScene.push_back(1);
	g_BlackBoardAgent->Priority_NewScene.push_back(1);
	g_BlackBoardAgent->simulation_mines_new_scene.push_back(cv_pt);
	g_BlackBoardAgent->mines_type_new_scene.push_back(MineType::DETECTED);

	// 针对带基数的新场景修改
	//TODO: 添加雷的两种阈值
	// g_BlackBoardAgent->MinesNewSceneNum.push_back();
	std::vector<int> Skill_temp = {0,1,2,3};
	g_BlackBoardAgent->Skill.push_back(Skill_temp);
	std::vector<clock_t> BanTime_temp = {0,0,0,0};
	g_BlackBoardAgent->RevisitBanTime.push_back(BanTime_temp);

	std::cout << "WorkTime size: " << g_BlackBoardAgent->WorkTime_NewScene.size() << std::endl;

	// 同步更新已探查到的水雷


	// 给定新数组存放雷的索引，与水雷同步更新
	if (g_BlackBoardAgent->Mine_Index_ID_new_scene.size() == 0) g_BlackBoardAgent->Mine_Index_ID_new_scene.push_back(0);
	else g_BlackBoardAgent->Mine_Index_ID_new_scene.push_back(g_BlackBoardAgent->Mine_Index_ID_new_scene[g_BlackBoardAgent->Mine_Index_ID_new_scene.size() - 1] + 1);
}

void Recognize::ApproachAndRecognize()
{
	//局部雷位置，没有需要考虑的则置空
	std::vector<cv::Point> local_mine_pos;
	//局部不可通行位置，没有需要考虑的则置空
	std::vector<cv::Point> forbidden_area_pos;
	//TODO target_location:雷的位置
	auto cv_start_pt=transfer_.LngAndlat2CV(g_BlackBoardAgent->current_pt[car_index]);
	auto cv_target_pt=transfer_.LngAndlat2CV(target_location);
	std::vector<cv::Point2f> traj=module.go_to_target(cv_start_pt,cv_target_pt,local_mine_pos, forbidden_area_pos);
}

void Recognize::Eliminate()
{
	// static bool get_mine_id = false;
	// if(!get_mine_id)
	// {
	// 	for (int k = 0; k < g_RevisitAgent->mine_pos.size(); k++)
	// 	{
	// 		g_BlackBoardAgent->Mine_Index_ID.push_back(k);			
	// 	}
	// 	get_mine_id = true;
	// 	std::cout << "++g_BlackBoardAgent->Mine_Index_ID: " << g_BlackBoardAgent->Mine_Index_ID.size() << std::endl;
	// }

	int MineIndex_temp;
	//NOTE 1.发送清除信号
    if (g_BlackBoardAgent->capacity[Eliminate_CarIndex] > 0)
	{
		// std::cout<<"eliminate_mine_index:"<<Eliminate_MineIndex<<std::endl;
		g_BlackBoardAgent->Delivery[Eliminate_MineIndex]--;
		g_BlackBoardAgent->capacity[Eliminate_CarIndex]--;
		if (g_BlackBoardAgent->Delivery[Eliminate_MineIndex] == 0)
		{
    		std::cout<<"eliminate_mine_location:(lat) "<<target_location.lat<<",(lng) "<<target_location.lng<<std::endl;
    		// std::cout<<"send_eliminate_cmd"<<std::endl;
    		// SendEliminateCommand();
		
    		g_RevisitAgent->mine_pos.erase(g_RevisitAgent->mine_pos.begin()+Eliminate_MineIndex);
			// std::cout<<"-----------------------------------------------"<<std::endl;
		
			// 同步更新雷的作业时间，雷群数量，优先级向量
			g_BlackBoardAgent->WorkTime.erase(g_BlackBoardAgent->WorkTime.begin() + Eliminate_MineIndex);
			g_BlackBoardAgent->Delivery.erase(g_BlackBoardAgent->Delivery.begin() + Eliminate_MineIndex);
			g_BlackBoardAgent->Priority.erase(g_BlackBoardAgent->Priority.begin() + Eliminate_MineIndex);
		}
		// 同步更新数组索引
		MineIndex_temp = g_BlackBoardAgent->Mine_Index_ID[Eliminate_MineIndex];
		// std::cout << "Eliminate_MineIndex: " << g_BlackBoardAgent->Mine_Index_ID[Eliminate_MineIndex] << std::endl;
		g_BlackBoardAgent->Mine_Index_ID.erase(g_BlackBoardAgent->Mine_Index_ID.begin() + Eliminate_MineIndex);
		// std::cout<<"+++++++++++++++++++++++++++++"<<std::endl;
		// NOTE 2. 给出清除雷开始时间
		clock_t start = clock();
		// std::cout << "MineIndex_temp: " << MineIndex_temp << std::endl;
		eliminate_start_time[MineIndex_temp] = start;
	}
	std::cout << "Eliminate Finished! " << std::endl;
}

void Recognize::Eliminate_NewScene()
{
	// static bool get_mine_id = false;
	// if(!get_mine_id)
	// {
	// 	for (int k = 0; k < g_RevisitAgent->mine_pos.size(); k++)
	// 	{
	// 		g_BlackBoardAgent->Mine_Index_ID.push_back(k);			
	// 	}
	// 	get_mine_id = true;
	// 	std::cout << "++g_BlackBoardAgent->Mine_Index_ID: " << g_BlackBoardAgent->Mine_Index_ID.size() << std::endl;
	// }

	int MineIndex_temp;
	//NOTE 1.发送清除信号
    if (g_BlackBoardAgent->capacity[Eliminate_CarIndex] > 0)
	{
		// std::cout<<"eliminate_mine_index:"<<Eliminate_MineIndex<<std::endl;
		g_BlackBoardAgent->Delivery_NewScene[Eliminate_MineIndex]--;
		g_BlackBoardAgent->capacity[Eliminate_CarIndex]--;
		if (g_BlackBoardAgent->Delivery_NewScene[Eliminate_MineIndex] <= 0)
		{
    		std::cout<<"eliminate_mine_location:(lat) "<<target_location.lat<<",(lng) "<<target_location.lng<<std::endl;
    		// std::cout<<"send_eliminate_cmd"<<std::endl;
    		// SendEliminateCommand();
		
    		g_RevisitAgent->mine_pos_new_scene.erase(g_RevisitAgent->mine_pos_new_scene.begin()+Eliminate_MineIndex);
			// std::cout<<"-----------------------------------------------"<<std::endl;
		
			// 同步更新雷的作业时间，雷群数量，优先级向量
			g_BlackBoardAgent->WorkTime_NewScene.erase(g_BlackBoardAgent->WorkTime_NewScene.begin() + Eliminate_MineIndex);
			g_BlackBoardAgent->Delivery_NewScene.erase(g_BlackBoardAgent->Delivery_NewScene.begin() + Eliminate_MineIndex);
			g_BlackBoardAgent->Priority_NewScene.erase(g_BlackBoardAgent->Priority_NewScene.begin() + Eliminate_MineIndex);
			// 新场景下需要更新的向量
			g_BlackBoardAgent->MinesNewSceneNum.erase(g_BlackBoardAgent->MinesNewSceneNum.begin() + Eliminate_MineIndex);
			g_BlackBoardAgent->Skill.erase(g_BlackBoardAgent->Skill.begin()+Eliminate_MineIndex);
		}
		// 同步更新数组索引
		MineIndex_temp = g_BlackBoardAgent->Mine_Index_ID_new_scene[Eliminate_MineIndex];
		// std::cout << "Eliminate_MineIndex: " << g_BlackBoardAgent->Mine_Index_ID[Eliminate_MineIndex] << std::endl;
		g_BlackBoardAgent->Mine_Index_ID_new_scene.erase(g_BlackBoardAgent->Mine_Index_ID_new_scene.begin() + Eliminate_MineIndex);
		// std::cout<<"+++++++++++++++++++++++++++++"<<std::endl;
		// NOTE 2. 给出清除雷开始时间
		clock_t start = clock();
		// std::cout << "MineIndex_temp: " << MineIndex_temp << std::endl;
		eliminate_start_time_new_scene[MineIndex_temp] = start;
	}
	std::cout << "Eliminate Finished! " << std::endl;
}


void Recognize::DoRevisitAct()
{

}

void Recognize::DoRevisitAct_NewScene()
{

}