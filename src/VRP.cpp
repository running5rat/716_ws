#include "VRP.hpp"

Problem::Problem(std::string FileIn, int VehNum)
{
    veh_num = VehNum;    // TODO: 区分场景待完成

    vroom::Amount vehicle_capacity(1);
    vroom::Amount job_pickup(1), job_delivery(1);
    vroom::Duration setup = 0;
    vehicle_capacity[0] = 0;
    job_delivery[0] = 0;
    
    Json::Reader reader;/*用于按照JSON数据格式进行解析*/
	Json::Value root;/*用于保存JSON类型的一段数据*/
	
	ifstream srcFile(FileIn, ios::binary);/*定义一个ifstream流对象，与文件.json进行关联*/
	if (!srcFile.is_open())
	{
		cout << "Fail to read " << endl;
		return;
	}
    if (reader.parse(srcFile, root))
    {
        Json::Value Duration = root["matrices"]["car"]["durations"];
        Json::Value Cost = root["matrices"]["car"]["costs"];
        vroom::Matrix<vroom::Duration> DurationMat_temp(Duration.size());
        vroom::Matrix<vroom::Cost> CostMat_temp(Cost.size());
        for (uint32_t i = 0; i < Duration.size(); i++)
        {
            for (uint32_t j = 0; j < Duration.size(); j++)
            {
                DurationMat_temp[i][j] = Duration[i][j].asUInt();
            }
        }

        for (uint32_t i = 0; i < Cost.size(); i++)
        {
            for (uint32_t j = 0; j < Cost.size(); j++)
            {
                CostMat_temp[i][j] = Cost[i][j].asUInt();
            }
        }
        
        for( uint32_t i = 0; i < root["vehicles"].size(); i++)
        {
            vroom::Id Id = root["vehicles"][i]["id"].asUInt();
            vroom::Location start_index = root["vehicles"][i]["start_index"].asUInt();
            vroom::Location end_index = root["vehicles"][i]["end_index"].asUInt();
            vroom::TimeWindow tw(root["vehicles"][i]["time_window"][0].asUInt(), 
                                                                root["vehicles"][i]["time_window"][1].asUInt());
            double speed_factor = root["vehicles"][i]["speed_factor"].asDouble();

            vehicle_capacity[0] = root["vehicles"][i]["capacity"][0].asUInt();

            vroom::Vehicle veh_temp(
                Id,
                start_index,
                end_index,
                "car", 
                vehicle_capacity,
                vroom::Skills(),
                tw,
                std::vector<vroom::Break>(),
                "",
                speed_factor,     //speed factor
                10,     //max_tasks; TODO: need to update
                std::vector<vroom::VehicleStep>()
            );
            Vehicles.push_back(veh_temp);
        }
        for (uint32_t j = 0; j < root["jobs"].size(); j++)
        {
            vroom::Id Id = root["jobs"][j]["id"].asUInt();
            uint32_t index(root["jobs"][j]["location_index"].asUInt());
            vroom::Duration service = root["jobs"][j]["service"].asUInt();
            vroom::Priority priority = root["jobs"][j]["priority"].asUInt();
            std::vector<vroom::TimeWindow> tws;
             vroom::TimeWindow tw(root["jobs"][j]["time_windows"][0][0].asUInt(), 
                                                                root["jobs"][j]["time_windows"][0][1].asUInt());
            job_delivery[0] = root["jobs"][j]["delivery"][0].asUInt();
            tws.push_back(tw);

            // std::cout << "location_index: " << root["jobs"][j]["location_index"].asUInt() <<std::endl;
            // std::cout << "service: " << root["jobs"][j]["service"].asUInt() << std::endl;
            // std::cout << "priority: " << root["jobs"][j]["priority"].asUInt() << std::endl;
            // std::cout << "delivery: " << root["jobs"][j]["delivery"][0].asUInt() << std::endl;


            vroom::Job job(
                Id,
                index,
                setup,
                service,
                job_delivery,
                job_pickup,
                vroom::Skills(),
                priority,
                tws
            );
            Jobs.push_back(job);
        }


        DurationMat = DurationMat_temp;
        CostMat = CostMat_temp;

    }
    return ;
}

Problem::~Problem() {}

std::vector<std::vector<int>> Problem::run_with_custom_matrix() {
    bool GEOMETRY = false;

    vroom::Input problem_instance(amount_dimension_Scene2);

    // Define custom matrix and bypass OSRM call.
    problem_instance.set_durations_matrix("car", std::move(DurationMat));
    problem_instance.set_costs_matrix("car",  std::move(CostMat));

    // Define vehicles (use std::nullopt for no start or no end).
    for (const auto& i : Vehicles) {
        problem_instance.add_vehicle(i);
    }
    // Define jobs with id and index of location in the matrix
    for (const auto& j : Jobs) {
        problem_instance.add_job(j);
    }
    // Solve!
    auto sol = problem_instance.solve(5,  // Exploration level.
                                    4); // Use 4 threads.
    
    log_solution(sol, GEOMETRY);    // 输出结果到命令行
    Unassigned_Mines(sol);
    Get_Duration(sol);
    // 检查 vector
    for (int veh_i = 1; veh_i <= veh_num ; veh_i++)
    {
        bool find_route = false;
        for (const auto& route : sol.routes)
        {
            if (route.vehicle == veh_i ) 
            {
                std::vector<int> veh_path;
                for (const auto& step : route.steps) {
                    if (step.step_type != vroom::STEP_TYPE::START and
                        step.step_type != vroom::STEP_TYPE::END)
                    {
                        veh_path.push_back(step.id);
                    }
                }
                Paths.push_back(veh_path);
                find_route = true;
                break;
            }
        }
        if (!find_route)
        {
            std::vector<int> veh_path2;
            veh_path2.push_back(9999);
            Paths.push_back(veh_path2);
        }
    }
    // for (size_t i = 0; i < Paths.size(); i++)
    // {
    //     for (size_t j = 0; j < Paths[i].size(); j++)
    //     {
    //         std::cout << Paths[i][j] << "  ";
    //     }
    //     std::cout << std::endl;
    // }
    return Paths;
}

void JSON_RW::ReadLocationFromJson(string ReadFile,LocationTrans &locationtrans)
{
    Json::Reader reader;/*用于按照JSON数据格式进行解析*/
	Json::Value root;/*用于保存JSON类型的一段数据*/
	
	ifstream srcFile(ReadFile, ios::binary);/*定义一个ifstream流对象，与文件.json进行关联*/
	if (!srcFile.is_open())
	{
		cout << "Fail to open " << ReadFile << endl;
		return;
	}
    if (reader.parse(srcFile, root))
    {
        for( int i = 0; i < root["vehicles"].size(); i++)
        {
            LatLngPoint veh_start, veh_end;
            veh_start.lng = root["vehicles"][i]["start"][0].asDouble();
            veh_start.lat = root["vehicles"][i]["start"][1].asDouble();
            veh_end.lng = root["vehicles"][i]["end"][0].asDouble();
            veh_end.lat = root["vehicles"][i]["end"][1].asDouble();
            //NOTE
            int vehicle_id= root["vehicles"][i]["id"].asInt();
            locationtrans.VehicleIndex2StartEndPoint[vehicle_id]=std::make_pair(veh_start,veh_end);
        }
        for (int j = 0; j < root["jobs"].size(); j++)
        {
            LatLngPoint job_point;
            job_point.lng = root["jobs"][j]["location"][0].asDouble();
            job_point.lat = root["jobs"][j]["location"][1].asDouble();
            int job_id=root["jobs"][j]["id"].asInt();
            locationtrans.JobIndex2Location[job_id]=job_point;
        }
    }
}

//NOTE:前面读4,后面读output

void JSON_RW::ReadOutputFromJson(LocationTrans &_location_trans,vector<vector<LatLngPoint>>& transfer_routes, 
                                                                                std::vector<std::vector<int>> Paths)
{
    // Json::Reader reader;/*用于按照JSON数据格式进行解析*/
	// Json::Value root;/*用于保存JSON类型的一段数据*/
	
	// ifstream srcFile(ReadFile, ios::binary);/*定义一个ifstream流对象，与文件.json进行关联*/
	// if (!srcFile.is_open())
	// {
	// 	cout << "Fail to open " << ReadFile << endl;
	// 	return;
	// }
    // if (reader.parse(srcFile, root))
    {
        /* ************为了编译暂时加的，按伯辰修改删除************ */
        // std::vector<std::vector<int>> Paths;
        for( int i = 0; i < Paths.size(); i++)
        {
            vector<LatLngPoint> transfer_route;
            vector<LatLngPoint> steps;
            int vehicle_id = i + 1;
            transfer_route.push_back(_location_trans.VehicleIndex2StartEndPoint[vehicle_id].first);
            for(int j = 0; j< Paths[i].size(); j++)
            {
                if(Paths[i][j] == 9999) break;
                int job_id = Paths[i][j];
                transfer_route.push_back(_location_trans.JobIndex2Location[job_id]);
            }
            transfer_route.push_back(_location_trans.VehicleIndex2StartEndPoint[vehicle_id].second);
            //transfer的索引就是vehicle的id，因为输出格式是按照vehicle_id排序过的
            transfer_routes.push_back(transfer_route);
        }
    }
}

void Problem::Unassigned_Mines(const vroom::Solution& sol)
{
    // Log unassigned jobs if any.
    std::cout << "Unassigned job ids: ";
    for (const auto& j : sol.unassigned) {
        std::cout << j.id << ", ";
        unassigned_mines.push_back(j.id);
    }
    std::cout << std::endl;

    return ;
}

void Problem::Get_Duration(const vroom::Solution& sol)
{
    std::vector<int> duration_temp;
    for (const auto& route : sol.routes) {
        // std::cout << "Steps for vehicle " << route.vehicle
        //       << " (cost: " << route.cost;
        // std::cout << " - duration: " << route.duration;
        // std::cout << std::endl;
        duration_temp.push_back(route.duration);
    }
    My_Duration.assign(duration_temp.begin(), duration_temp.end());
}