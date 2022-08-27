#include "Data.hpp"

double GET_INFO::rad_data(double d)
{
    double pi = 3.1415926535897932384626433832795;
    return d * pi /180.0;
}

double GET_INFO::RealDistance(double lat1,double lng1,double lat2,double lng2)//lat1第一个点纬度,lng1第一个点经度,lat2第二个点纬度,lng2第二个点经度
{
	
	double a;
   	double b;
   	double radLat1 = rad_data(lat1);
   double radLat2 = rad_data(lat2);
   a = radLat1 - radLat2;
   b = rad_data(lng1) - rad_data(lng2);
   double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
   double EARTH_RADIUS = 6378.137;
    s = s * EARTH_RADIUS;
    s = s * 1000;
    return s;
}

void GET_INFO:: WriteDataToLocationJson()
{
    for (int i = 0; i < VehicleNum; i++)
    {
        Json::Value veh;
        Point_data veh_start, veh_end;
        std::vector<double> veh_JWD_temp;
        veh["id"] = Json::Value(i + 1);
        veh["time_window"][0] = TimeWindow[i][0];
        veh["time_window"][1] = TimeWindow[i][1];
        
        veh_start.lng = JWD[i *2][0];
        veh_start.lat = JWD[i *2][1];
        veh_end.lng = JWD[i *2 + 1][0];
        veh_end.lat = JWD[i *2 + 1][1];
        veh["start"][0] = veh_start.lng;
        veh["start"][1] = veh_start.lat;
        veh["end"][0] = veh_end.lng;
        veh["end"][1] = veh_end.lat;
        veh["capacity"][0] = Capacity[i];
        veh["max_tasks"] = Json::Value( JobNum / VehicleNum + 1);
        veh["speed_factor"] = Json::Value(Speed[i]);

        // 新场景Skill
        veh["skills"][0] = Json::Value(i);

        Vehicles_Location["vehicles"].append(veh);
    }
    for (int j = VehicleNum * 2; j < JWD.size(); j++)
    {
        Point_data job_point;
        Json::Value job;
        job["id"] = j - VehicleNum * 2 + 1;
        job["time_windows"][0][0] = TimeWindow[j - VehicleNum][0];
        job["time_windows"][0][1] = TimeWindow[j - VehicleNum][1];
        job["service"] = Json::Value(ServiceTime[j  - VehicleNum * 2]);
        job_point.lng = JWD[j][0];
        job_point.lat = JWD[j][1];
        job["location"][0] = job_point.lng;
        job["location"][1] = job_point.lat;
        job["delivery"][0] = Delivery[j  - VehicleNum * 2];

        // if ( j < 48 ) job["priority"] = 10; // TODO: priority 的定义
        // else job["priority"] = 0;
        job["priority"] = Priorty[j  - VehicleNum * 2];

        // 新场景Skill
        for (int sk_i = 0; sk_i < Skill[j  - VehicleNum * 2].size(); sk_i++)
        {
            job["skills"][sk_i] = Json::Value( Skill[j  - VehicleNum * 2][sk_i]);
        }

        Jobs_Location["jobs"].append(job);
    }

    Json::StyledWriter sw;
    ofstream desFile(LocationJson, std::ios::out);
	if (!desFile.is_open())
	{
	    cout << "Fail to write " << endl;
	    return;
    }
    Json::Value root;
    root["vehicles"] = Vehicles_Location["vehicles"];
    root["jobs"] = Jobs_Location["jobs"];
	desFile << sw.write(root);
	desFile.close();

    std::cout << "Write Location success! " << std::endl;
    return ;
}

void GET_INFO:: WriteDataToMatJson()
{
    for (int i = 0; i < VehicleNum; i++)
    {
        Json::Value veh;
        Point_data veh_start, veh_end;
        std::vector<double> veh_JWD_temp;
        veh["id"] = Json::Value(i + 1);
        veh["time_window"][0] = TimeWindow[i][0];
        veh["time_window"][1] = TimeWindow[i][1];
        
        veh_start.lng = JWD[i *2][0];
        veh_start.lat = JWD[i *2][1];
        veh_end.lng = JWD[i *2 + 1][0];
        veh_end.lat = JWD[i *2 + 1][1];
        veh["start_index"] = Json::Value(i * 2);
        veh["end_index"] = Json::Value(i * 2 +1);
        veh["capacity"][0] = Capacity[i];
        veh["max_tasks"] = Json::Value( JobNum / VehicleNum + 1);
        veh["speed_factor"] = Json::Value(Speed[i]);
        Location.push_back(veh_start);
        Location.push_back(veh_end);

        // 新场景Skill
        veh["skills"][0] = Json::Value(i);

        Vehicles_Mat["vehicles"].append(veh);
    }
    for (int j = VehicleNum * 2; j < JWD.size(); j++)
    {
        Point_data job_point;
        Json::Value job;
        job["id"] = j - VehicleNum * 2 + 1;
        job["time_windows"][0][0] = TimeWindow[j - VehicleNum][0];
        job["time_windows"][0][1] = TimeWindow[j - VehicleNum][1];
        job["service"] = Json::Value(ServiceTime[j - VehicleNum * 2]);
        job_point.lng = JWD[j][0];
        job_point.lat = JWD[j][1];
        job["location_index"] = j;

        job["delivery"][0] = Delivery[j - VehicleNum * 2];
        // if ( j < 48 ) job["priority"] = 10; // TODO: priority 的定义
        // else job["priority"] = 0;
        job["priority"] = Priorty[j - VehicleNum * 2];

        // 新场景Skill
        for (int sk_i = 0; sk_i < Skill[j  - VehicleNum * 2].size(); sk_i++)
        {
            job["skills"][sk_i] = Json::Value( Skill[j  - VehicleNum * 2][sk_i]);
        }

        Location.push_back(job_point);
        Jobs_Mat["jobs"].append(job);
    }

    for (int i = 0; i < Location.size(); i++)
    {
        std::vector<int> temp_list;
        std::vector<int> temp_cost;
        for (int j = 0; j < Location.size(); j++)
        {
            unsigned int length = (int)RealDistance(Location[i].lat, Location[i].lng, Location[j].lat, Location[j].lng) ;
            if (i < VehicleNum * 2 && TimeWindow[i / 2][0] > 0 && i % 2 == 0)
            {
                temp_cost.push_back(length +TimeWindow[i / 2][0]);
                Costs[i][j] = length + TimeWindow[i / 2][0];
                std::cout << "j: " << j << std::endl;
            }
            else
            {
                temp_cost.push_back(length);
                Costs[i][j] = length;
            }
            temp_list.push_back(length);
            Durations[i][j] = length;
        }
        DurationMat.push_back(temp_list);
        CostMat.push_back(temp_cost);

        // std::cout << "Location: " << i << std::endl;
    }
    Car["car"]["durations"] = Durations;
    Car["car"]["costs"] = Costs;


    Json::StyledWriter sw;
    ofstream desFile(MatJson, std::ios::out);
	if (!desFile.is_open())
	{
	    cout << "Fail to write " << endl;
	    return;
    }
    Json::Value root;
    root["vehicles"] = Vehicles_Mat["vehicles"];
    root["jobs"] = Jobs_Mat["jobs"];
    root["matrices"] = Car;
	desFile << sw.write(root);
	desFile.close();

    std::cout << "Write Mat success! " << std::endl;
    return ;
}
void GET_INFO::GetJWDFromFile( std::vector<std::vector<double>> &JWD )
{
    // 此处目前为读取文件，如果直接给定请注释掉
    /********/
    {
        Json::Reader reader;/*用于按照JSON数据格式进行解析*/
	    Json::Value root;/*用于保存JSON类型的一段数据*/

	    ifstream srcFile("2_example_6.json", ios::binary);/*定义一个ifstream流对象，与文件.json进行关联*/
    	if (!srcFile.is_open())
	    {
		    cout << "Fail to read " << endl;
		    return;
    	}
        if (reader.parse(srcFile, root))
        {
            for( int i = 0; i < root["vehicles"].size(); i++)
            {
                Point_data veh_start, veh_end;

                veh_start.lng = root["vehicles"][i]["start"][0].asDouble();
                veh_start.lat = root["vehicles"][i]["start"][1].asDouble();
                veh_end.lng = root["vehicles"][i]["end"][0].asDouble();
                veh_end.lat = root["vehicles"][i]["end"][1].asDouble();

                //此处将读取得到的JWD存入给定的vector中
                std::vector<double> veh_JWD_start, veh_JWD_end;
                veh_JWD_start.push_back(veh_start.lng);
                veh_JWD_start.push_back(veh_start.lat);
                JWD.push_back(veh_JWD_start);
                veh_JWD_end.push_back(veh_end.lng);
                veh_JWD_end.push_back(veh_end.lat);
                JWD.push_back(veh_JWD_end);
            }
            for (int j = 0; j < root["jobs"].size(); j++)
            {
                Point_data job_point;
                job_point.lng = root["jobs"][j]["location"][0].asDouble();
                job_point.lat = root["jobs"][j]["location"][1].asDouble();

                // 此处将读取得到的JWD存入给定的vector中
                std::vector<double> job_JWD_temp;
                job_JWD_temp.push_back(job_point.lng);
                job_JWD_temp.push_back(job_point.lat);
                JWD.push_back(job_JWD_temp);
            }
        }
    }
    /********/
    
    return ;
}
// int main()
// {
//     int TimeWindow[2] = {0, 14400};
//     std::vector<std::vector<double>> jwd;   // 给定的vector标准格式
    
//     // 目前没有给数据，vector从文件中读取，若直接给定则取消注释
//     // GET_INFO FROM_VEC("example_5_2.json", 4, 30, TimeWin, 100, jwd);
//     // GET_INFO FROM_VEC("example_4.json", "example_5_2.json", 4, 30, TimeWindow, 100);
    
//     GET_INFO FROM_VEC("2_example_6.json", "2_example_7.json");
//     FROM_VEC.ReadDataFromJson();
//     FROM_VEC.CalculateCost();
//     FROM_VEC.WriteDataToMatJson();
//    cout << "No Problem reported! " << endl;
//    return 0;
// }