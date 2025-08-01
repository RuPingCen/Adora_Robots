extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}


#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
#include <sys/time.h>

using json = nlohmann::json;

#include "node_routing_core.h"
#include"data_type.h"



int len;
LaneLine line_ref;            //参考路径信息
PathPlanning paths;                  //实现路径规划的对象
map<string,AEB_STOP>  AEB_list; 
map<string,AEB_STOP>  STOP_list; 
map<string,AEB_STOP>::iterator iter;
ChangeLane change_lane_info;  //变道信息
AEB_STOP abe_stop;
Speed_task speed_task;
Backcar_task backcar_task;
Request_h request;
CurrentState navi_data; 
double Expedspeed;
int count_cur_pose = 0;




void onRoadSpeedMsgRecvd( void * dora_context, char *msg)     
{
    RoadAttri_h *attri = reinterpret_cast<RoadAttri_h *>(msg);
    double spe_differ;   
    double Shortest_dis = 100;
    bool is_have_abe = false;
    bool is_have_stop = false;
    
    if(abe_stop.enable)
    {
        is_have_stop = true;
        request.reques_type = request.STOP_ENABLE_H;
        Shortest_dis = abe_stop.info < Shortest_dis ? abe_stop.info : Shortest_dis;
        request.stop_distance = Shortest_dis;

        Request_h * req_out = &request;
        char * output_data = (char*)req_out;
        std::string out_id = "Request";
        int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Request_h));
        if (result != 0)
        {
            std::cerr << "failed to send output" << std::endl;
        }
        // std::cout<< "successful send_stop !!!!!!" << std::endl;
        
        // std::cout << iter->first << " triggered AEB !!!!!!" << std::endl;
    } 
    if(is_have_stop) return;

    Expedspeed = attri->velocity;
    // spe_differ = Expedspeed - navi_data.velocity;      
    // if(fabs(spe_differ) <2 )  request.run_speed = Expedspeed;
    // else request.run_speed = navi_data.velocity + spe_differ*speed_proportion; 
    request.run_speed = Expedspeed;
    // std::cout << "request.run_speed: " << request.run_speed <<std::endl;
    request.reques_type = backcar_task.enable ? request.BACK_ENABLE_H : request.FORWARD_ENABLE_H;    
    

    // std::cout << "++++++++++++++++++" << "planning speed: " << request.run_speed << "++++++++++++++++++" <<std::endl;
    Request_h * req_out = &request;
    char * output_data = (char*)req_out;
    std::string out_id = "Request";
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Request_h));
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }

    return;
}

void onStatDataRecvd(char *msg) 
{
    VehicleStat_h *stat = reinterpret_cast<VehicleStat_h *>(msg);
    navi_data.velocity = stat->veh_speed;
}

bool onControlHandleFunction(char *msg)   
{
    Controlsrv_h *req = reinterpret_cast<Controlsrv_h *>(msg);
    switch( req->type )
     {
        case 1:     //Is_stop
          abe_stop.enable = req->enable;
          abe_stop.info = req->info;
          break;

        case 2:     //Is_aeb
          abe_stop.enable = req->enable;
          abe_stop.info = req->info;
          AEB_list[req->source] = abe_stop;     //无则添加，有则修改
          break;

        case 3:   //Is_back
          backcar_task.enable = req->enable;
          break;

        case 4:   //Is_switch_speed
          speed_task.enable = req->enable;
          speed_task.info = req->info;
          break;
        // default  : std::cout << "Control.srv's type is error!!!!!!" << std::endl;
     }

	return	true;
}


bool onRoutingHandleFunction(char* msg)   //routing_service处理函数
{

    Route_h *req = reinterpret_cast<Route_h *>(msg);

    // std::cout << "***************SETOFFSET********************" << std::endl;
    // std::cout << "change_lane_info.offset_postion: " << change_lane_info.offset_postion <<std::endl;

    change_lane_info.is_change_lane = req->enable;                 //变道标志位
    change_lane_info.offset_postion = req->target_d;               //横向偏移目标位置
    change_lane_info.first_s = req->target_s;                  
	return	true;
}


void toPath(Path_h &path_msg, CurrentPose &current_pose_temp)  
{
    path_msg.x_ref.resize(paths.x_ref.size());
    path_msg.y_ref.resize(paths.y_ref.size());
    int con = 0;
    for (int i = 0; i < paths.x_ref.size(); i++)
    {
        double shift_x = paths.x_ref[i] - current_pose_temp.x;
        double shift_y = paths.y_ref[i] - current_pose_temp.y;
        // con++;
        // std::cout << "****************" << con << "***************"<< std::endl;
        // std::cout << "paths X: " << paths.x_ref[i] << " paths Y: " << paths.y_ref[i] << std::endl;
        // std::cout << "current X: " << current_pose_temp.x << " current Y: " << current_pose_temp.y << std::endl;
        // std::cout << "shift X: " << shift_x << " shift Y: " << shift_y << std::endl;
        
        //将路径点转换到车辆坐标系（后轮中心）
        path_msg.x_ref[i]=shift_x * sin(current_pose_temp.yaw) - shift_y * cos(current_pose_temp.yaw);
        path_msg.y_ref[i]=shift_y * sin(current_pose_temp.yaw) + shift_x * cos(current_pose_temp.yaw);

        // std::cout << "path: X: " << path_msg.x_ref[i] << " path Y: " << path_msg.y_ref[i] << std::endl;
    }
    return;
}


void Current_Pose_callback(void *dora_context, char *msg)
{
    CurrentPose pose;
    CurrentPose current_pose;     //主车当前位姿
    Path_h path;
    CurPose_h *cur_pose = reinterpret_cast<CurPose_h *>(msg);
    navi_data.velocity = 0.25;
    
    count_cur_pose ++;
    struct timeval tv_1;
    gettimeofday(&tv_1, NULL);

    // cout << "The count is : " << count_cur_pose <<" Rec Curr Pose time is: "  << tv_1.tv_sec <<","<< tv_1.tv_usec/1000.0f <<" ms " << std::endl;


    // pose.yaw = j["theta"]
    current_pose.yaw = deg2rad(cur_pose->theta);
    // current_pose.x = cur_pose->x - trans_para_back * cos(pose.yaw); 
    // current_pose.y = cur_pose->y - trans_para_back * sin(pose.yaw);

    current_pose.x = cur_pose->x; 
    current_pose.y = cur_pose->y;

    vector<double> map_waypoints_x_temp;    
    vector<double> map_waypoints_y_temp;
    for (int j = 0; j < ( line_ref.points.size() ); j++)
    {
        map_waypoints_x_temp.push_back( ( line_ref.points.at(j).x ) ) ;
        map_waypoints_y_temp.push_back( ( line_ref.points.at(j).y ) ) ;
    } 

    vector<double> current_sd_para = getFrenet2(current_pose.x, current_pose.y, map_waypoints_x_temp, map_waypoints_y_temp, 0);
     current_pose.s = current_sd_para[0];
     current_pose.d = current_sd_para[1];
    // std::cout << "x: " << current_pose.x << " y: " << current_pose.y 
    // << " yaw: " << current_pose.yaw << " s: " << current_pose.s << " d: " << current_pose.d <<std::endl;

    paths.generate_path(current_pose, line_ref, navi_data, change_lane_info); 
    toPath(path,current_pose);         //传出规划路径，传出path      current_pose_temp

    std::string out_id = "raw_path";
    std::vector<char> output_data;
    for(const auto & x : path.x_ref){
        float v = x;
        // std::cout << "x: " << v << std::endl;
        char *temp = reinterpret_cast<char*>(&v);
        output_data.insert(output_data.end(), temp, temp + sizeof(float));
    }
    for(const auto & y :path.y_ref){
        float v = y;
        // std::cout << "y: " << v << std::endl;
        char *temp = reinterpret_cast<char*>(&v);
        output_data.insert(output_data.end(), temp, temp + sizeof(float));
    }

    size_t output_data_len = output_data.size();
    // std::cout << "output_data_len: " << output_data_len << std::endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data.data(), output_data_len);


    struct timeval tv_2;
    gettimeofday(&tv_2, NULL);
    // cout << "The count is : " << count_cur_pose <<" Pub Raw_Path time is: "  << tv_2.tv_sec <<","<< tv_2.tv_usec/1000.0f <<" ms " << std::endl;


    if(result != 0){
        std::cerr << "failed to send output" << std::endl;
    }

    return;
}


void Line_Recv_callback(char *msg)
{
    // WayPoint *points = reinterpret_cast<WayPoint *>(msg);
    // std::vector<float> x_v;
    // std::vector<float> y_v;
    // int count = 0;
    // for(int i = 0; i < points->x_ref.size(); i++){
    //     count++;
    //     int x = points->x_ref[i];
    //     int y = points->y_ref[i];
    //     std::cout << "x: " << x << "y: " << y << count <<std::endl;
    //     x_v.push_back(x);
    //     y_v.push_back(y);
    // }


    int num_points = len / sizeof(float); 
    float* float_array = reinterpret_cast<float*>(msg); 

    int num_xy_points = num_points / 2; 

    std::vector<float> x_v(float_array, float_array + num_xy_points);
    std::vector<float> y_v(float_array + num_xy_points, float_array + num_points); 
    line_ref.points.clear();
    Point point_ref;
    std::vector<double> x_v_double(x_v.begin(), x_v.end());
    std::vector<double> y_v_double(y_v.begin(), y_v.end());
    int num = 0;
    for(int i = 0; i < x_v.size(); i++){
        num++;
        point_ref.x = x_v[i];
        point_ref.y = y_v[i];
        point_ref.s = getFrenet2(x_v[i], y_v[i], x_v_double, y_v_double, 0)[0];
        // std::cout << "x: " << point_ref.x << " y: " << point_ref.y << " s: " << point_ref.s << " count: " << num <<std::endl;
        line_ref.points.push_back(point_ref);
    }

    // int num_1 = 0;
    // int num_2 = 0;
    // for(const auto &x : x_v){
    //     num_1++;
    //     std::cout << "x: " << x << " " << num_1 << std::endl;
    // }
    // for(const auto &y : y_v){
    //     num_2++;
    //     std::cout << "y: " << y << "  " << num_2 << std::endl;
    // }
    return;
}


int run(void *dora_context)
{

    while (true)
    {

        void * event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            len = data_len;
            // std::cout << "Input Data length: " << data_len << std::endl;
            // std::string id(data_id, data_id_len);
            // std::cout << id.c_str() << std::endl;

            if (strncmp("road_lane", data_id, 9) == 0)
            {
                Line_Recv_callback(data);
            }
            else if (strncmp("cur_pose_all", data_id, 12) == 0)
            {
                Current_Pose_callback(dora_context, data);
            }            
            else if (strncmp("road_attri_msg", data_id, 14) == 0)
            {
                onRoadSpeedMsgRecvd(dora_context, data);
            }
            else if (strncmp("SetSpeed_service", data_id , 16) == 0 || strncmp("BackCar_service", data_id, 15) == 0 || strncmp("SetStop_service", data_id, 15) == 0)
            {
                onControlHandleFunction(data);
            }
            else if(strncmp("routing_service", data_id, 15) == 0)
            {
                onRoutingHandleFunction(data);
            }
            else if(strncmp("VehicleStat", data_id, 11) == 0)
            {
                onStatDataRecvd(data);
            }

        }


        
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "node_routing_core" << std::endl;

    auto dora_context = init_dora_context_from_env();

    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "END node_routing_core" << std::endl;

    return ret;
}


