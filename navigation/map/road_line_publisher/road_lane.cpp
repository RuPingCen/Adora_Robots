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
#include <sys/time.h>



#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "road_lane.h"
#include "Localization.h"
#include "SlamPose.h"

int len;
int rec_count = 0;

// struct Pose2D
// {
//     double x;
//     double y;
//     double theta;
// };

std::vector<double> x_v_double;
std::vector<double> y_v_double;

double s_start_x;
double s_start_y;
double s_end_x;
double s_end_y;


void Map_Point_Callback(char *msg){
    // std::cout << "-------------------" << std::endl;


    int num_points = len / sizeof(float); 
    float* float_array = reinterpret_cast<float*>(msg); 

    int num_xy_points = num_points / 2; 

    std::vector<float> x_v(float_array, float_array + num_xy_points);
    std::vector<float> y_v(float_array + num_xy_points, float_array + num_points); 

    x_v_double.assign(x_v.begin(), x_v.end());
    y_v_double.assign(y_v.begin(), y_v.end());

    s_start_x = x_v_double[0];
    s_start_y = y_v_double[0];
    s_end_x = x_v_double.back();
    s_end_y = y_v_double.back();
}



void Current_Pose_callback(void *dora_context, char *msg)
{
    // rec_count++;
    // struct timeval tv_1;
    // gettimeofday(&tv_1, NULL);

    // cout << "The count is : " << rec_count << " Rec GPS time is: "  << tv_1.tv_sec <<","<< tv_1.tv_usec/1000.0f <<" ms " << std::endl;


    // std::cout << "++++++++++++++++" << std::endl;
    Pose2D_h pose;
    //current_pose.yaw = deg2rad(msg->theta)
    // std::string data_str(msg, len);
    // json j = json::parse(data_str);
    // // std::cout << j << std::endl;

    // pose.x = j["position"]["x"].get<double>();
    // // std::cout << "x"<< pose.x << std::endl;
    // pose.y = j["position"]["y"].get<double>();
    // // std::cout << "y"<< pose.y << std::endl;
    // double theta_raw = j["orientation"]["Heading"].get<double>();
    // // std::cout << "theta_raw"<< theta_raw << std::endl;
    // double theta = theta_raw*57.3;
    // while (theta < 0) {
    //     theta += 360;
    // }
    // while (theta >= 360) {
    //     theta -= 360;
    // }
    Pose2D_h *cur_pose = reinterpret_cast<Pose2D_h *>(msg);
    pose.x = cur_pose->x;
    pose.y = cur_pose->y;
    pose.theta = cur_pose->theta;
    // std::cout << "cur_pose theta: "<< pose.theta << std::endl;
    
    // pose.theta = (cur_pose->theta >= 0) && (cur_pose->theta < 90) ?   //-->>真北方向夹角转地图坐标系
    //                             90 - cur_pose->theta : 450 - cur_pose->theta;   
    // pose.theta = j["theta"]


    std::vector<double> start_frenet = getFrenet2(s_start_x, s_start_y, x_v_double, y_v_double, 0);
    std::vector<double> end_frenet = getFrenet2(s_end_x, s_end_y, x_v_double, y_v_double, 0); 
    std::vector<double> cur_frenet = getFrenet2(pose.x,pose.y,x_v_double, y_v_double, 0);

    CurPose_h cur_pose_all;
    cur_pose_all.x = pose.x;
    cur_pose_all.y = pose.y;
    cur_pose_all.theta = pose.theta;
    cur_pose_all.s = cur_frenet[0];
    cur_pose_all.d = cur_frenet[1];

    // std::cout << "x: " << cur_pose_all.x << " y: " << cur_pose_all.y << " theta: "<< cur_pose_all.theta<< " s: " << cur_pose_all.s << std::endl;


    std::string out_id = "cur_pose_all";
    CurPose_h *pose_all_ptr = &cur_pose_all;
    char *output_data = (char*)pose_all_ptr;
    size_t output_data_len = sizeof(cur_pose_all);
    // std::cout << "output_data_len: " << output_data_len << std::endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data, output_data_len);
    
    // struct timeval tv_2;
    // gettimeofday(&tv_2, NULL);

    // cout  << "The count is : " << rec_count <<" Pub Curr Pose time is: "  << tv_2.tv_sec <<","<< tv_2.tv_usec/1000.0f <<" ms " << std::endl;



    if(result != 0){
        std::cerr << "failed to send output" << std::endl;
    }

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
            std::string id(data_id, data_id_len);
            // std::cout << "Input Data length: " << data_len << std::endl;
            // std::cout << id.c_str() << std::endl;

            if (strncmp("cur_pose", data_id, 8) == 0)
            {
                Current_Pose_callback(dora_context, data);
            }
            else if (strncmp("road_lane", data_id, 9) == 0)
            {
                Map_Point_Callback(data);
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
    std::cout << "road_lane " << std::endl;

    auto dora_context = init_dora_context_from_env();

    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "END road_lane" << std::endl;

    return ret;
}


