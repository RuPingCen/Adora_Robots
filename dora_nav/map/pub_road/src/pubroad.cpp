extern "C" {
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}


#include <string.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>
#include <unistd.h>

#include <stdio.h>

// int sumnum;

typedef struct
{
  float x;
  float y;
} Data;

typedef struct
{
    std::vector<float> x_ref;
    std::vector<float> y_ref;
} WayPoint;

int run(void * dora_context)
{
    while(true){
    usleep(10000);
    // sumnum++;
    // std::cout<<sumnum<<std::endl;
    void * event = dora_next_event(dora_context);

    if (event == NULL) {
        printf("[c node] ERROR: unexpected end of event\n");
        return -1;
    }

    enum DoraEventType ty = read_dora_event_type(event);

    if (ty == DoraEventType_Input) {
        FILE * file = fopen("Waypoints.txt", "r");
        if (file == NULL) {
            printf("Failed to open file\n");
            return -1;
        }
        // printf("------------------------");

        Data data;
        WayPoint waypoint;
        int count = 0;

        while (fscanf(file, "%f %f", &data.x, &data.y) == 2) 
        {
            float x = data.x;
            float y = data.y;
            waypoint.x_ref.push_back(x);
            waypoint.y_ref.push_back(y);
        }

        fclose(file);


        // // 记录所有坐标点的S
        // double frenet_s = 0;
        // for(int i = 0; i < waypoint.x_ref.size() - 1; i++) {
        //     // 累加所有路径段的长度
        //     frenet_s += sqrt((waypoint.x_ref[i+1]-waypoint.x_ref[i])*(waypoint.x_ref[i+1]-waypoint.x_ref[i])+(waypoint.y_ref[i+1]-waypoint.y_ref[i])*(waypoint.y_ref[i+1]-waypoint.y_ref[i]));
        //     std::cout<<frenet_s<<std::endl;
        // }

        std::string out_id = "road_lane"; 
        std::vector<char> output_data; 
        for(const auto & x: waypoint.x_ref)
        { 
            float v=x; 
            char* temp = reinterpret_cast<char*>(&v); 
            output_data.insert(output_data.end(), temp, temp + sizeof(float)); 
        } 

        for(const auto & y: waypoint.y_ref)
        { 
            float v=y; 
            char* temp = reinterpret_cast<char*>(&v); 
            output_data.insert(output_data.end(), temp, temp + sizeof(float));  
        } 

        size_t output_data_len = output_data.size();
        int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data.data(), output_data_len);
        if (result != 0)
        {
            std::cerr << "failed to send output" << std::endl;
        }
    } 
    
    else if (ty == DoraEventType_Stop) {
        printf("[c node] received stop event\n");
    } 
    else {
        printf("[c node] received unexpected event: %d\n", ty);
    }
    free_dora_event(event);
  }
  return 0;
}

int main()
{
    std::cout << "test gnss poser for dora " << std::endl;
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "exit test gnss poser ..." << std::endl;
    return ret;
}
