extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}


#include"task_server.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>

int ct = 0;
task_server core;
int run_flag = 0;
int begin_catch = 0;
int continue_go = 0;  // 需要等待的变量
std::mutex mtx;
std::condition_variable cv;
struct sockaddr_in server_address;

int init_location = 0; // 初始化定位是否成功，该标志位用于处理最开始定位漂移的问题



//****************************************接受消息**********************************************

void task_server::onCurPoseSDRecvd(char *msg)
{
    ct = 1;
    // std::cout << "***********cur_pose**************" << std::endl;
    mtx_pose.lock();
    CurPose_h *cur_pose = reinterpret_cast<CurPose_h *>(msg);
    if (cur_pose != nullptr) 
    {  
        dora_cur_pose = *cur_pose; 
        // std::cout <<"dora_cur_pose.s: "<< dora_cur_pose.s<< std::endl; 
    } 
    else
    {
        std::cerr << "Invalid pointer: cur_pose is null." << std::endl;
    }
    mtx_pose.unlock();
}



//*****************************************************************************

//***********************************加锁访问***********************************

inline CurPose_h task_server::get_Curpose_WithMutex()
{
    CurPose_h res;
    mtx_pose.lock();
    res = dora_cur_pose;
    mtx_pose.unlock();
    return res;
}

//***************************************************************************************************************************


//*******************************************************车辆控制**************************************************************
/**
 * @brief 设置速度
 * @param 
 */
inline bool task_server::SetSpeed(bool enable,float speed,std::string source, void *dora_context){
    Controlsrv_h *srv = new Controlsrv_h;
    srv->type   = srv->Is_swith_speed;
    srv->enable = enable;
    srv->info   = speed;
    srv->source = source;

    Controlsrv_h * srv_out = srv;
    char * output_data = (char*)srv_out;
    std::string out_id = "SetSpeed_service";
    //std::cout<<json_string<<endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Controlsrv_h));
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
    delete srv;

}

inline bool task_server::SetStop(bool enable,float distance,std::string source, void* dora_context){
    Controlsrv_h *srv = new Controlsrv_h;
    srv->type   = srv->Is_stop;
    srv->enable = enable;
    srv->info   = distance;
    srv->source = source;
       
    Controlsrv_h * srv_out = srv;
    char * output_data = (char*)srv_out;
    std::string out_id = "SetStop_service";
    //std::cout<<json_string<<endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Controlsrv_h));
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
    delete srv;
    return true;
}


inline bool task_server::SetBackCar(bool enable,std::string source, void* dora_context){
    Controlsrv_h *srv = new Controlsrv_h;
    srv->type   = srv->Is_back;
    srv->enable = enable;
    srv->source = source;


    Controlsrv_h * srv_out = srv;
    char * output_data = (char*)srv_out;
    std::string out_id = "BackCar_service";
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Controlsrv_h));
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
    delete srv;
    return true;
}
//***********************************************************************************************************



void* task_server::Back_Car_pthread(void *dora_context)
{
    int cut = 0;
    while(true)
    {
        const int rate = 25;   // 设定频率为 xx HZ
        const chrono::milliseconds interval((int)(1000/rate));
        if(ct > 0)  // 接收到了位置信息
        {
            CurPose_h           cur_pose;
            static uint8_t init_location_cnt = 0;
            cur_pose  = get_Curpose_WithMutex();
            if ( cur_pose.s <= 0.2f && init_location == 0) init_location_cnt ++;
            if ( init_location_cnt >= 10)
            {
                init_location_cnt = 0;
                init_location = 1;
                std::cout << "Init location success" << std::endl;
            }

            // std::cout << "cur_pose.s = " << cur_pose.s << std::endl;



            ct = 0; // 清空标志位

            if ( init_location == 1)
            {
                if(cur_pose.s > 11 && cut == 2)
                {
                    std::cout << "第三次停" << std::endl;
                    SetStop(true,9,"Stop",dora_context);
    
                    begin_catch = 1;
    
                    std::this_thread::sleep_for(chrono::milliseconds(100));
                    std::unique_lock<std::mutex> lock(mtx);
                    cv.wait(lock, [] { return continue_go == 1; });
                    continue_go = 0;
    
                    // std::this_thread::sleep_for(chrono::seconds(5));
    
                    SetStop(false,9,"Stop",dora_context);
                    cut = -1;
                    
                }
    
    
    
                else if(cur_pose.s > 2.0 && cut == 1)
                {
                    std::cout << "第二次停" << std::endl;
                    SetStop(true,9,"Stop",dora_context);
    
                    begin_catch = 1;
                    std::this_thread::sleep_for(chrono::milliseconds(100));
                    std::unique_lock<std::mutex> lock(mtx);
                    cv.wait(lock, [] { return continue_go == 1; });
                    continue_go = 0;
    
                    // std::this_thread::sleep_for(chrono::seconds(5));
    
                    SetStop(false,9,"Stop",dora_context);
                    cut = 2;
                    
                }
    
    
    
    
                else if(cur_pose.s > 1.0 && cut == 0)
                {
                    std::cout << "第一次停" << std::endl;
                    SetStop(true,9,"Stop",dora_context);
    
                    begin_catch = 1;
                    std::this_thread::sleep_for(chrono::milliseconds(100));
                    std::unique_lock<std::mutex> lock(mtx);
                    cv.wait(lock, [] { return continue_go == 1; });
                    continue_go = 0;
    
                    // std::this_thread::sleep_for(chrono::seconds(5));
    
                    SetStop(false,9,"Stop",dora_context);
                    cut = 1;
                    
                }
            }




        }
        this_thread::sleep_for(interval);

    }
}

void UDPReceiver(int sock, struct sockaddr_in& server_address) {

    char message[1024];
    char command[64];
    socklen_t server_len = sizeof(server_address);
    int bytes_received;


    server_address.sin_port = htons(33335);
    server_address.sin_addr.s_addr = inet_addr("127.0.0.1"); // 服务器IP地址

    while (true) 
    {   
        std::cout << "11111111111111111111111111111111111" << std::endl;
    //     std::cout << "begin_catch " << begin_catch << std::endl;
        // server_address.sin_port = htons(33334);
        // server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
        // bytes_received = recvfrom(sock, message, sizeof(message), 0, (struct sockaddr *)&server_address, &server_len);
        // std::cout << "2222222222222222222222222222222222222" << std::endl;

        // message[bytes_received] = '\0'; // 确保字符串正确终止
        // std::cout << "Received message: " << message << std::endl;

        if(begin_catch == 1)
        {
            std::strcpy(command, "stop");
            server_address.sin_port = htons(33333);
            server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
            sendto(sock, command, strlen(command), 0, (struct sockaddr *)&server_address, sizeof(server_address));
            std::cout << "catch stop" << std::endl;

            std::this_thread::sleep_for(chrono::milliseconds(50));
            server_address.sin_port = htons(33335);
            server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
            bytes_received = recvfrom(sock, message, sizeof(message), 0, (struct sockaddr *)&server_address, &server_len);
            if (bytes_received > 0) 
            {
                message[bytes_received] = '\0'; // 确保字符串正确终止
                // std::cout << "Received message: " << message << std::endl;
                if (strcmp(message, "finish") == 0) 
                {
                    std::cout << "catch finish" << std::endl;
                    
                    std::lock_guard<std::mutex> lock(mtx);
                    continue_go = 1;
                    cv.notify_one();
                    std::strcpy(command, "back");
                    server_address.sin_port = htons(33333);
                    server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
                    sendto(sock, command, strlen(command), 0, (struct sockaddr *)&server_address, sizeof(server_address));

                    std::this_thread::sleep_for(chrono::seconds(5));
                    server_address.sin_port = htons(33333);
                    server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
                    std::strcpy(command, "Home");
                    sendto(sock, command, strlen(command), 0, (struct sockaddr *)&server_address, sizeof(server_address));

                    begin_catch = 0;
                }
            }
        }

        // bytes_received = recvfrom(sock, message, sizeof(message), 0, (struct sockaddr *)&server_address, &server_len);
        // if (bytes_received > 0) 
        // {
        //     message[bytes_received] = '\0'; // 确保字符串正确终止

        //     if (strcmp(message, "finish") == 0) 
        //     {
        //         std::lock_guard<std::mutex> lock(mtx);
        //         continue_go = 1;
        //         cv.notify_one();
        //         std::strcpy(command, "back");
        //         sendto(sock, command, strlen(command), 0, (struct sockaddr *)&server_address, sizeof(server_address));

        //         std::this_thread::sleep_for(chrono::seconds(5));
        //         std::strcpy(command, "Home");
        //         sendto(sock, command, strlen(command), 0, (struct sockaddr *)&server_address, sizeof(server_address));
        //     }
        // }
        std::this_thread::sleep_for(chrono::milliseconds(20));

    }
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
            // std::cout << "Input Data length: " << data_len << std::endl;
            if(strncmp("cur_pose_all", data_id, 12) == 0)
            {
                core.onCurPoseSDRecvd(data);
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
    std::cout << "task_server_exc" << std::endl;

    int sock;

    auto dora_context = init_dora_context_from_env();

    // 创建UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        std::cerr << "Set socket option failed" << std::endl;
        close(sock);
        return -1;
    }

    // 设置服务器地址
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(33335); // 服务器端口
    // server_address.sin_addr.s_addr = INADDR_ANY; // 服务器IP地址
    server_address.sin_addr.s_addr = inet_addr("127.0.0.1"); // 服务器IP地址

    if (bind(sock, (struct sockaddr*)&server_address, sizeof(server_address)) < 0)
    {
        std::cerr << "Bind failed" << std::endl;
        close(sock);
        return -1;
    }

    // 创建接收和发送线程
    std::thread receiver_thread(UDPReceiver, sock, std::ref(server_address));
    // std::thread sender_thread(UDPSender, sock, std::ref(server_address));

    // 等待发送线程结束
    // sender_thread.join();
    // 等待接收线程结束
    //receiver_thread.detach();



    auto context = new task_server::ThreadContext{ &core, dora_context };
    pthread_t id_1 = 1;
    if (pthread_create(&id_1, nullptr, task_server::Back_Car_pthread_wrapper, context) != 0) 
    {
        std::cerr << "create Pub_road_attri_msg thread fail!" << std::endl;
        exit(-1);
    }


    auto ret = run(dora_context);
    free_dora_context(dora_context);  

    receiver_thread.join();

    std::cout << "END task_server_exc" << std::endl;

    return ret;
}

