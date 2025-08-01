extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <chrono>
#include "datatype.h"
#include "RoadAttri.h"
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <atomic>
#include <condition_variable>
#include <arpa/inet.h>
using namespace std;

int is_rec = 0;
int is_go = 0;
std::mutex mtx;
std::condition_variable cv;
struct sockaddr_in server_address, client_address;



void UDPReceiver(int sock, struct sockaddr_in& server_address) {
    std::cout << "UDPReceiver is running" << std::endl;
    char message[1024];
    char command[1024];
    socklen_t server_len = sizeof(server_address);
    int bytes_received;
    int count = 0;

    while (true) 
    {
        bytes_received = recvfrom(sock, message, sizeof(message), 0, (struct sockaddr *)&server_address, &server_len);
        if (bytes_received > 0) {
            message[bytes_received] = '\0'; // 确保字符串正确终止
            std::cout << "Received message: " << message << std::endl;
            if (strcmp(message, "start") == 0 && count == 0) 
            {
                is_rec = 1;
                std::cout << "success start" << std::endl;
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [] { return is_go == 1; });
                std::strcpy(command, "go");
                server_address.sin_port = htons(33333);
                server_address.sin_addr.s_addr = inet_addr("127.0.0.1"); // 服务器IP地址
                sendto(sock, command, strlen(command), 0, (struct sockaddr *)&server_address, sizeof(server_address));
                // std::cout << "send go" << std::endl;
                count = 1;
                
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}



void readFileAndParseData(const std::string& filename, RoadAttribute& roadAttr, void* dora_context) {
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Unable to open file" << std::endl;
        return;
    }

    std::unordered_map<std::string, float> data;
    std::string line;

    // 逐行读取文件
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        float value;

        // 将行数据解析为键值对
        if (iss >> key >> value) {
            data[key] = value; // 将键值对存入unordered_map
        } 
    }

    file.close();
    RoadAttri_h *road_attri_msg = new RoadAttri_h;

    if(is_rec == 0)
    {
        road_attri_msg->velocity = 0;
        road_attri_msg->road_width = 0;
        road_attri_msg->aeb_front = 0;
        road_attri_msg->aeb_back = 0;
        road_attri_msg->aeb_left = 0;
        road_attri_msg->aeb_right = 0;
    }
    else if(is_rec == 1)
    {
        if (data.find("id") != data.end()) roadAttr.id_map = static_cast<int>(data["id"]);
        if (data.find("velocity") != data.end()) roadAttr.velocity = data["velocity"];
        if (data.find("road_width") != data.end()) roadAttr.road_width = data["road_width"];
        if (data.find("aeb_front") != data.end()) roadAttr.aeb_front = data["aeb_front"];
        if (data.find("aeb_back") != data.end()) roadAttr.aeb_back = data["aeb_back"];
        if (data.find("aeb_left") != data.end()) roadAttr.aeb_left = data["aeb_left"];
        if (data.find("aeb_right") != data.end()) roadAttr.aeb_right = data["aeb_right"];
        if (data.find("start_s") != data.end()) roadAttr.start_s = data["start_s"];
        if (data.find("end_s") != data.end()) roadAttr.end_s = data["end_s"];

        road_attri_msg->velocity = roadAttr.velocity;
        // std::cout << "road_attri_msg->velocity: " << road_attri_msg->velocity << std::endl;
        road_attri_msg->road_width = roadAttr.road_width;
        road_attri_msg->aeb_front = roadAttr.aeb_front;
        road_attri_msg->aeb_back = roadAttr.aeb_back;
        road_attri_msg->aeb_left = roadAttr.aeb_left;
        road_attri_msg->aeb_right = roadAttr.aeb_right;
        
        std::lock_guard<std::mutex> lock(mtx);
        is_go = 1;
        cv.notify_one();
    }

    std::string out_id = "road_attri_msg";
    RoadAttri_h *road_attri_msg_ptr = road_attri_msg;
    char *output_data = (char *)road_attri_msg_ptr;
    size_t output_data_len = sizeof(RoadAttri_h);
    // std::cout << "output_data_len: " << output_data_len << std::endl;


    // // 打印 output_data 指向的 RoadAttri_h 结构体的内容
    // RoadAttri_h *output_data_as_road_attri_h = reinterpret_cast<RoadAttri_h*>(output_data);
    // std::cout << "Printing output_data contents:" << std::endl;
    // std::cout << "Velocity: " << output_data_as_road_attri_h->velocity << std::endl;
    // std::cout << "Road Width: " << output_data_as_road_attri_h->road_width << std::endl;
    // std::cout << "AEB Front: " << output_data_as_road_attri_h->aeb_front << std::endl;
    // std::cout << "AEB Back: " << output_data_as_road_attri_h->aeb_back << std::endl;
    // std::cout << "AEB Left: " << output_data_as_road_attri_h->aeb_left << std::endl;
    // std::cout << "AEB Right: " << output_data_as_road_attri_h->aeb_right << std::endl;


    int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data, output_data_len);
    // if()
    if(result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }

    // std::cout << "******************************************" << std::endl;
    delete road_attri_msg;

}

int run(void *dora_context)
{

    while(true)
    {
        const int rate = 10;   // 设定频率为 xx HZ
        const chrono::milliseconds interval((int)(1000/rate));
        void * event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            RoadAttribute roadAttr;
            std::string filename = "road_msg.txt";//planning/mission_planning/task_pub/main.cpp
            readFileAndParseData(filename, roadAttr, dora_context);
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
        this_thread::sleep_for(interval);

    }
    return 0;
}


int main()
{
    std::cout << "task_pub" << std::endl;

    int sock;

    auto dora_context = init_dora_context_from_env();

    socklen_t client_len = sizeof(client_address);
    // 创建UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        std::cerr << "Set socket option failed" << std::endl;
        close(sock);
        return -1;
    }
    // 设置服务器地址和端口
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(33334); // 与发送方相同的端口
    server_address.sin_addr.s_addr = inet_addr("127.0.0.1"); // 服务器IP地址
    // server_address.sin_addr.s_addr = INADDR_ANY; // 服务器IP地址
    

    // server_address.sin_addr.s_addr = INADDR_ANY; // 监听任何IP地址
    // 绑定socket到服务器地址和端口
    if (bind(sock, (struct sockaddr*)&server_address, sizeof(server_address)) < 0)
    {
        std::cerr << "Bind failed" << std::endl;
        close(sock);
        return -1;
    }
    // std::cout << "Receiver is listening on port 12345..." << std::endl;
    // 创建接收和发送线程
    // std::thread receiver_thread(UDPReceiver, sock, std::ref(client_address), std::ref(client_len));
    std::thread receiver_thread(UDPReceiver, sock, std::ref(server_address));
    // std::thread sender_thread(UDPSender, sock, std::ref(client_address), std::ref(client_len));
    // 等待发送线程结束
    // sender_thread.join();
    // 发送线程结束后，通知接收线程也应该结束
    // running = false;
    // 等待接收线程结束
    receiver_thread.detach();
    // std::cout << "111111111111111111111111111111111 " << std::endl;

    auto ret = run(dora_context);
    free_dora_context(dora_context);
    
    std::cout << "END task_pub" << std::endl;

    return ret;
}