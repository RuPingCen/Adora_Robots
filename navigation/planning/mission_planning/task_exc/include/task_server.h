#ifndef TASK_SERVER_H
#define TASK_SERVER_H

#include <thread>
#include <mutex>
#include <iostream>
#include <map>
#include <string>
#include <queue>
#include <sstream>
#include <iomanip>
#include "LaneLineArray.h"
#include "VehicleStat.h"
#include "Localization.h"
#include "Task.h"
#include "Controlsrv.h"
#include "Route.h"
#include "RoadAttri.h"
#include "LaneLine.h"
#include "LaneLineArray.h" 
#include "ObjectArray.h"




class task_server{

public:
    bool SetSpeed(bool enable,float speed,std::string source, void *dora_context);
    bool SetRoute(bool enabel,float d,float s, void *dora_context);
    bool SetStop(bool enable,float distance,std::string source, void *dora_context);
    bool SetBackCar(bool enable,std::string source, void* dora_context);


    //回调函数
    bool onTaskCallRecvd(char *msg, void *dora_context);
    void onStatMsgRecvd(char *msg);
    void onCurPoseSDRecvd(char *msg);
    void onTargetRecvd(char *msg);
    void onAttriRecvd(char *msg);



private:
    std::mutex        mtx_stat,mtx_pose,mtx_object,mtx_road; 
    VehicleStat_h     dora_stat_data;
    CurPose_h         dora_cur_pose;
    ObjectArray_h     dora_object_arry;
    RoadAttri_h       dora_road_attri;
    LaneLineArray_h   dora_road_line;


    VehicleStat_h   get_Stat_WithMutex();
    CurPose_h       get_Curpose_WithMutex();
    ObjectArray_h   get_Object_WithMutex();
    RoadAttri_h     get_RoadAttri_WithMutex();
    
public:
    /* ==================== 通用任务处理函数  ==================== */
    void *Back_Car_pthread(void *arg);

    static void* Back_Car_pthread_wrapper(void* arg) 
    {
        // std::cout << "1111111111111111111111111" << std::endl;
        // 将 void* 转换为传入的结构体类型
        auto context = static_cast<ThreadContext*>(arg);
        // 调用实际的成员函数并传入 dora_context 参数
        void* result = context->instance->Back_Car_pthread(context->dora_context);
        // 释放分配的内存
        delete context;
        return result;
    }

    // 定义一个用于传递参数的结构体
    struct ThreadContext
    {
        task_server* instance;
        void* dora_context;
    };

    /* ======================== 功能函数  ======================== */

    bool Is_have_target(int position,float s_cur,float front,float back,ObjectArray_h &objectArry);



public:
    //任务表
    std::map<unsigned long long,std::string> task_table;
    void Add_task(std::string str);
    void Delete_task();
    void Show_taskTable();

    double park_lat;
    double park_lon;
    double park_x;
    double park_y;
    double park_s;

    double CameraObj_s;

};

#endif
