#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <stdint.h>
#pragma pack(1)                 //强制连续排列


struct RoadAttribute{                      //道路属性结构体
    int id_map;

    float velocity;

    float road_width;

    float aeb_front;
    float aeb_back;
    float aeb_left;
    float aeb_right;

    float start_s;
    float end_s;
};

struct TaskAttribute{                      //任务属性结构体
    int id_task;
    int id_map;

    int task_type;

    float start_s;
    float end_s;

    float task_info;
    char *notes;
};


struct DBAddr{                         //数据库地址及端口
	char user[20];
	char pswd[20];
	char host[20];     //mysql服务器地址
	char dbname[20];   //要连接的数据库名称
	unsigned int port; //端口3306
};


#pragma pack()
#endif // DATA_TYPE_H
