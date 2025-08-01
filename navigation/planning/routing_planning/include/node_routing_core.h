#ifndef NODE_ROUTING_CORE_H
#define NODE_ROUTING_CORE_H

#include"data_type.h"
#include "frenet.h"
#include "Planning.h"
#include "pathplanning.h"
#include "Localization.h"
#include "RoadAttri.h"
#include "Controlsrv.h"
#include "Route.h"
#include "VehicleStat.h"
       

void Line_Recv_callback(char *msg);   //获取车道线
void toPath(Path_h &path_msg, CurrentPose &current_pose_temp); 
void Current_Pose_callback(void *dora_context, char *msg);
void onRoadSpeedMsgRecvd(void * dora_context, char *msg);
bool onControlHandleFunction(char *msg);   
bool onRoutingHandleFunction(char* msg);







#endif // NODE_ROUTING_CORE_H

