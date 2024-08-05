#ifndef COMMON_H
#define COMMON_H

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/TargetedMobility.h"
#include "veins/modules/messages/BeaconMessage_m.h"
#include <queue>
#include <math.h>
#include <random>
#include <ctime>
#include <unordered_map>

using namespace veins;

struct Cluster
{
    std::vector<LAddress::L2Type> CarInCluster;
    Coord centroid;
    double max_distance;
    bool tmp_MergeOrNot;
    int Total_task; // 上一個time_slot中總共的任務數量
};

struct Car_info
{
    Coord Position;
    int Num_Task;
};

struct UAV_info
{
    Coord Position;
    double cal_capability;
    int remain_cpu;
    int remain_mem;
};

// 宣告LAB_par(實驗參數)結構體
struct LAB_par
{
    double E1_car; // 發送器所耗功率
    double E2_car; // 放大器所耗功率
    double E3_car; // 接收器所耗功率

    double E1_UAV;
    double E2_UAV;
    double E3_UAV;

    double E1_MEC;
    double E2_MEC;
    double E3_MEC;

    double P_car; // 處理任務所耗功率
    double P_UAV;
    double P_MEC;

    double Energy_perMeter; // 無人機每移動一個單位所需耗能(J)
    double Energy_Hovering; // UAV懸停的能量消耗功率(W)

    double DelayRatio;
    double EnergyRatio;

    double CalculateRatio;
    double DistanceRatio;
};
extern LAB_par parameter;

// 宣告task結構體
struct task
{
    LAddress::L2Type id;
    int require_cpu;
    int require_memory;
    int packet_size;
    double delay_limit;
    int qos;
    double start_time;
    double expire_time;
    bool must_send_MEC;

    task(int q); // 定義在MyTest11p.cc中
};

// 宣告全域變數
extern int TotalPacket;
extern double PacketLossTime;
extern double SuccessedTime;
extern double averageDelayPercent; // 平均完成時間佔總delay限制多少比例
extern double UAV_cal_capability;
extern double MEC_cal_capability;

extern int Successed_Car;
extern int Successed_UAV;
extern int Successed_UAV_MEC;
extern int Successed_MEC;
extern int CantFindOffload;

extern long long taskSize;
extern long double energyComputing;
extern long double energyCommunication;
extern long double energyFlying;
extern long double energyHovering;

extern int CAR_SELF;
extern int CAR_UAV;
extern int CAR_MEC;
extern int CAR_UAV_MEC;


extern std::unordered_map<LAddress::L2Type, Car_info> Car_map;
extern std::unordered_map<LAddress::L2Type, UAV_info> UAV_maps;
extern std::unordered_map<LAddress::L2Type, Coord> Dispatch_Coord;


extern std::mt19937 rnd_generator;



#endif // COMMON_H
