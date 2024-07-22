//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/BeaconMessage_m.h"
#include "veins/modules/application/traci/common.h" // 所有全域變數宣告在common.h中，並在MyTest11p.cc中定義
#include <queue>

namespace veins {

/**
 * @brief
 * A tutorial demo for TraCI. When the car is stopped for longer than 10 seconds
 * it will send a message out to other cars containing the blocked road id.
 * Receiving cars will then trigger a reroute via TraCI.
 * When channel switching between SCH and CCH is enabled on the MAC, the message is
 * instead send out on a service channel following a Service Advertisement
 * on the CCH.
 *
 * @author Christoph Sommer : initial DemoApp
 * @author David Eckhoff : rewriting, moving functionality to DemoBaseApplLayer, adding WSA
 *
 */

struct car_resource
{
    int remain_cpu;
    int remain_memory;
    bool followed;
    double tmp_time;
    Coord tmp_Position;
    Coord cSpeed;
    LAddress::L2Type followed_car;
    double cal_capability = 25000;
    std::queue<task> pending_tasks; // 待處理之任務(FIFO)
    std::list<task> handling_tasks; // 正在被車輛處理之任務
    //std::list<task> waiting_tasks; // 分配完等待處理的任務
    std::list<std::pair<task, int>> waiting_tasks; // 分配完等待處理的任務
    // 已經分配自己運算，但是因運算資源不足而等待的任務
    //後面的int代表需要運算的大小(與packet size不同，因任務分成多個部分分開計算，若int與packet size相同代表任務完全由自己來運算)
    std::queue<std::pair<task, int>> queuing_tasks;

    car_resource (int c, int m)
    {
        remain_cpu = c;
        remain_memory = m;
    }
};


struct UAV_MapData
{
    double generate_time; // 收到該beacon的時間
    Coord position; // UAV的所在位置
    double Delay; // UAV傳送beacon至該車輛的delay
    double Delay_to_MEC; // 該UAV傳送beacon至Delay最小的MEC的Delay，-1代表該UAV沒有和MEC連線
    double Distance_to_MEC; //該UAV距離Delay最小的MEC的距離，-1代表該UAV沒有和MEC連線
    int remain_cpu;
    int remain_memory;
};


class VEINS_API MyMethodCar : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;
    car_resource node_resource;
    std::map<LAddress::L2Type, UAV_MapData> UAV_map;
    MyMethodCar();
    void dispatchTask();
    void dispatchTaskConsiderEnergy();
    void handleQueuingTask();
    void clearExpiredTask();
    LAddress::L2Type Nearest_MEC = -1;
    double Delay_to_MEC = DBL_MAX;
    double Distance_to_MEC = -1;
    double MEC_generate_time;
    int MEC_remain_cpu;
    int MEC_remain_mem;


protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;

protected:
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void onBSM(DemoSafetyMessage* wsm) override;
    void onBM(BeaconMessage* bsm) override;
    void finish() override;


    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;

};

}
