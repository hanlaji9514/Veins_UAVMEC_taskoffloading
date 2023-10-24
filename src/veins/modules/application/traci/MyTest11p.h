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
#include <queue>
#include <math.h>
#include <random>
#include <ctime>

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

struct task
{
    int require_cpu;
    int require_memory;
    int packet_size;
    double delay_limit;
    int qos;
    double start_time;
    double expire_time;

    task(int q);
};

struct resource
{
    int remain_cpu;
    int remain_memory;
    double cal_capability = 1000000;
    std::queue<task> pending_tasks; // 待處理之任務(FIFO)
    std::list<task> handling_tasks; // 正在被處理之任務
    std::list<task> waiting_tasks; // 傳送出去等待處理完回傳的任務

    resource (int c, int m)
    {
        remain_cpu = c;
        remain_memory = m;
    }
};

struct UAV_MapData
{
    double generate_time; // 收到該beacon的時間
    double Delay; // UAV傳送beacon至該車輛的delay
    double Delay_to_MEC; // 該UAV傳送beacon至Delay最小的MEC的Delay，-1代表該UAV沒有和MEC連線
};

class VEINS_API MyTest11p : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;
    resource node_resource;
    std::map<LAddress::L2Type, UAV_MapData> UAV_map;
    MyTest11p();

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;

protected:
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void onBSM(DemoSafetyMessage* wsm) override;
    void onBM(BeaconMessage* bsm) override;

    void dispatchTask();
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;

};

}
