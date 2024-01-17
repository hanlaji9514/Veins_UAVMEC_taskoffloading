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
#include "veins/modules/mobility/TargetedMobility.h"
#include <queue>
#include <math.h>

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
    LAddress::L2Type id;
    int require_cpu;
    int require_memory;
    int packet_size;
    int full_packet_size;
};

struct UAV_resource
{
    int remain_cpu;
    int remain_memory;
    double cal_capability = 1000000;
    double following_speed;
    double following_speed_1;
    double following_speed_2;
    LAddress::L2Type following_car;
    bool following;
    double following_time;
    std::queue<task> received_tasks; // 正在等待offloading的task
    std::list<task> handling_tasks; // UAV正在處理的task
    std::list<task> waiting_tasks; // UAV轉傳來自car傳送給MEC等待處理的task

    UAV_resource (int c, int m)
    {
        remain_cpu = c;
        remain_memory = m;
    }
};

struct MEC_MapData
{
    double generate_time; // 收到MEC ACK確認連線的時間
    double Delay_to_MEC; // MEC傳送ACK回給UAV所需的Delay
    double Distance_to_MEC; //MEC與UAV之間的距離
    Coord MEC_Position; // MEC的所在座標
};

class VEINS_API MyMethodUAV : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;
    UAV_resource UAV_resource;
    std::map<LAddress::L2Type, MEC_MapData> MEC_map;
    MyMethodUAV();
    void handleReceivedTask();
    double Delay_to_MEC = DBL_MAX;
    LAddress::L2Type Nearest_MEC = -1;
    double Distance_to_MEC = -1;
    Coord MEC_Position = Coord(0,0,0);

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;

protected:
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void onBM(BeaconMessage* bsm) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
};

} // namespace veins
