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
};

struct resource
{
    int remain_cpu;
    int remain_memory;
    double cal_capability = 1000000;
    std::queue<task> received_tasks; //�q�����ݰe�ӵ��ݳB�z��task
    std::list<task> handling_tasks; //�B�z��������
    std::list<task> waiting_tasks; //��浹MEC���ݨ�B�z�^�Ǫ�����

    resource (int c, int m)
    {
        remain_cpu = c;
        remain_memory = m;
    }
};

struct MEC_MapData
{
    double generate_time; // �����MEC ACK���ɶ�
    double Delay_to_MEC; // MEC�ǰeACK�^��UAV��Delay
    double Distance_to_MEC; //MEC��UAV���Z��
};

class VEINS_API CoCaCoUAV : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;
    resource UAV_resource;
    std::map<LAddress::L2Type, MEC_MapData> MEC_map;
    CoCaCoUAV();
    void handleReceivedTask();
    double Delay_to_MEC = DBL_MAX;
    LAddress::L2Type Nearest_MEC = -1;
    double Distance_to_MEC = -1;

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
