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
    int cpu_require;
    int memory_require;
    double delay_limit;
    int qos;
    double start_time;
    double expire_time;

    task (int q)
    {
        qos = q;
        switch (q)
        {
            case 1:
                delay_limit = (10.0 + fmod(rand() , 40.0)) / 1000.0; // 10ms~50ms
                break;
            case 2:
                delay_limit = 100.0 / 1000.0; // 100ms
                break;
            case 3:
                delay_limit = 150.0 / 1000.0; // 150ms
                break;
            case 4:
                delay_limit = 300.0 / 1000.0; // 300ms
                break;
        }
    }
};

struct resource
{
    int remain_cpu;
    int remain_memory;
    std::queue<task> pending_tasks; //待處理之任務

    resource (int c, int m)
    {
        remain_cpu = c;
        remain_memory = m;
    }
};

class VEINS_API MyTestRSU11p : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;

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
