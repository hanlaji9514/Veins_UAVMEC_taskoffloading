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

#include "veins/modules/application/traci/MyTestUAV11p.h"

#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include "veins/modules/messages/DemoSafetyMessage_m.h"

#include "veins/modules/messages/BeaconMessage_m.h"

using namespace veins;

Define_Module(veins::MyTestUAV11p);

std::map<LAddress::L2Type, MEC_MapData> MEC_map;

double Delay_to_MEC = DBL_MAX;
LAddress::L2Type Nearest_MEC;

void MyTestUAV11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

        cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.1, resourceMsg);
        cMessage *beaconTimer = new cMessage("beacon");
        scheduleAt(simTime() + 0.05, beaconTimer);
    }
}

MyTestUAV11p::MyTestUAV11p() : UAV_resource(200,200) {} // 讓每一個node都有其獨立的node_resource

void MyTestUAV11p::onWSA(DemoServiceAdvertisment* wsa)
{
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
        currentSubscribedServiceId = wsa->getPsid();
        if (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService(static_cast<Channel>(wsa->getTargetChannel()), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

void MyTestUAV11p::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);
    //EV << myId << ": Receive a packet from: " << wsm->getSenderAddress() << " at time: " << wsm->getTimestamp()/* << " And the data: " << wsm->getDemoData() */<< " Delay = " << simTime() - wsm->getTimestamp();
    if(std::string(wsm->getName()).substr(0, 10) == "UAV_handle")
    {
        EV << "UAV " << myId << ": Handling the task from " << wsm->getSenderAddress() << " and the packet size = " << wsm->getByteLength() << endl;
        std::string name = wsm->getName();
        std::stringstream ss(name);
        std::string segment;
        std::vector<std::string> seglist;

        // 使用 '_' 來分割字串
        while(std::getline(ss, segment, '_'))
        {
           seglist.push_back(segment);
        }

        // 創建一個 task 物件並設定其成員變數
        // seglist[2] 是 require_cpu
        // seglist[3] 是 require_memory
        task received_t;
        received_t.id = wsm->getSenderAddress();
        received_t.packet_size = wsm->getByteLength();
        received_t.require_cpu = std::stoi(seglist[2]);
        received_t.require_memory = std::stoi(seglist[3]);
        UAV_resource.received_tasks.push(received_t);
        handleReceivedTask();
    }
    /*if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getDemoData(), 9999);
    if (!sentMessage) {
        sentMessage = true;
        // repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);//myId是該node的網卡（nic)的id
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01, 0.2), wsm->dup());
    }*/
}

void MyTestUAV11p::onBM(BeaconMessage* bsm)
{
    if(bsm->getBeaconType() == 2)//收到MEC ACK
    {
        double d = (simTime() - bsm->getTimestamp()).dbl();
        MEC_map[bsm->getSenderAddress()] = {simTime().dbl(), d};
        if(d < Delay_to_MEC && Nearest_MEC != bsm->getSenderAddress())//擁有最低的delay
        {
            Delay_to_MEC = d;
            Nearest_MEC = bsm->getSenderAddress();
        }
        else if(d > Delay_to_MEC && Nearest_MEC == bsm->getSenderAddress())//為目前delay最低的MEC，但這一次的delay高於上一次，需要檢查他是不是仍是所有MEC中delay最低的，若不是就將其取代
        {
            Delay_to_MEC = d;
            Nearest_MEC = bsm->getSenderAddress();//先寫進去，如果他不是delay最低的就會被改，如果是的話也已經預先寫進去了
            for(std::map<LAddress::L2Type, MEC_MapData>::iterator it = MEC_map.begin(); it != MEC_map.end(); it++)
            {
                if((*it).second.Delay_to_MEC < Delay_to_MEC)
                {
                    Delay_to_MEC = (*it).second.Delay_to_MEC;
                    Nearest_MEC = (*it).first;
                }
            }
        }
        EV << "UAV " << myId << " : add RSU " << bsm->getSenderAddress() << " in my area, delay = " << simTime().dbl() - bsm->getTimestamp() << endl;
        std::string s = "MEC_" + std::to_string(bsm->getSenderAddress());
        cMessage *MECMsg = new cMessage(s.c_str());
        scheduleAt(simTime() + 2.0, MECMsg);
    }
}

void MyTestUAV11p::handleSelfMsg(cMessage* msg)
{
    /*
    if (TraCIDemo11pMessage* wsm = dynamic_cast<TraCIDemo11pMessage*>(msg)) {
        // send this message on the service channel until the counter is 3 or higher.
        // this code only runs when channel switching is enabled
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() + 1);
        if (wsm->getSerial() >= 3) {
            // stop service advertisements
            stopService();
            delete (wsm);
        }
        else {
            scheduleAt(simTime() + 1, wsm);
        }
    }
    else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
    */
    if(!strcmp(msg->getName(), "beacon"))
    {
        BeaconMessage *bsm = new BeaconMessage("UAV_beacon");
        // populate some common properties with the BaseWaveApplLayer method
        populateWSM(bsm);
        bsm->setBeaconType(1);
        bsm->setSenderAddress(myId);
        // set the sender position with the mobility module position
        bsm->setSenderPos(curPosition);
        // set the speed with the mobility module speed
        bsm->setSenderSpeed(curSpeed);
        // set the heading with the mobility module direction
        // send the BSM to the MAC layer immediately
        bsm->setByteLength(300); //beacon message 大約為300Bytes
        bsm->setTimestamp(simTime());
        if(Nearest_MEC != -1)
            bsm->setDelay_to_MEC(Delay_to_MEC);
        bsm->setRemain_cpu(UAV_resource.remain_cpu);
        bsm->setRemain_mem(UAV_resource.remain_memory);
        sendDown(bsm);
        cMessage *beaconTimer = new cMessage("beacon");
        scheduleAt(simTime() + 0.05, beaconTimer);
    }
    else if (std::string(msg->getName()).substr(0, 4) == "MEC_")
    {
        LAddress::L2Type num = std::stoi(std::string(msg->getName()).substr(4));
        // 現在，變數 num 包含了 "MEC_" 之後的數字
        if(simTime().dbl() == MEC_map[num].generate_time + 2.0)
        {
            MEC_map.erase(num);
            EV << "UAV " << myId << " : RSU " << num << " is not in my area" << endl;
            // 刪除其中一個MEC時需要重新找剩下來delay最低的一個MEC
            Nearest_MEC = -1;
            Delay_to_MEC = DBL_MAX;
            if(!MEC_map.empty())
            {
                for(std::map<LAddress::L2Type, MEC_MapData>::iterator it = MEC_map.begin(); it != MEC_map.end(); it++)
                {
                    if((*it).second.Delay_to_MEC < Delay_to_MEC)
                    {
                        Delay_to_MEC = (*it).second.Delay_to_MEC;
                        Nearest_MEC = (*it).first;
                    }
                }
            }

        }
    }
    else if (std::string(msg->getName()).substr(0, 5) == "Task_") // 幫車輛運算的任務計算完成，準備回傳給車輛
    {
        std::string name = msg->getName();
        std::stringstream ss(name);
        std::string segment;
        std::vector<std::string> seglist;

        // 使用 '_' 來分割字串
        while(std::getline(ss, segment, '_'))
        {
           seglist.push_back(segment);
        }
        LAddress::L2Type finish_id = std::stoi(seglist[1]);
        int finish_size = std::stoi(seglist[2]);
        for (auto it = UAV_resource.handling_tasks.begin(); it != UAV_resource.handling_tasks.end();)
        {
            if (it->packet_size == finish_size)
            {
                UAV_resource.remain_cpu += it->require_cpu;
                UAV_resource.remain_memory += it->require_memory;
                std::string s = "TaskSendBack_" + std::to_string(it->packet_size);
                TraCIDemo11pMessage *SendBack = new TraCIDemo11pMessage;
                populateWSM(SendBack);
                SendBack->setByteLength(it->packet_size);
                SendBack->setSenderAddress(myId);
                SendBack->setRecipientAddress(it->id);
                SendBack->setName(s.c_str());
                sendDown(SendBack);
                it = UAV_resource.handling_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                EV << "UAV " << myId << ": Handling finish. Size = " << finish_size << ", send back to car " << finish_id << endl;
                break;
            }
            else
            {
                ++it;  // 如果當前元素不符合條件，則遞增迭代器
            }
        }
        if(!UAV_resource.received_tasks.empty())
            handleReceivedTask();
    }
    else if(!strcmp(msg->getName(), "check_resource"))
    {
        EV << "I'm UAV " << myId << " and my remained cpu = " << UAV_resource.remain_cpu << ", remained memory = " << UAV_resource.remain_memory << endl;
        cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.1, resourceMsg);
    }
}

void MyTestUAV11p::handleReceivedTask()
{
    int loop_time = UAV_resource.received_tasks.size();
    for(int i=0; i<loop_time; i++)
    {
        task top_task = UAV_resource.received_tasks.front();
        UAV_resource.received_tasks.pop();
        if(UAV_resource.remain_cpu >= top_task.require_cpu && UAV_resource.remain_memory >= top_task.require_memory) // 車輛自行處理
        {
            double cal_time = top_task.packet_size / (UAV_resource.cal_capability * top_task.require_cpu / 100.0);
            UAV_resource.remain_cpu -= top_task.require_cpu;
            UAV_resource.remain_memory -= top_task.require_memory;
            std::string s = "Task_" + std::to_string(top_task.id) + "_" + std::to_string(top_task.packet_size);
            EV << "UAV " << myId << ": handling the task! Handling time = " << cal_time << " / size = " << top_task.packet_size << " / remain cpu = " << UAV_resource.remain_cpu << " remain memory = " << UAV_resource.remain_memory << endl;
            UAV_resource.handling_tasks.push_back(top_task);
            cMessage *Task_handlingTimer = new cMessage(s.c_str());
            scheduleAt(simTime() + cal_time, Task_handlingTimer);
        }
        else
        {
            UAV_resource.received_tasks.push(top_task);
        }
    }

}

void MyTestUAV11p::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
}
