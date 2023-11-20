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

#include "veins/modules/application/traci/MyTestRSU11p.h"

#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include "veins/modules/messages/DemoSafetyMessage_m.h"

#include "veins/modules/messages/BeaconMessage_m.h"

using namespace veins;


Define_Module(veins::MyTestRSU11p);

void MyTestRSU11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

        cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.1, resourceMsg);
        cMessage *beaconTimer = new cMessage("beacon");
        scheduleAt(simTime() + 0.1, beaconTimer);
    }
}

MyTestRSU11p::MyTestRSU11p() : RSU_resource(10000,10000) {} // 讓每一個RSU都有其獨立的RSU_resource

void MyTestRSU11p::onWSA(DemoServiceAdvertisment* wsa)
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

void MyTestRSU11p::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);


    //EV_INFO << myId << ": Receive a packet from: " << wsm->getSenderAddress() << " at time: " << wsm->getTimestamp()/* << " And the data: " << wsm->getDemoData() */<< " Delay = " << simTime() - wsm->getTimestamp();
    if(std::string(wsm->getName()).substr(0, 10) == "MEC_handle") // MEC收到UAV轉傳過來的任務請求
    {
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
        // seglist[4] 是 source_id
        task received_t;
        received_t.source_id = std::stoi(seglist[4]);
        received_t.relay_id = wsm->getSenderAddress();
        received_t.packet_size = wsm->getByteLength();
        received_t.require_cpu = std::stoi(seglist[2]);
        received_t.require_memory = std::stoi(seglist[3]);
        RSU_resource.pending_tasks.push(received_t);
        EV << "RSU " << myId << ": Received a task from UAV " << wsm->getSenderAddress() << ", and the source car is " << received_t.source_id << endl;
        handleReceivedTask();

    }
}

void MyTestRSU11p::onBM(BeaconMessage* bsm)
{
    if(bsm->getBeaconType() == 1)
    {
        //findHost()->getDisplayString().setTagArg("i", 1, "blue");
        EV_INFO << myId << " RSU: I Receive a beacon from UAV " << bsm->getSenderAddress() << " and it's position : " << bsm->getSenderPos() << " / speed : " << bsm->getSenderSpeed() << " / direction : " << bsm->getSenderDirection() << " / Delay to MEC : " << bsm->getDelay_to_MEC();
        LAddress::L2Type recipientAddress = bsm->getSenderAddress();
        BeaconMessage *bm = new BeaconMessage("UAV_beacon_RSUACK");
        populateWSM(bm);
        bm->setBeaconType(2);//Beacon_ACK回傳給UAV
        bm->setSenderAddress(myId);
        bm->setByteLength(300);
        bm->setTimestamp(simTime());
        bm->setRecipientAddress(recipientAddress);
        sendDown(bm);
    }
}

void MyTestRSU11p::handleSelfMsg(cMessage* msg)
{
    if (std::string(msg->getName()).substr(0, 5) == "Task_") // 任務處理完成，須回傳給轉傳過來的UAV讓他回傳給車輛
    {
        std::string name = msg->getName();
        std::stringstream ss(name);
        std::string segment;
        std::vector<std::string> seglist;

        while(std::getline(ss, segment, '_'))
        {
           seglist.push_back(segment);
        }
        LAddress::L2Type source_id = std::stoi(seglist[1]);
        int finish_size = std::stoi(seglist[2]);
        for (auto it = RSU_resource.handling_tasks.begin(); it != RSU_resource.handling_tasks.end();)
        {
            if (it->packet_size == finish_size && it->source_id == source_id)
            {
                RSU_resource.remain_cpu += it->require_cpu;
                RSU_resource.remain_memory += it->require_memory;
                std::string s = "MECTaskSendBack_" + std::to_string(it->packet_size);
                TraCIDemo11pMessage *SendBack = new TraCIDemo11pMessage;
                populateWSM(SendBack);
                SendBack->setByteLength(it->packet_size);
                SendBack->setSenderAddress(myId);
                SendBack->setRecipientAddress(it->relay_id);
                SendBack->setName(s.c_str());
                sendDown(SendBack);
                EV << "RSU " << myId << ": Handling finish. Size = " << finish_size << ", send back to UAV " << it->relay_id << ", and Relay back to " << it->source_id << endl;
                it = RSU_resource.handling_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                break;
            }
            else
            {
                ++it;  // 如果當前元素不符合條件，則遞增迭代器
            }
        }
        if(!RSU_resource.pending_tasks.empty())
            handleReceivedTask();
    }
    else if(!strcmp(msg->getName(), "check_resource"))
    {
        EV << "I'm " << myId << " and my remained cpu = " << RSU_resource.remain_cpu << ", remained memory = " << RSU_resource.remain_memory << endl;
        cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.1, resourceMsg);
    }
    delete msg;
}

void MyTestRSU11p::handleReceivedTask()
{
    int loop_time = RSU_resource.pending_tasks.size();
    for(int i=0; i<loop_time; i++)
    {
        task top_task = RSU_resource.pending_tasks.front();
        RSU_resource.pending_tasks.pop();
        if(RSU_resource.remain_cpu >= top_task.require_cpu && RSU_resource.remain_memory >= top_task.require_memory)
        {
            double cal_time = top_task.packet_size / (RSU_resource.cal_capability * top_task.require_cpu / 100.0);
            RSU_resource.remain_cpu -= top_task.require_cpu;
            RSU_resource.remain_memory -= top_task.require_memory;
            std::string s = "Task_" + std::to_string(top_task.source_id) + "_" + std::to_string(top_task.packet_size);
            EV << "RSU " << myId << ": handling the task! Handling time = " << cal_time << " / size = " << top_task.packet_size << " / remain cpu = " << RSU_resource.remain_cpu << " remain memory = " << RSU_resource.remain_memory << endl;
            RSU_resource.handling_tasks.push_back(top_task);
            cMessage *Task_handlingTimer = new cMessage(s.c_str());
            scheduleAt(simTime() + cal_time, Task_handlingTimer);
        }
        else
        {
            RSU_resource.pending_tasks.push(top_task);
        }
    }
}

void MyTestRSU11p::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);

    /*
    // stopped for for at least 10s?
    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10 && sentMessage == false) {
            findHost()->getDisplayString().setTagArg("i", 1, "red");
            sentMessage = true;

            TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
            populateWSM(wsm);
            wsm->setDemoData(mobility->getRoadId().c_str());
            wsm->setTimestamp(simTime());
            wsm->setSenderAddress(myId);
            EV << "I'm " << myId << "and I sent a packet: " << wsm->getDemoData();
            // host is standing still due to crash
            if (dataOnSch) {
                startService(Channel::sch2, 42, "Traffic Information Service");
                // started service and server advertising, schedule message to self to send later
                scheduleAt(computeAsynchronousSendingTime(1, ChannelType::service), wsm);
            }
            else {
                // send right away on CCH, because channel switching is disabled
                sendDown(wsm);
            }
        }
    }
    else {
        lastDroveAt = simTime();
    }*/
}
