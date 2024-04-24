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

#include <veins/modules/application/traci/GHDSF/GHDSFUAV.h>
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include "veins/modules/messages/DemoSafetyMessage_m.h"

#include "veins/modules/messages/BeaconMessage_m.h"

#include "veins/modules/application/traci/common.h" // 所有全域變數宣告在common.h中，並在MyTest11p.cc中定義

#include <unordered_map>

using namespace veins;

Define_Module(veins::GHDSFUAV);




void GHDSFUAV::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

        UAV_resource.following = false;


        cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.5, resourceMsg);
        cMessage *beaconTimer = new cMessage("beacon");
        scheduleAt(simTime() + 0.05, beaconTimer);
    }
    else if(stage == 1)
    {
        UAV_info uavInfo = {curPosition, UAV_resource.cal_capability, UAV_resource.remain_cpu, UAV_resource.remain_memory};
        UAV_maps.insert(std::make_pair(myId, uavInfo));
        EV << "I'm UAV " << myId << ", my calculate capability = " << UAV_resource.cal_capability << endl;
    }
}

GHDSFUAV::GHDSFUAV() : UAV_resource(200,200) {} // 每個UAV要有自己的UAV_resource

void GHDSFUAV::onWSA(DemoServiceAdvertisment* wsa)
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

void GHDSFUAV::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);
    EV << "UAV " << myId << ": Receive a packet from: " << wsm->getSenderAddress() << " at time: " << simTime().dbl() << " And the Name: " << wsm->getName() << " Delay = " << simTime() - wsm->getTimestamp() << endl;
    if(std::string(wsm->getName()).substr(0, 10) == "UAV_handle") // 車輛傳送給UAV的任務請求
    {
        std::string name = wsm->getName();
        std::stringstream ss(name);
        std::string segment;
        std::vector<std::string> seglist;

        while(std::getline(ss, segment, '_'))
        {
           seglist.push_back(segment);
        }

        // 創建一個 task 物件並設定其成員變數
        // seglist[2] 是 require_cpu
        // seglist[3] 是 require_memory
        // seglist[4] 是 full packet size
        task received_t;
        received_t.id = wsm->getSenderAddress();
        received_t.packet_size = wsm->getByteLength();
        received_t.require_cpu = std::stoi(seglist[2]);
        received_t.require_memory = std::stoi(seglist[3]);
        received_t.full_packet_size = std::stoi(seglist[4]);
        UAV_resource.received_tasks.push(received_t);
        EV << "UAV " << myId << ": received the task request from " << wsm->getSenderAddress() << ", full packet size = " << received_t.full_packet_size << " and the handle size = " << wsm->getByteLength() << endl;
        handleReceivedTask();
    }
    /*
    else if(std::string(wsm->getName()).substr(0, 14) == "UAV_MEC_handle") // 車輛傳送給UAV要求轉傳給MEC(1/3)，以及UAV自己算(1/3)的任務請求
    {

        std::string name = wsm->getName();
        std::stringstream ss(name);
        std::string segment;
        std::vector<std::string> seglist;

        while(std::getline(ss, segment, '_'))
        {
           seglist.push_back(segment);
        }
        // 創建一個 task 物件並設定其成員變數
        // seglist[3] 是 require_cpu
        // seglist[4] 是 require_memory
        // seglist[5] 是 full packet size
        task received_t;
        received_t.id = wsm->getSenderAddress();
        received_t.packet_size = wsm->getByteLength() / 2;
        received_t.require_cpu = std::stoi(seglist[3]);
        received_t.require_memory = std::stoi(seglist[4]);
        received_t.full_packet_size = std::stoi(seglist[5]);
        UAV_resource.received_tasks.push(received_t);
        UAV_resource.waiting_tasks.push_back(received_t);

        EV << "UAV " << myId << ": received the task request from " << wsm->getSenderAddress() << ", full packet size = " << received_t.full_packet_size << " and the handle size = " << wsm->getByteLength() / 2 << endl;
        EV << "UAV " << myId << ": Relaying the task from car " << wsm->getSenderAddress() << " to MEC " << Nearest_MEC << ", full packet size = " << received_t.full_packet_size << " and the handle size = " << wsm->getByteLength() / 2 << endl;

        std::string s = std::string(wsm->getName()).substr(4) + "_" + std::to_string(wsm->getSenderAddress());
        TraCIDemo11pMessage *SendtoMEC = new TraCIDemo11pMessage;
        populateWSM(SendtoMEC);
        SendtoMEC->setByteLength(received_t.packet_size);
        SendtoMEC->setSenderAddress(myId);
        SendtoMEC->setRecipientAddress(Nearest_MEC);
        SendtoMEC->setName(s.c_str());
        sendDown(SendtoMEC);
        handleReceivedTask();
    }
    */
    else if(std::string(wsm->getName()).substr(0, 14) == "UAV_MEC_handle") // 車輛傳送給UAV要求轉傳給MEC
    {

        std::string name = wsm->getName();
        std::stringstream ss(name);
        std::string segment;
        std::vector<std::string> seglist;

        while(std::getline(ss, segment, '_'))
        {
           seglist.push_back(segment);
        }
        // 創建一個 task 物件並設定其成員變數
        // seglist[3] 是 require_cpu
        // seglist[4] 是 require_memory
        // seglist[5] 是 full packet size
        task received_t;
        received_t.id = wsm->getSenderAddress();
        received_t.packet_size = wsm->getByteLength();
        received_t.require_cpu = std::stoi(seglist[3]);
        received_t.require_memory = std::stoi(seglist[4]);
        received_t.full_packet_size = std::stoi(seglist[5]);
        //UAV_resource.received_tasks.push(received_t);
        UAV_resource.waiting_tasks.push_back(received_t);

        //EV << "UAV " << myId << ": received the task request from " << wsm->getSenderAddress() << ", full packet size = " << received_t.full_packet_size << " and the handle size = " << wsm->getByteLength() / 2 << endl;
        EV << "UAV " << myId << ": Relaying the task from car " << wsm->getSenderAddress() << " to MEC " << Nearest_MEC << ", full packet size = " << received_t.full_packet_size << " and the handle size = " << wsm->getByteLength()<< endl;

        double energy_UAVtoMEC = (parameter.E1_UAV + parameter.E3_MEC + (parameter.E2_UAV * Distance_to_MEC * Distance_to_MEC)) * wsm->getByteLength();
        energyCommunication += energy_UAVtoMEC;
        EV << "(UAV->MEC)Energy in communication = " << energy_UAVtoMEC << " / energyCommunication = " << energyCommunication << endl;

        std::string s = std::string(wsm->getName()).substr(4) + "_" + std::to_string(wsm->getSenderAddress());
        TraCIDemo11pMessage *SendtoMEC = new TraCIDemo11pMessage;
        populateWSM(SendtoMEC);
        SendtoMEC->setByteLength(received_t.packet_size);
        SendtoMEC->setSenderAddress(myId);
        SendtoMEC->setRecipientAddress(Nearest_MEC);
        SendtoMEC->setName(s.c_str());
        sendDown(SendtoMEC);
        //handleReceivedTask();
    }
    else if(std::string(wsm->getName()).substr(0, 15) == "MECTaskSendBack") // UAV收到MEC處理完成的任務，要轉傳回給車輛
    {
        int Back_FullPacketSize = std::stoi(std::string(wsm->getName()).substr(16));
        EV << myId << ": Received the finished task from MEC " << wsm->getSenderAddress() << ", and need to send back to the car, the Full packet size = " << Back_FullPacketSize << endl;
        double dis_MECtoUAV_x = curPosition.x - wsm->getSenderPosition().x;
        double dis_MECtoUAV_y = curPosition.y - wsm->getSenderPosition().y;
        double dis_MECtoUAV = dis_MECtoUAV_x * dis_MECtoUAV_x + dis_MECtoUAV_y * dis_MECtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
        double energy_MECtoUAV = (parameter.E1_MEC + parameter.E3_UAV + (parameter.E2_MEC * dis_MECtoUAV)) * wsm->getByteLength();
        energyCommunication += energy_MECtoUAV;
        EV << "(MEC->UAV)Energy in communication = " << energy_MECtoUAV << " / energyCommunication = " << energyCommunication << endl;
        for(auto it = UAV_resource.waiting_tasks.begin(); it != UAV_resource.waiting_tasks.end();)
        {
            EV << "waiting task full size = " << it->full_packet_size << " / back full packet size = " << Back_FullPacketSize << endl;
            if (it->full_packet_size == Back_FullPacketSize)
            {
                std::string s = "MEC_UAV_TaskSendBack_" + std::to_string(it->full_packet_size);
                TraCIDemo11pMessage *SendBacktoNode = new TraCIDemo11pMessage;
                populateWSM(SendBacktoNode);
                SendBacktoNode->setByteLength(wsm->getByteLength());
                SendBacktoNode->setSenderAddress(myId);
                SendBacktoNode->setRecipientAddress(it->id);
                SendBacktoNode->setName(s.c_str());
                sendDown(SendBacktoNode);
                EV << "UAV " << myId << ": Send Back! handle size = " << wsm->getByteLength() << ", full packet size = " << it->full_packet_size << ", send back to car " << it->id << endl;
                it = UAV_resource.waiting_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                break;
            }
            ++it;  // 如果當前元素不符合條件，則遞增迭代器
        }
    }
}

void GHDSFUAV::onBM(BeaconMessage* bsm)
{
    if(!strcmp(bsm->getName(), "UAV_beacon_RSUACK"))//收到MEC ACK
    {
        double d = (simTime() - bsm->getTimestamp()).dbl();
        double dx = bsm->getSenderPos().x - curPosition.x;
        double dy = bsm->getSenderPos().y - curPosition.y;
        double distance = sqrt(dx * dx + dy * dy);
        MEC_map[bsm->getSenderAddress()] = {simTime().dbl(), d, distance};
        if(d < Delay_to_MEC && Nearest_MEC != bsm->getSenderAddress())// 擁有最低的delay
        {
            Delay_to_MEC = d;
            Nearest_MEC = bsm->getSenderAddress();
            Distance_to_MEC = distance;
            MEC_Position = bsm->getSenderPos();
        }
        else if(d > Delay_to_MEC && Nearest_MEC == bsm->getSenderAddress())// 這個MEC是上次Delay最低的MEC，但是這次的Delay比較高，需要檢查他是否仍是所有MEC裡面Delay最低的，若不是就將其取代
        {
            Delay_to_MEC = d;
            Nearest_MEC = bsm->getSenderAddress();// 先寫進去，若他不是delay最低的就會被蓋掉，若是也已經寫進去了
            Distance_to_MEC = distance;
            MEC_Position = bsm->getSenderPos();
            for(std::map<LAddress::L2Type, MEC_MapData>::iterator it = MEC_map.begin(); it != MEC_map.end(); it++)
            {
                if((*it).second.Delay_to_MEC < Delay_to_MEC)
                {
                    Delay_to_MEC = (*it).second.Delay_to_MEC;
                    Nearest_MEC = (*it).first;
                    Distance_to_MEC = (*it).second.Distance_to_MEC;
                    MEC_Position = (*it).second.MEC_Position;
                }
            }
        }
        EV << "UAV " << myId << " : add RSU " << bsm->getSenderAddress() << " in my area, delay = " << simTime().dbl() - bsm->getTimestamp() << endl;
        std::string s = "MEC_" + std::to_string(bsm->getSenderAddress());
        cMessage *MECMsg = new cMessage(s.c_str());
        scheduleAt(simTime() + 1.0, MECMsg);
    }
    else if(!UAV_resource.following && !strcmp(bsm->getName(), "FollowMe"))
    {
       UAV_resource.following_car = bsm->getSenderAddress();
       UAV_resource.following = true;
       UAV_resource.following_time = simTime().dbl();
       EV << "UAV " << myId << ": I'm following the car " << UAV_resource.following_car << "!" << endl;
       double sp = bsm->getCurrentSpeed();
       UAV_resource.following_speed_2 = sp; // 跟隨車輛速度
       BeaconMessage *bm = new BeaconMessage("FollowMe_ACK");
       populateWSM(bm);
       bm->setSenderAddress(myId);
       bm->setByteLength(300);
       bm->setTimestamp(simTime());
       bm->setRecipientAddress(UAV_resource.following_car);
       bm->setSenderPos(curPosition);
       bm->setFollowing(UAV_resource.following);
       sendDown(bm);
    }
    else if(!strcmp(bsm->getName(), "KeepFollow"))
    {
        UAV_resource.following_time = simTime().dbl();
        double sp = bsm->getCurrentSpeed();
        Coord MovingDiff = bsm->getSenderSpeed(); // 取得兩個time slot追蹤車輛的座標差
        Coord following_loc = bsm->getSenderPos();
        following_loc.x += 10 * MovingDiff.x;
        following_loc.y += 10 * MovingDiff.y; // 先直接將兩個Time slot的座標差(轉換成秒)當作速度在X軸和Y軸的分量
        following_loc.z = 3;
        if(Nearest_MEC != -1) // 若UAV有連結MEC，將目的地設定為預測車輛位置與MEC位置的中心點
        {
            following_loc.x = (following_loc.x + MEC_Position.x) / 2;
            following_loc.y = (following_loc.y + MEC_Position.y) / 2;
        }
        EV << "sp = " << sp << endl;
        UAV_resource.following_speed_1 = sp;
        UAV_resource.following_speed =
                0.8 * UAV_resource.following_speed_1 + 0.2 * (UAV_resource.following_speed_1 - UAV_resource.following_speed_2) + 1.0;
        if(UAV_resource.following_speed <= 0) // 避免將speed設置成0 or 負數(因設定成0會導致該mobility消失)
            UAV_resource.following_speed = 1.0;
        EV << "v-1 = " << UAV_resource.following_speed_1 << " / v-2 = " << UAV_resource.following_speed_2 << " / V = " << UAV_resource.following_speed << endl;
        UAV_resource.following_speed_2 = UAV_resource.following_speed_1;
        EV << "Nearest MEC = " << Nearest_MEC << " / MEC position = " << MEC_Position << endl;
        EV << "UAV " << myId << ": Change my destination to " << following_loc << ", and my speed is set to " << UAV_resource.following_speed << endl;
        auto mobilityModule = check_and_cast<veins::TargetedMobility*>(getParentModule()->getSubmodule("mobility"));
        // 呼叫targetMobility的去更新目的地
        mobilityModule->par("speed").setDoubleValue(UAV_resource.following_speed);
        mobilityModule->updateDestination(following_loc);
    }

}

void GHDSFUAV::handleSelfMsg(cMessage* msg)
{
    if(!strcmp(msg->getName(), "beacon"))
    {
        if((UAV_resource.following) && simTime().dbl() >= (UAV_resource.following_time + 0.5))
        {
            EV << "UAV " << myId << ": I'm loss the follow of car " << UAV_resource.following_car << endl;
            UAV_resource.following = false;
        }
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
        bsm->setFollowing(UAV_resource.following);
        if(Nearest_MEC != -1)
        {
            bsm->setDelay_to_MEC(Delay_to_MEC);
            bsm->setDistance_to_MEC(Distance_to_MEC);
        }
        else
        {
            bsm->setDelay_to_MEC(-1);
            bsm->setDistance_to_MEC(-1);
        }
        bsm->setRemain_cpu(UAV_resource.remain_cpu);
        bsm->setRemain_mem(UAV_resource.remain_memory);
        /*if(!Dispatch_Coord.empty())
        {
            Coord myDestination = Dispatch_Coord[myId];
            EV << "UAV " << myId << ": Change my destination to " << myDestination << endl;
            auto mobilityModule = check_and_cast<veins::TargetedMobility*>(getParentModule()->getSubmodule("mobility"));
            // 呼叫targetMobility的去更新目的地
            mobilityModule->updateDestination(myDestination);
        }*/
        delete msg;
        sendDown(bsm);
        cMessage *beaconTimer = new cMessage("beacon");
        scheduleAt(simTime() + 0.05, beaconTimer);
    }
    else if (std::string(msg->getName()).substr(0, 4) == "MEC_")
    {
        LAddress::L2Type num = std::stoi(std::string(msg->getName()).substr(4));
        delete msg;
        if(simTime().dbl() == MEC_map[num].generate_time + 1.0)
        {
            MEC_map.erase(num);
            EV << "UAV " << myId << " : RSU " << num << " is not in my area" << endl;
            // 刪除其中一個MEC後需要找剩下來中Delay最低的那個MEC
            Nearest_MEC = -1;
            Delay_to_MEC = DBL_MAX;
            Distance_to_MEC = -1;
            MEC_Position = Coord(0,0,0);
            if(!MEC_map.empty())
            {
                for(std::map<LAddress::L2Type, MEC_MapData>::iterator it = MEC_map.begin(); it != MEC_map.end(); it++)
                {
                    if((*it).second.Delay_to_MEC < Delay_to_MEC)
                    {
                        Delay_to_MEC = (*it).second.Delay_to_MEC;
                        Nearest_MEC = (*it).first;
                        Distance_to_MEC = (*it).second.Distance_to_MEC;
                        MEC_Position = (*it).second.MEC_Position;
                    }
                }
            }

        }
    }
    else if (std::string(msg->getName()).substr(0, 5) == "Task_") // 幫車輛的運算任務計算完成，準備回傳給車輛
    {
        std::string name = msg->getName();
        std::stringstream ss(name);
        std::string segment;
        std::vector<std::string> seglist;
        delete msg;
        while(std::getline(ss, segment, '_'))
        {
           seglist.push_back(segment);
        }
        LAddress::L2Type finish_id = std::stoi(seglist[1]);
        int finish_size = std::stoi(seglist[2]);
        for (auto it = UAV_resource.handling_tasks.begin(); it != UAV_resource.handling_tasks.end();)
        {
            if (it->packet_size == finish_size && it->id == finish_id)
            {
                UAV_resource.remain_cpu += it->require_cpu;
                UAV_resource.remain_memory += it->require_memory;
                std::string s = "TaskSendBack_" + std::to_string(it->full_packet_size);
                TraCIDemo11pMessage *SendBack = new TraCIDemo11pMessage;
                populateWSM(SendBack);
                SendBack->setByteLength(it->full_packet_size);
                SendBack->setSenderAddress(myId);
                SendBack->setRecipientAddress(it->id);
                SendBack->setName(s.c_str());
                sendDown(SendBack);
                EV << "UAV " << myId << ": Handling finish. Handle Size = " << finish_size << ", full packet size = " << it->full_packet_size << ", send back to car " << finish_id << " / remain cpu = " << UAV_resource.remain_cpu << ", remain memory = " << UAV_resource.remain_memory << endl;
                it = UAV_resource.handling_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器

                BeaconMessage *bsm = new BeaconMessage("UAV_beacon"); // 更新目前的狀態給其他車輛知道
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
                else
                    bsm->setDelay_to_MEC(-1);
                bsm->setRemain_cpu(UAV_resource.remain_cpu);
                bsm->setRemain_mem(UAV_resource.remain_memory);
                sendDown(bsm);
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
        EV << "The MEC in my Area :" << endl;
        for (const auto &pair : MEC_map)
        {
            LAddress::L2Type key = pair.first;
            EV << " Key: " << key << ", Generation time: " << pair.second.generate_time << ", Delay to MEC: " << pair.second.Delay_to_MEC << endl;
        }
        delete msg;
        UAV_maps[myId].Position = curPosition;
        UAV_maps[myId].remain_cpu = UAV_resource.remain_cpu;
        UAV_maps[myId].remain_mem = UAV_resource.remain_memory;

        computeFlyingEnergy();

        cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.5, resourceMsg);
    }
}

void GHDSFUAV::handleReceivedTask()
{
    int loop_time = UAV_resource.received_tasks.size();
    for(int i=0; i<loop_time; i++)
    {
        task top_task = UAV_resource.received_tasks.front();
        UAV_resource.received_tasks.pop();
        if(UAV_resource.remain_cpu >= top_task.require_cpu && UAV_resource.remain_memory >= top_task.require_memory) // UAV處理任務，不行的也只能放著
        {
            double cal_time = top_task.packet_size / (UAV_resource.cal_capability * top_task.require_cpu / 100.0);
            UAV_resource.remain_cpu -= top_task.require_cpu;
            UAV_resource.remain_memory -= top_task.require_memory;
            std::string s = "Task_" + std::to_string(top_task.id) + "_" + std::to_string(top_task.packet_size);
            EV << "UAV " << myId << ": handling the task! Handling time = " << cal_time << " / handle size = " << top_task.packet_size << " / Full packet size = " << top_task.full_packet_size << " / remain cpu = " << UAV_resource.remain_cpu << " remain memory = " << UAV_resource.remain_memory << endl;
            UAV_resource.handling_tasks.push_back(top_task);
            energyComputing += cal_time * parameter.P_UAV;
            EV << "UAV " << myId << ": EnergyComsumption = " << cal_time * parameter.P_UAV << " / energyComputing = " << energyComputing << endl;
            cMessage *Task_handlingTimer = new cMessage(s.c_str());
            scheduleAt(simTime() + cal_time, Task_handlingTimer);
        }
        else
        {
            UAV_resource.received_tasks.push(top_task);
        }
    }
    BeaconMessage *bsm = new BeaconMessage("UAV_beacon"); // 更新目前的狀態給其他車輛知道
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
    {
        bsm->setDelay_to_MEC(Delay_to_MEC);
        bsm->setDistance_to_MEC(Distance_to_MEC);
    }
    else
    {
        bsm->setDelay_to_MEC(-1);
        bsm->setDistance_to_MEC(-1);
    }
    bsm->setRemain_cpu(UAV_resource.remain_cpu);
    bsm->setRemain_mem(UAV_resource.remain_memory);
    sendDown(bsm);

}

void GHDSFUAV::computeFlyingEnergy()
{
    if(lastCoord == Coord(0,0,0))
        lastCoord = curPosition;
    Coord nowCoord = curPosition;
    double Dis_x = nowCoord.x - lastCoord.x;
    double Dis_y = nowCoord.y - lastCoord.y;
    EV << "Dis_x = " << Dis_x << " / Dis_y = " << Dis_y << endl;
    double movingDistance = sqrt(Dis_x * Dis_x + Dis_y * Dis_y);
    energyFlying += parameter.Energy_perMeter * movingDistance;
    EV << "UAV " << myId << ": lastCoord = " << lastCoord << " / nowCoord = " << nowCoord << " / movingDistance = " << movingDistance << endl;
    lastCoord = nowCoord;
}

void GHDSFUAV::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
}

