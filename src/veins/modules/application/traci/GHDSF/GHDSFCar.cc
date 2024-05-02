// 20231025 Testing the git when I change the file.
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

#include <veins/modules/application/traci/GHDSF/GHDSFCar.h>
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include "veins/modules/messages/DemoSafetyMessage_m.h"

#include "veins/modules/messages/BeaconMessage_m.h"

#include "veins/modules/application/traci/common.h" // 所有全域變數宣告在common.h中，並在MyTest11p.cc中定義

using namespace veins;

int CAR_SELF = 0;
int CAR_UAV = 0;
int CAR_MEC = 0;
int CAR_UAV_MEC = 0;
double averageDelayPercent = 0;

int TotalPacket = 0;
double PacketLossTime = 0;
double SuccessedTime = 0;
double UAV_cal_capability = 1000000;
double MEC_cal_capability = 2000000;

std::mt19937 rnd_generator;

LAB_par parameter =
{
    0.000005 * 1,    // E1_car = 發送器所耗功率
    0.000005 * 1, // E2_car = 放大器所耗功率
    0.000005 * 1,  // E3_car = 接收器所耗功率

    0.000005 * 1.25,    // E1_UAV
    0.000005 * 1.25, // E2_UAV
    0.000005 * 1.25,  // E3_UAV

    0.000005 * 1.75,    // E1_MEC
    0.000005 * 1.75, // E2_MEC
    0.000005 * 1.75,  // E3_MEC

    1000 * 1,    // P_car = 處理任務所耗功率
    1000 * 1.5, // P_UAV
    1000 * 2, // P_MEC

    20000, // Energy_perMeter

    0.5,  // DelayRatio
    0.5,  // EnergyRatio

    0.8,  // DistanceRatio
    0.2   // ResourceRatio
};

Define_Module(veins::GHDSFCar);

task::task (int q)
{
    qos = q;
    //EV << "Time : " << (unsigned int)time(NULL) << endl;
    std::normal_distribution<double> normal_dis(50.0, 16.0);
    double rnd = normal_dis(rnd_generator);
    if(rnd < 0)
        rnd = 0;
    else if(rnd > 100)
        rnd = 100;
    int packet_rnd = rand() % 100;
    int MEC_rnd = rand() % 100;
    if(MEC_rnd < 70)
        must_send_MEC = true;
    else
        must_send_MEC = false;
    switch (q)
    {
        case 1:
            packet_size = 300 + (int)(700 * rnd / 100); // Priority 1 : packet size = 300 ~ 1000 Bytes
            delay_limit = (10.0 + 40.0 * rnd / 100) / 1000.0; // 10ms~50ms
            require_cpu = 20;
            require_memory = 15 + (int)(5 * rnd / 100);
            break;
        case 2:
            packet_size = 100 + (int)(700 * rnd / 100); // Priority 2 : packet size = 100 ~ 800 Bytes
            delay_limit = 100.0 / 1000.0; // 100ms
            require_cpu = 15;
            require_memory = 10 + (int)(5 * rnd / 100);
            break;
        case 3:
            packet_size = 9375 + (int)(28125 * rnd / 100); // Priority 3 : packet size = 9375 ~ 37500 Bytes
            switch (packet_rnd)
            {
                case 0 ... 40:
                packet_size = 9375 + (int)(6562 * rnd / 100); // Priority 3 : packet size = 9375 ~ 15897 Bytes（低負載）
                break;
                case 41 ... 80:
                packet_size = 15898 + (int)(6563 * rnd / 100); // Priority 3 : packet size = 15898 ~ 22500 Bytes（中負載）
                break;
                case 81 ... 100:
                packet_size = 22500 + (int)(15000 * rnd / 100); // Priority 3 : packet size = 22500 ~ 37500 Bytes (高負載，車輛無法自主運算，須交由UAV、MEC處理)
                break;
            }
            delay_limit = 150.0 / 1000.0; // 150ms
            require_cpu = 15;
            require_memory = 10 + (int)(5 * rnd / 100);
            break;
        case 4:
            packet_size = 1000 + (int)(5400 * rnd / 100); // Priority 4 : packet size = 1000 ~ 6400 Bytes
            delay_limit = 300.0 / 1000.0; // 300ms
            require_cpu = 10;
            require_memory = 5 + (int)(5 * rnd / 100);
            break;
    }
}

GHDSFCar::GHDSFCar() : node_resource(100,100) {} // 讓每個node都有自己的一個node_resource


void GHDSFCar::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        srand((unsigned int)time(NULL));
        rnd_generator = std::mt19937(rand());
        node_resource.followed = false;
    }
    else if(stage == 1)
    {
        Car_info carInfo = {curPosition, 0};
        Car_map.insert(std::make_pair(myId, carInfo));
        cMessage *taskMsg = new cMessage("generate_task");
        scheduleAt(simTime() + uniform(0.1 , 2.5), taskMsg);
        cMessage *refreshMsg = new cMessage("refresh_Coord");
        scheduleAt(simTime() + 0.1, refreshMsg);
        cMessage *UAV_mapTimer = new cMessage("UAV_map");
        scheduleAt(simTime() + 2, UAV_mapTimer);
    }
}

void GHDSFCar::onWSA(DemoServiceAdvertisment* wsa)
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

void GHDSFCar::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);
    //EV_INFO << myId << ": Receive a packet from: " << wsm->getSenderAddress() << " at time: " << wsm->getTimestamp()/* << " And the data: " << wsm->getDemoData() */<< " Delay = " << simTime() - wsm->getTimestamp();
    if(std::string(wsm->getName()).substr(0, 12) == "TaskSendBack")
    {
        int Back_FullPacketSize = std::stoi(std::string(wsm->getName()).substr(13));
        LAddress::L2Type Back_UAVID = wsm->getSenderAddress();
        EV << myId << ": Received the finished task part from UAV " << Back_UAVID << ", Full packet size = " << Back_FullPacketSize << endl;
        double dis_UAVtoCAR_x = curPosition.x - wsm->getSenderPosition().x;
        double dis_UAVtoCAR_y = curPosition.y - wsm->getSenderPosition().y;
        double dis_UAVtoCAR = dis_UAVtoCAR_x * dis_UAVtoCAR_x + dis_UAVtoCAR_y * dis_UAVtoCAR_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
        double energy_UAVtoCAR = (parameter.E1_UAV + parameter.E3_car + (parameter.E2_UAV * dis_UAVtoCAR)) * wsm->getByteLength();
        energyCommunication += energy_UAVtoCAR;
        EV << "(UAV->CAR)Energy in communication = " << energy_UAVtoCAR << " / energyCommunication = " << energyCommunication << endl;
        for(auto it = node_resource.waiting_tasks.begin(); it != node_resource.waiting_tasks.end();)
        {
            if (it->first.packet_size == Back_FullPacketSize)
            {
                it->second -= 2; // 因為是由UAV回傳回來的任務，將UAV的數字扣掉
                EV << myId << ": The returning packet from UAV is a part of packet " << it->first.packet_size << ", the remaining = " << it->second << endl;
                if(it->second == 0) // 扣到0代表任務運算完成
                {
                    if(simTime() > it->first.expire_time)
                    {
                        PacketLossTime++;
                        averageDelayPercent += 1.0;
                        EV << myId << ": Size = " << it->first.packet_size << " : packet loss!" << " / Packet Loss Time : " << PacketLossTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
                    }
                    else
                    {
                        SuccessedTime++;
                        Successed_UAV++;
                        averageDelayPercent += (1.0 - (it->first.expire_time - simTime().dbl()) / it->first.delay_limit);
                        EV << myId << ": Size = " << it->first.packet_size << " : handling success!" << " / Success Time : " << SuccessedTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
                        taskSize += it->first.packet_size;
                    }

                    it = node_resource.waiting_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                    continue;
                }
            }
            ++it;  // 如果當前元素不符合條件，則遞增迭代器
        }
    }
    else if(std::string(wsm->getName()).substr(0, 20) == "MEC_UAV_TaskSendBack")
    {
        int Back_FullPacketSize = std::stoi(std::string(wsm->getName()).substr(21));
        EV << myId << ": Received the finished task relayed from UAV " << wsm->getSenderAddress() << ", and the Full packet size = " << Back_FullPacketSize << endl;
        double dis_UAVtoCAR_x = curPosition.x - wsm->getSenderPosition().x;
        double dis_UAVtoCAR_y = curPosition.y - wsm->getSenderPosition().y;
        double dis_UAVtoCAR = dis_UAVtoCAR_x * dis_UAVtoCAR_x + dis_UAVtoCAR_y * dis_UAVtoCAR_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
        double energy_UAVtoCAR = (parameter.E1_UAV + parameter.E3_car + (parameter.E2_UAV * dis_UAVtoCAR)) * wsm->getByteLength();
        energyCommunication += energy_UAVtoCAR;
        EV << "(UAV->CAR)Energy in communication = " << energy_UAVtoCAR << " / energyCommunication = " << energyCommunication << endl;
        for(auto it = node_resource.waiting_tasks.begin(); it != node_resource.waiting_tasks.end();)
        {
            if (it->first.packet_size == Back_FullPacketSize)
            {
                it->second -= 3; // 因為是由MEC回傳回來的任務，將MEC的數字扣掉
                EV << myId << ": The returning packet from MEC->UAV is a part of packet " << it->first.packet_size << ", the remaining = " << it->second << endl;
                if(it->second == 0) // 扣到0代表任務運算完成
                {
                    if(simTime() > it->first.expire_time)
                    {
                        PacketLossTime++;
                        averageDelayPercent += 1.0;
                        EV << myId << ": Size = " << it->first.packet_size << " : packet loss!" << " / Packet Loss Time : " << PacketLossTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
                    }
                    else
                    {
                        SuccessedTime++;
                        Successed_MEC++;
                        averageDelayPercent += (1.0 - (it->first.expire_time - simTime().dbl()) / it->first.delay_limit);
                        EV << myId << ": Size = " << it->first.packet_size << " : handling success!" << " / Success Time : " << SuccessedTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
                        taskSize += it->first.packet_size;
                    }

                    it = node_resource.waiting_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                    continue;
                }
            }
            ++it;  // 如果當前元素不符合條件，則遞增迭代器
        }
    }
    else if(std::string(wsm->getName()).substr(0, 15) == "MECTaskSendBack") // 車輛收到MEC處理完成的任務
    {
        int Back_FullPacketSize = std::stoi(std::string(wsm->getName()).substr(16));
        EV << myId << ": Received the finished task from MEC " << wsm->getSenderAddress() << ", and the Full packet size = " << Back_FullPacketSize << endl;
        double dis_MECtoCAR_x = curPosition.x - wsm->getSenderPosition().x;
        double dis_MECtoCAR_y = curPosition.y - wsm->getSenderPosition().y;
        double dis_MECtoCAR = dis_MECtoCAR_x * dis_MECtoCAR_x + dis_MECtoCAR_y * dis_MECtoCAR_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
        double energy_MECtoCAR = (parameter.E1_MEC + parameter.E3_car + (parameter.E2_MEC * dis_MECtoCAR)) * wsm->getByteLength();
        energyCommunication += energy_MECtoCAR;
        EV << "(MEC->CAR)Energy in communication = " << energy_MECtoCAR << " / energyCommunication = " << energyCommunication << endl;
        for(auto it = node_resource.waiting_tasks.begin(); it != node_resource.waiting_tasks.end();)
        {
            if (it->first.packet_size == Back_FullPacketSize)
            {
                it->second -= 3; // 因為是由MEC回傳回來的任務，將MEC的數字扣掉
                EV << myId << ": The returning packet from MEC is a part of packet " << it->first.packet_size << ", the remaining = " << it->second << endl;
                if(it->second == 0) // 扣到0代表任務運算完成
                {
                    if(simTime() > it->first.expire_time)
                    {
                        PacketLossTime++;
                        averageDelayPercent += 1.0;
                        EV << myId << ": Size = " << it->first.packet_size << " : packet loss!" << " / Packet Loss Time : " << PacketLossTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
                    }
                    else
                    {
                        SuccessedTime++;
                        Successed_MEC++;
                        averageDelayPercent += (1.0 - (it->first.expire_time - simTime().dbl()) / it->first.delay_limit);
                        EV << myId << ": Size = " << it->first.packet_size << " : handling success!" << " / Success Time : " << SuccessedTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
                        taskSize += it->first.packet_size;
                    }

                    it = node_resource.waiting_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                    continue;
                }
            }
            ++it;  // 如果當前元素不符合條件，則遞增迭代器
        }
    }

}

void GHDSFCar::onBSM(DemoSafetyMessage* wsm)
{

}

void GHDSFCar::onBM(BeaconMessage* bsm)
{
    if(!strcmp(bsm->getName(), "MEC_beacon"))
    {
        double tmp_delay = simTime().dbl() - bsm->getTimestamp().dbl();
        double tmp_distance_x = bsm->getSenderPos().x - curPosition.x;
        double tmp_distance_y = bsm->getSenderPos().y - curPosition.y;
        double tmp_distance = sqrt(tmp_distance_x * tmp_distance_x + tmp_distance_y * tmp_distance_y);
        if(Nearest_MEC == bsm->getSenderAddress()) // 收到同一個MEC的beacon，更新資料
        {
            Delay_to_MEC = tmp_delay;
            Distance_to_MEC = tmp_distance;
            MEC_generate_time = simTime().dbl();
            MEC_remain_cpu = bsm->getRemain_cpu();
            MEC_remain_mem = bsm->getRemain_mem();
            EV << myId << ": The MEC " << Nearest_MEC << " refresh it's delay!" << endl;
        }
        else
        {
            if(tmp_delay <= Delay_to_MEC) // 若收到其他MEC的beacon，比較確認哪一個是delay最短的
            {
                Nearest_MEC = bsm->getSenderAddress();
                Delay_to_MEC = tmp_delay;
                Distance_to_MEC = tmp_distance;
                MEC_generate_time = simTime().dbl();
                MEC_remain_cpu = bsm->getRemain_cpu();
                MEC_remain_mem = bsm->getRemain_mem();
                EV << myId << ": I find another MEC " << Nearest_MEC << ", and it's delay is less than before!" << endl;
            }
        }
        cMessage *MECTimer = new cMessage("check_MEC_beacon");
        scheduleAt(simTime() + 0.5, MECTimer);
    }
    else
    {
        EV_INFO << myId << ": I Receive a beacon from UAV " << bsm->getSenderAddress() << " / it's name = " << bsm->getName() << " and it's position : " << bsm->getSenderPos() << " / speed : " << bsm->getSenderSpeed() << " / direction : " << bsm->getSenderDirection() << " / Delay to MEC : " << bsm->getDelay_to_MEC() << " / Distance to MEC : " << bsm->getDistance_to_MEC() << endl;
        UAV_map[bsm->getSenderAddress()] = {simTime().dbl(), bsm->getSenderPos(), (simTime() - bsm->getTimestamp()).dbl(), bsm->getDelay_to_MEC(), bsm->getDistance_to_MEC(), bsm->getRemain_cpu(), bsm->getRemain_mem()};
        /*if(!node_resource.followed && !bsm->getFollowing())
        {
            EV << myId << ": UAV " << bsm->getSenderAddress() << " want to follow me!" << endl;

            node_resource.tmp_Position = curPosition;
            node_resource.tmp_time = simTime().dbl();
            //EV << myId << ": tmp_Position =  " << node_resource.tmp_Position << " / tmp_time = " << node_resource.tmp_time << endl;
            BeaconMessage *bm = new BeaconMessage("FollowMe");
            populateWSM(bm);
            bm->setSenderAddress(myId);
            bm->setByteLength(300);
            bm->setTimestamp(simTime());
            bm->setRecipientAddress(bsm->getSenderAddress());
            bm->setSenderPos(curPosition);
            sendDown(bm);
        }
        if(!node_resource.followed && !strcmp(bsm->getName(), "FollowMe_ACK"))
        {
            node_resource.followed = true;
            node_resource.followed_car = bsm->getSenderAddress();
            EV << myId << ": I'm followed by UAV " << bsm->getSenderAddress() << "!" << endl;
            cMessage *FollowMsg = new cMessage("Follow");
            scheduleAt(simTime() + 0.05, FollowMsg);
        }*/
    }
}

void GHDSFCar::handleSelfMsg(cMessage* msg)
{
    if(!strcmp(msg->getName(), "generate_task"))
    {
        int numtasks = intuniform(3,8);
        TotalPacket += numtasks;
        Car_map[myId].Num_Task += numtasks;
        for(int i=0; i<numtasks; i++)
        {
            int task_p =  intuniform(1,100);
            if (task_p >= 1 && task_p <= 20) // 使用if-else來判斷範圍
            {
                task t(1);
                t.id = myId;
                t.start_time = simTime().dbl();
                t.expire_time = t.start_time + t.delay_limit;
                node_resource.pending_tasks.push(t);
            }
            else if (task_p >= 21 && task_p <= 45)
            {
                task t(2);
                t.id = myId;
                t.start_time = simTime().dbl();
                t.expire_time = t.start_time + t.delay_limit;
                node_resource.pending_tasks.push(t);
            }
            else if (task_p >= 46 && task_p <= 70)
            {
                task t(3);
                t.id = myId;
                t.start_time = simTime().dbl();
                t.expire_time = t.start_time + t.delay_limit;
                node_resource.pending_tasks.push(t);
            }
            else if (task_p >= 71 && task_p <= 100)
            {
                task t(4);
                t.id = myId;
                t.start_time = simTime().dbl();
                t.expire_time = t.start_time + t.delay_limit;
                node_resource.pending_tasks.push(t);
            }
            EV << "I'm " << myId << " and I generate a task: QoS = " << node_resource.pending_tasks.back().qos << " , Delay_limit : " << node_resource.pending_tasks.back().delay_limit <<  " , start_time = " << node_resource.pending_tasks.back().start_time << " , expire_time = " << node_resource.pending_tasks.back().expire_time << " , size = " << node_resource.pending_tasks.back().packet_size << " / must MEC = " << node_resource.pending_tasks.back().must_send_MEC << endl;
        }
        //CoCaCoTaskOffloading();
        dispatchTaskConsiderEnergy();
        delete msg;
        cMessage *taskMsg = new cMessage("generate_task");
        scheduleAt(simTime() + uniform(0.1 , 2.5), taskMsg);
    }
    else if(!strcmp(msg->getName(), "refresh_Coord"))
    {
        delete msg;
        Car_map[myId].Position = curPosition;
        //EV << "I'm " << myId << " and my remained cpu = " << node_resource.remain_cpu << ", remained memory = " << node_resource.remain_memory << endl;
        cMessage *refreshMsg = new cMessage("refresh_Coord");
        scheduleAt(simTime() + 0.1, refreshMsg);
    }
    else if(!strcmp(msg->getName(), "UAV_map"))
    {
        if(!UAV_map.empty())
        {
            for(std::map<LAddress::L2Type, UAV_MapData>::iterator it = UAV_map.begin(); it != UAV_map.end();)
            {
                if(simTime().dbl() >= (*it).second.generate_time + 2.0)
                {
                    EV << "I'm " << myId << ", UAV " << (*it).first << " is no longer in my area!";
                    it = UAV_map.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }
        delete msg;
        cMessage *UAV_mapTimer = new cMessage("UAV_map");
        scheduleAt(simTime() + 2, UAV_mapTimer);
    }
    else if(std::string(msg->getName()).substr(0, 7) == "myTask_")
    {
        int finish_size = std::stoi(std::string(msg->getName()).substr(7));
        EV << myId << ": Handling finish. Size = " << finish_size;
        delete msg;
        for (auto it = node_resource.handling_tasks.begin(); it != node_resource.handling_tasks.end();)
        {
            if (it->packet_size == finish_size)
            {
                node_resource.remain_cpu += it->require_cpu;
                node_resource.remain_memory += it->require_memory;
                for (auto it2 = node_resource.waiting_tasks.begin(); it2 != node_resource.waiting_tasks.end();)
                {
                    if (it2->first.packet_size == it->packet_size)
                    {
                        it2->second -= 1; // 扣掉車輛自己運算的部分
                        EV << " / remain part = " << it2->second << endl;
                        if(it2->second == 0)
                        {
                            if(simTime() > it2->first.expire_time)
                            {
                                PacketLossTime++;
                                averageDelayPercent += 1.0;
                                EV << myId << ": Full packet Size = " << it2->first.packet_size << " : packet loss!" << " Packet Loss Time : " << PacketLossTime << endl;
                                EV << "The expire time : " << it2->first.expire_time << ", and now is : " << simTime() << endl;
                            }
                            else
                            {
                                SuccessedTime++;
                                Successed_Car++;
                                averageDelayPercent += (1.0 - (it2->first.expire_time - simTime().dbl()) / it2->first.delay_limit);
                                EV << myId << ": Full packet Size = " << it2->first.packet_size << " : handling success!" << " Success Time : " << SuccessedTime << endl;
                                EV << "The expire time : " << it2->first.expire_time << ", and now is : " << simTime() << endl;
                                taskSize += it2->first.packet_size;
                            }
                            it2 = node_resource.waiting_tasks.erase(it2);  // 刪除符合條件的元素並更新迭代器
                        }
                        break;
                    }
                    else
                    {
                        ++it2;  // 如果當前元素不符合條件，則遞增迭代器
                    }
                }
                it = node_resource.handling_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                break;
            }
            else
            {
                ++it;  // 如果當前元素不符合條件，則遞增迭代器
            }
        }
        if(!node_resource.queuing_tasks.empty())
        {
            handleQueuingTask();
        }
        if(!node_resource.pending_tasks.empty())
        {

            //CoCaCoTaskOffloading();
            dispatchTaskConsiderEnergy();
        }

    }
    else if(!strcmp(msg->getName(), "Follow"))
    {
        delete msg;
        // 由兩個time slot車輛的座標差計算車輛的移動速度(mps)
        double sp = sqrt((curPosition.x - node_resource.tmp_Position.x) * (curPosition.x - node_resource.tmp_Position.x)
                       + (curPosition.y - node_resource.tmp_Position.y) * (curPosition.y - node_resource.tmp_Position.y))
                       / (simTime().dbl() - node_resource.tmp_time);
        EV << myId << ": Please Keep Follow Me! curPosition = " << curPosition << " / sp = " << sp << endl;
        //EV << "curPosition.x - node_resource.tmp_Position.x = " << (curPosition.x - node_resource.tmp_Position.x) << endl;
        //EV << "curPosition.y - node_resource.tmp_Position.y = " << (curPosition.y - node_resource.tmp_Position.y) << endl;
        //EV << "node_resource.tmp_time = " << node_resource.tmp_time << endl;
        //EV << "simTime().dbl() = " << simTime().dbl() << endl;
        Coord MovingDiff = curPosition - node_resource.tmp_Position; // 車輛前一個slot與這一次slot間的移動座標
        node_resource.tmp_Position = curPosition;
        node_resource.tmp_time = simTime().dbl();
        BeaconMessage *bm = new BeaconMessage("KeepFollow");
        populateWSM(bm);
        bm->setSenderAddress(myId);
        bm->setByteLength(300);
        bm->setTimestamp(simTime());
        bm->setRecipientAddress(node_resource.followed_car);
        bm->setSenderPos(curPosition);
        bm->setCurrentSpeed(sp);
        bm->setSenderSpeed(MovingDiff);
        sendDown(bm);
        cMessage *FollowMsg = new cMessage("Follow");
        scheduleAt(simTime() + 0.1, FollowMsg);
    }
    else if(!strcmp(msg->getName(), "check_MEC_beacon"))
    {
        if(MEC_generate_time + 0.5 <= simTime().dbl())
        {
            EV << myId << " : The MEC " << Nearest_MEC << " is lost connect!" << endl;
            Nearest_MEC = -1;
            Delay_to_MEC = DBL_MAX;
        }
        delete msg;
    }
}

struct ScheduledMessage
{
    cMessage* msg;
    double cal_time;
};

void GHDSFCar::clearExpiredTask() // 在node離開地圖前，刪除已過期的任務
{
    for(auto it = node_resource.waiting_tasks.begin(); it != node_resource.waiting_tasks.end();)
    {
        if (it->first.expire_time < simTime().dbl())
        {
            PacketLossTime++;
            CantFindOffload++;
            averageDelayPercent += 1.0;
            EV << myId << " : My Task is expired, packet loss! Size = " << it->first.packet_size << " / Packet loss time : " << PacketLossTime << endl;
            it = node_resource.waiting_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
            continue;
        }
        ++it;  // 如果當前元素不符合條件，則遞增迭代器
    }
}

void GHDSFCar::CoCaCoTaskOffloading()
{
    std::vector<ScheduledMessage> selfscheduledMessages;
    clearExpiredTask();
    int loop_time = node_resource.pending_tasks.size();
    for(int i=0; i<loop_time; i++)
    {
        task top_task = node_resource.pending_tasks.front();
        node_resource.pending_tasks.pop();
        EV << myId << " : expire time =  " << top_task.expire_time << ", Nearest_MEC = " << Nearest_MEC << endl;
        if(top_task.expire_time > simTime().dbl())
        {
            if(!UAV_map.empty())
            {
                double minDelay = std::numeric_limits<double>::max(); // 初始化為最大值
                LAddress::L2Type minDelayUAV;
                for(const auto& UAV_in_my_Area : UAV_map)
                {
                    if(UAV_in_my_Area.second.Delay < minDelay)
                    {
                        minDelay = UAV_in_my_Area.second.Delay;
                        minDelayUAV = UAV_in_my_Area.first;
                    }
                }
                // 現在，minDelayUAV 是 Delay 最小的 UAV 的地址
                UAV_MapData minDelayUAV_in_my_Area = UAV_map[minDelayUAV]; // 找到delay最小的UAV
                if(minDelayUAV_in_my_Area.Delay_to_MEC != -1) // Delay最小的UAV有和MEC連線 => 任務分為車輛、UAV、MEC計算
                {
                    CAR_UAV_MEC++;
                    // 注意MEC任務傳輸路徑：車輛->UAV->MEC
                    EV << myId << ": handle the task by myself and UAV and MEC(CAR->UAV->MEC)! " << " Packet size = " << top_task.packet_size << " / handle size by car = " << top_task.packet_size / 3 << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    node_resource.waiting_tasks.push_back({top_task, 6}); // 將任務放進等待任務欄中，6代表車輛(1)運算+UAV(2)運算+MEC(3)運算
                    node_resource.queuing_tasks.push({top_task, top_task.packet_size / 3}); // 將車輛需要自己算的部分packet size放入
                    EV << myId << ": Send a task to UAV " << minDelayUAV << ", a part handle by UAV and another part need to relay to MEC, full packet size = " << top_task.packet_size << ", handle size = " << top_task.packet_size / 3 << endl;
                    std::string s1 = "UAV_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                    TraCIDemo11pMessage *task_UAV_MEC_msg = new TraCIDemo11pMessage;
                    populateWSM(task_UAV_MEC_msg);
                    task_UAV_MEC_msg->setByteLength((top_task.packet_size / 3) * 2);
                    task_UAV_MEC_msg->setSenderAddress(myId);
                    task_UAV_MEC_msg->setRecipientAddress(minDelayUAV);
                    task_UAV_MEC_msg->setName(s1.c_str());
                    sendDown(task_UAV_MEC_msg);

                }
                else // Delay最小的UAV沒有和MEC連線
                {
                    if(Nearest_MEC != -1) // UAV沒和MEC連線，車輛自己有和MEC連線 => 任務分為車輛、UAV、MEC計算
                    {
                        CAR_UAV_MEC++;
                        // 注意MEC任務傳輸路徑：車輛->MEC
                        EV << myId << ": handle the task by myself and UAV and MEC(CAR->MEC)! " << " Packet size = " << top_task.packet_size << " / handle size by car = " << top_task.packet_size / 3 << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                        node_resource.waiting_tasks.push_back({top_task, 6}); // 將任務放進等待任務欄中，6代表車輛(1)運算+UAV(2)運算+MEC(3)運算
                        node_resource.queuing_tasks.push({top_task, top_task.packet_size / 3}); // 將車輛需要自己算的部分packet size放入
                        EV << myId << ": Send a task to UAV " << minDelayUAV << ", full packet size = " << top_task.packet_size << ", handle size = " << top_task.packet_size / 3 << endl;
                        std::string s = "UAV_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                        TraCIDemo11pMessage *task_UAV_msg = new TraCIDemo11pMessage;
                        populateWSM(task_UAV_msg);
                        task_UAV_msg->setByteLength(top_task.packet_size / 3);
                        task_UAV_msg->setSenderAddress(myId);
                        task_UAV_msg->setRecipientAddress(minDelayUAV);
                        task_UAV_msg->setName(s.c_str());
                        sendDown(task_UAV_msg);
                        EV << myId << ": Send a task to MEC " << Nearest_MEC << ", full packet size = " << top_task.packet_size << ", handle size = " << top_task.packet_size / 3 << endl;
                        std::string s1 = "Car_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                        TraCIDemo11pMessage *task_CartoMEC_msg = new TraCIDemo11pMessage;
                        populateWSM(task_CartoMEC_msg);
                        task_CartoMEC_msg->setByteLength(top_task.packet_size / 3);
                        task_CartoMEC_msg->setSenderAddress(myId);
                        task_CartoMEC_msg->setRecipientAddress(Nearest_MEC);
                        task_CartoMEC_msg->setName(s1.c_str());
                        sendDown(task_CartoMEC_msg);
                    }
                    else // UAV沒和MEC連線，車輛自己也沒和MEC連線 => 任務分為車輛和UAV計算
                    {
                        CAR_UAV++;
                        EV << myId << ": handle the task by myself and UAV! " << " Packet size = " << top_task.packet_size << " / handle size by car = " << top_task.packet_size / 2 << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                        node_resource.waiting_tasks.push_back({top_task, 3}); // 將任務放進等待任務欄中，3代表車輛(1)運算+UAV(2)運算
                        node_resource.queuing_tasks.push({top_task, top_task.packet_size / 2}); // 將車輛需要自己算的部分packet size放入
                        EV << myId << ": Send a task to UAV " << minDelayUAV << ", full packet size = " << top_task.packet_size << ", handle size = " << top_task.packet_size / 2 << endl;
                        std::string s = "UAV_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                        TraCIDemo11pMessage *task_UAV_msg = new TraCIDemo11pMessage;
                        populateWSM(task_UAV_msg);
                        task_UAV_msg->setByteLength(top_task.packet_size / 2);
                        task_UAV_msg->setSenderAddress(myId);
                        task_UAV_msg->setRecipientAddress(minDelayUAV);
                        task_UAV_msg->setName(s.c_str());
                        sendDown(task_UAV_msg);
                    }
                }
            }
            else
            {
                if(Nearest_MEC != -1) // 車輛沒連線UAV、但是有連線MEC => 任務分為車輛和MEC計算
                {
                    CAR_MEC++;
                    EV << myId << ": handle the task by myself and MEC! " << " Packet size = " << top_task.packet_size << " / handle size by car = " << top_task.packet_size / 2 << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    node_resource.waiting_tasks.push_back({top_task, 4}); // 將任務放進等待任務欄中，4代表車輛運算+MEC運算
                    node_resource.queuing_tasks.push({top_task, top_task.packet_size / 2}); // 將車輛需要自己算的部分packet size放入
                    EV << myId << ": Send a task to MEC " << Nearest_MEC << ", full packet size = " << top_task.packet_size << ", handle size = " << top_task.packet_size / 2 << endl;
                    std::string s = "Car_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                    TraCIDemo11pMessage *task_CartoMEC_msg = new TraCIDemo11pMessage;
                    populateWSM(task_CartoMEC_msg);
                    task_CartoMEC_msg->setByteLength(top_task.packet_size / 2);
                    task_CartoMEC_msg->setSenderAddress(myId);
                    task_CartoMEC_msg->setRecipientAddress(Nearest_MEC);
                    task_CartoMEC_msg->setName(s.c_str());
                    sendDown(task_CartoMEC_msg);
                }
                else // 車輛沒連線UAV也沒連線MEC => 車輛自行運算
                {

                    CAR_SELF++;
                    EV << myId << ": handle the task by myself!" << " Packet size = " << top_task.packet_size << " / handle size by car = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    node_resource.waiting_tasks.push_back({top_task, 1}); // 將任務放進等待任務欄中，1代表只有車輛運算
                    node_resource.queuing_tasks.push({top_task, top_task.packet_size}); // 將車輛需要自己算的部分packet size放入
                }
            }
        }
        else
        {
            PacketLossTime++;
            CantFindOffload++;
            averageDelayPercent += 1.0;
            EV << myId << " : My Task is expired, packet loss! Size = " << top_task.packet_size << " / Packet loss time : " << PacketLossTime << endl;
        }
    }
    handleQueuingTask();
}

void GHDSFCar::handleQueuingTask()
{
    std::vector<ScheduledMessage> selfscheduledMessages;
    int loop_time = node_resource.queuing_tasks.size();
    for(int i=0; i<loop_time; i++)
    {
        task top_task = node_resource.queuing_tasks.front().first;
        int top_task_size = node_resource.queuing_tasks.front().second;
        node_resource.queuing_tasks.pop();
        if(node_resource.remain_cpu >= top_task.require_cpu && node_resource.remain_memory >= top_task.require_memory)
        {
            double cal_time = top_task_size / (node_resource.cal_capability * top_task.require_cpu / 100.0);
            node_resource.remain_cpu -= top_task.require_cpu;
            node_resource.remain_memory -= top_task.require_memory;
            node_resource.handling_tasks.push_back(top_task);
            std::string s = "myTask_"+ std::to_string(top_task.packet_size);
            EV << myId << ": handling the task! Handling time = " << cal_time << " / packet size = " << top_task.packet_size << " / handle size = " << top_task_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
            energyComputing += cal_time * parameter.P_car;
            EV << myId << ": EnergyComsumption = " << cal_time * parameter.P_car << " / energyComputing = " << energyComputing << endl;
            cMessage *Task_handlingTimer = new cMessage(s.c_str());
            selfscheduledMessages.push_back({Task_handlingTimer, cal_time});
        }
        else // 資源不足自己運算只能先放回queuing_task中等有資源時處理
        {
            node_resource.queuing_tasks.push({top_task, top_task_size});
            continue;
        }
        if(node_resource.remain_cpu == 0 || node_resource.remain_memory == 0)
            break;
    }
    for(const auto &selfscheduledMessage : selfscheduledMessages) // 開始自己運算的任務
    {
        scheduleAt(simTime() + selfscheduledMessage.cal_time, selfscheduledMessage.msg);
    }
}


void GHDSFCar::dispatchTaskConsiderEnergy()
{
    std::vector<ScheduledMessage> selfscheduledMessages;
    int loop_time = node_resource.pending_tasks.size();
    for(int i=0; i<loop_time; i++)
    {
        task top_task = node_resource.pending_tasks.front();
        node_resource.pending_tasks.pop();
        if(top_task.expire_time > simTime().dbl()) // 任務尚未過期，嘗試決定任務要怎麼offload
        {
            double delay_car = -1;
            double energy_car = -1;
            double cost_car = DBL_MAX;
            double delay_UAV = -1;
            double energy_UAV = -1;
            double cost_UAV = DBL_MAX;
            LAddress::L2Type candidate_UAV = -1;
            double delay_UAV_MEC = -1;
            double energy_UAV_MEC = -1;
            double cost_UAV_MEC = DBL_MAX;
            double delay_normalize = -DBL_MAX; // 用來將delay正規化
            double energy_normalize = -DBL_MAX; // 用來將energy正規化
            LAddress::L2Type candidate_UAV_MEC = -1; // candidate_UAV_MEC指的是負責relay給MEC的UAV的id(因為要先傳給該該UAV)
            if(node_resource.remain_cpu >= top_task.require_cpu && node_resource.remain_memory >= top_task.require_memory) // 車輛自行運算
            {
                double cal_time = top_task.packet_size / (node_resource.cal_capability * top_task.require_cpu / 100.0);
                if(cal_time <= (top_task.expire_time - simTime().dbl())) // 來得及算完才將其列考慮之一
                {
                    delay_car = cal_time;
                    //energy_car = parameter.P_car * top_task.packet_size;
                    energy_car = parameter.P_car * cal_time;
                    cost_car = parameter.DelayRatio * delay_car + parameter.EnergyRatio * energy_car;
                }
                if(top_task.must_send_MEC)
                    cost_car = DBL_MAX;
            }
            if(!UAV_map.empty())
            {
                for(const auto UAV_in_my_Area : UAV_map)
                {
                    if(UAV_in_my_Area.second.Delay_to_MEC != -1) // 若該UAV有和MEC連線，需額外計算其成本(車輛->UAV->MEC)
                    {
                        double dis_CARtoUAV_x = curPosition.x - UAV_in_my_Area.second.position.x;
                        double dis_CARtoUAV_y = curPosition.y - UAV_in_my_Area.second.position.y;
                        double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                        double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + parameter.E1_UAV + parameter.E3_car + (parameter.E2_car + parameter.E2_UAV) * dis_CARtoUAV) * top_task.packet_size; // 傳去和傳回來都一起先算
                        double energy_UAVtoMEC = (parameter.E1_UAV + parameter.E3_MEC + parameter.E1_MEC + parameter.E3_UAV + (parameter.E2_UAV + parameter.E2_MEC) * UAV_in_my_Area.second.Distance_to_MEC * UAV_in_my_Area.second.Distance_to_MEC) * top_task.packet_size; // 傳去和傳回來都一起先算
                        //double tmp_energy_UAV_MEC = parameter.P_MEC * top_task.packet_size + energy_CARtoUAV + energy_UAVtoMEC;
                        double tmp_energy_UAV_MEC = (parameter.P_MEC * top_task.packet_size / (MEC_cal_capability * top_task.require_cpu / 100.0)) + energy_CARtoUAV + energy_UAVtoMEC;
                        // MEC 總處理時間 = 車輛到UAV傳輸時間(來回) + UAV到MEC傳輸時間(來回) + MEC處理時間
                        // Delay和Delay_to_MEC是beacon(packet_size=300)的傳輸時間，在這邊算大略的傳輸耗時直接根據packet_size等比例放大計算
                        double tmp_delay_UAV_MEC = 2 * ((UAV_in_my_Area.second.Delay + UAV_in_my_Area.second.Delay_to_MEC) / 300 * top_task.packet_size) + top_task.packet_size / (MEC_cal_capability * top_task.require_cpu / 100.0);
                        double tmp_cost_UAV_MEC = parameter.DelayRatio * tmp_delay_UAV_MEC + parameter.EnergyRatio * tmp_energy_UAV_MEC;
                        if(tmp_delay_UAV_MEC > (top_task.expire_time - simTime().dbl())) // 來不及算完就把成本當作DBL_MAX
                            tmp_cost_UAV_MEC = DBL_MAX;
                        if(tmp_cost_UAV_MEC < cost_UAV_MEC)
                        {
                            cost_UAV_MEC = tmp_cost_UAV_MEC;
                            energy_UAV_MEC = tmp_energy_UAV_MEC;
                            delay_UAV_MEC = tmp_delay_UAV_MEC;
                            candidate_UAV_MEC = UAV_in_my_Area.first; // candidate_UAV_MEC指的是負責relay給MEC的UAV的id(因為要先傳給該該UAV)
                        }
                    }
                    // 計算車輛offload到該UAV的成本(車輛->UAV)
                    if(top_task.must_send_MEC == false)
                    {
                        if(UAV_in_my_Area.second.remain_cpu >= top_task.require_cpu && UAV_in_my_Area.second.remain_memory >= top_task.require_memory)
                        {
                            double dis_CARtoUAV_x = curPosition.x - UAV_in_my_Area.second.position.x;
                            double dis_CARtoUAV_y = curPosition.y - UAV_in_my_Area.second.position.y;
                            double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                            double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + parameter.E1_UAV + parameter.E3_car + (parameter.E2_car + parameter.E2_UAV) * dis_CARtoUAV) * top_task.packet_size; // 傳去和傳回來都一起先算(這邊不包含運算耗能)
                            //double tmp_energy_UAV = parameter.P_UAV * top_task.packet_size + energy_CARtoUAV;
                            double tmp_energy_UAV = (parameter.P_UAV * top_task.packet_size / (UAV_cal_capability * top_task.require_cpu / 100.0)) + energy_CARtoUAV;
                            // UAV 總處理時間 = 車輛到UAV傳輸時間(來回) + UAV處理時間
                            // Delay是beacon(packet_size=300)(UAV->車輛)的傳輸時間，在這邊算大略的傳輸耗時直接根據packet_size等比例放大計算
                            double tmp_delay_UAV = 2 * (UAV_in_my_Area.second.Delay / 300 * top_task.packet_size) + top_task.packet_size / (UAV_cal_capability * top_task.require_cpu / 100.0);
                            double tmp_cost_UAV = parameter.DelayRatio * tmp_delay_UAV + parameter.EnergyRatio * tmp_energy_UAV;
                            if(tmp_delay_UAV > (top_task.expire_time - simTime().dbl())) // 來不及算完就把成本當作DBL_MAX
                                tmp_cost_UAV = DBL_MAX;
                            if(tmp_cost_UAV < cost_UAV)
                            {
                                cost_UAV = tmp_cost_UAV;
                                energy_UAV = tmp_energy_UAV;
                                delay_UAV = tmp_delay_UAV;
                                candidate_UAV = UAV_in_my_Area.first;
                            }
                        }
                    }
                }
            }
            // 正規化，先取得最大的energy和delay，接著再重新計算一次正規化過後的cost
            if (cost_car != DBL_MAX)
            {
                energy_normalize = std::max(energy_normalize, energy_car);
                delay_normalize = std::max(delay_normalize, delay_car);
            }
            if (cost_UAV != DBL_MAX)
            {
                energy_normalize = std::max(energy_normalize, energy_UAV);
                delay_normalize = std::max(delay_normalize, delay_UAV);
            }
            if (cost_UAV_MEC != DBL_MAX)
            {
                energy_normalize = std::max(energy_normalize, energy_UAV_MEC);
                delay_normalize = std::max(delay_normalize, delay_UAV_MEC);
            }

            EV << "delay_car = " << delay_car << " / energy_car = " << energy_car << " / cost_car = " << cost_car << endl;
            EV << "delay_UAV = " << delay_UAV << " / energy_UAV = " << energy_UAV << " / cost_UAV = " << cost_UAV << endl;
            EV << "delay_UAV_MEC = " << delay_UAV_MEC << " / energy_UAV_MEC = " << energy_UAV_MEC << " / cost_UAV_MEC = " << cost_UAV_MEC << endl;
            EV << "delay_normalize = " << delay_normalize << " / energy_normalize = " << energy_normalize << endl;
            EV << "Must send to MEC = " << top_task.must_send_MEC << endl;
            // 正規化，重新計算一次正規化過後的cost
            if(cost_car != DBL_MAX)
            {
                cost_car = parameter.DelayRatio * (delay_car / delay_normalize) + parameter.EnergyRatio * (energy_car / energy_normalize);
            }
            if(cost_UAV != DBL_MAX)
            {
                cost_UAV = parameter.DelayRatio * (delay_UAV / delay_normalize) + parameter.EnergyRatio * (energy_UAV / energy_normalize);
            }
            if(cost_UAV_MEC != DBL_MAX)
            {
                cost_UAV_MEC = parameter.DelayRatio * (delay_UAV_MEC / delay_normalize) + parameter.EnergyRatio * (energy_UAV_MEC / energy_normalize);
            }

            if(cost_car != DBL_MAX) // 車輛的成本不為DBL_MAX代表車輛來得及自行運算(所以cost被更新)，比較時要四種方式一起比
            {
                double min_cost = std::min({cost_car, cost_UAV, cost_UAV_MEC}); // 就算沒和UAV連線，也是選cost_car讓車輛自己算
                if(min_cost == cost_car) // 車輛自行運算成本最低
                {
                    CAR_SELF++;
                    double cal_time = top_task.packet_size / (node_resource.cal_capability * top_task.require_cpu / 100.0);
                    node_resource.remain_cpu -= top_task.require_cpu;
                    node_resource.remain_memory -= top_task.require_memory;
                    node_resource.handling_tasks.push_back(top_task);
                    node_resource.waiting_tasks.push_back({top_task, 1});
                    std::string s = "myTask_"+ std::to_string(top_task.packet_size);
                    EV << myId << ": handling the task! Handling time = " << cal_time << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_UAV_MEC = " << cost_UAV_MEC << endl;
                    energyComputing += cal_time * parameter.P_car;
                    EV << myId << ": EnergyComsumption = " << cal_time * parameter.P_car << " / energyComputing = " << energyComputing << endl;
                    cMessage *Task_handlingTimer = new cMessage(s.c_str());
                    selfscheduledMessages.push_back({Task_handlingTimer, cal_time});

                }
                else if(min_cost == cost_UAV) // 車輛傳給UAV運算成本最低
                {
                    CAR_UAV++;
                    EV << myId << ": I find UAV " << candidate_UAV << " to handle my task!"  << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_UAV_MEC = " << cost_UAV_MEC << endl;
                    double dis_CARtoUAV_x = curPosition.x - UAV_map[candidate_UAV].position.x;
                    double dis_CARtoUAV_y = curPosition.y - UAV_map[candidate_UAV].position.y;
                    double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                    double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + (parameter.E2_car * dis_CARtoUAV)) * top_task.packet_size;
                    energyCommunication += energy_CARtoUAV;
                    EV << "(CAR->UAV)Energy in communication = " << energy_CARtoUAV << " / energyCommunication = " << energyCommunication << endl;
                    std::string s = "UAV_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                    TraCIDemo11pMessage *task_UAV_msg = new TraCIDemo11pMessage;
                    populateWSM(task_UAV_msg);
                    task_UAV_msg->setByteLength(top_task.packet_size);
                    task_UAV_msg->setSenderAddress(myId);
                    task_UAV_msg->setRecipientAddress(candidate_UAV);
                    task_UAV_msg->setName(s.c_str());
                    sendDown(task_UAV_msg);
                    node_resource.waiting_tasks.push_back({top_task, 2});

                }
                else if(min_cost == cost_UAV_MEC) // 車輛傳給UAV再給MEC運算成本最低
                {
                    CAR_UAV_MEC++;
                    // candidate_UAV_MEC指的是負責relay給MEC的UAV的id(因為要先傳給該UAV)
                    EV << myId << " : I find UAV " << candidate_UAV_MEC << " and it is connecting to a Edge Server!" << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_UAV_MEC = " << cost_UAV_MEC << endl;
                    double dis_CARtoUAV_x = curPosition.x - UAV_map[candidate_UAV_MEC].position.x;
                    double dis_CARtoUAV_y = curPosition.y - UAV_map[candidate_UAV_MEC].position.y;
                    double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                    double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + (parameter.E2_car * dis_CARtoUAV)) * top_task.packet_size;
                    energyCommunication += energy_CARtoUAV;
                    EV << "(CAR->UAV)Energy in communication = " << energy_CARtoUAV << " / energyCommunication = " << energyCommunication << endl;
                    std::string s = "UAV_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                    TraCIDemo11pMessage *task_MEC_msg = new TraCIDemo11pMessage;
                    populateWSM(task_MEC_msg);
                    task_MEC_msg->setByteLength(top_task.packet_size);
                    task_MEC_msg->setSenderAddress(myId);
                    task_MEC_msg->setRecipientAddress(candidate_UAV_MEC);
                    task_MEC_msg->setName(s.c_str());
                    sendDown(task_MEC_msg);
                    node_resource.waiting_tasks.push_back({top_task, 3});
                }
            }
            else // 車輛的成本為DBL_MAX代表車輛沒辦法自行運算(cost_car沒被更新)，比較時只比UAV和MEC的cost，但如果三者都是DBL_MAX(沒和UAV連線，這個任務就只能先放著)
            {
                if(cost_UAV == DBL_MAX && cost_UAV_MEC == DBL_MAX)
                {
                    EV << myId << ": I can't handle this task and no one can help me!" << endl;
                    node_resource.pending_tasks.push(top_task);
                    continue;
                }
                else
                {
                    double min_cost = std::min({cost_UAV, cost_UAV_MEC});
                    if(min_cost == cost_UAV) // 車輛傳給UAV運算成本最低
                    {
                        CAR_UAV++;
                        EV << myId << ": I find UAV " << candidate_UAV << " to handle my task!"  << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                        EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_UAV_MEC = " << cost_UAV_MEC << endl;
                        double dis_CARtoUAV_x = curPosition.x - UAV_map[candidate_UAV].position.x;
                        double dis_CARtoUAV_y = curPosition.y - UAV_map[candidate_UAV].position.y;
                        double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                        double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + (parameter.E2_car * dis_CARtoUAV)) * top_task.packet_size;
                        energyCommunication += energy_CARtoUAV;
                        EV << "(CAR->UAV)Energy in communication = " << energy_CARtoUAV << " / energyCommunication = " << energyCommunication << endl;
                        std::string s = "UAV_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                        TraCIDemo11pMessage *task_UAV_msg = new TraCIDemo11pMessage;
                        populateWSM(task_UAV_msg);
                        task_UAV_msg->setByteLength(top_task.packet_size);
                        task_UAV_msg->setSenderAddress(myId);
                        task_UAV_msg->setRecipientAddress(candidate_UAV);
                        task_UAV_msg->setName(s.c_str());
                        sendDown(task_UAV_msg);
                        node_resource.waiting_tasks.push_back({top_task, 2});
                    }
                    else if(min_cost == cost_UAV_MEC) // 車輛傳給UAV再給MEC運算成本最低
                    {
                        CAR_UAV_MEC++;
                        // candidate_UAV_MEC指的是負責relay給MEC的UAV的id(因為要先傳給該UAV)
                        EV << myId << " : I find UAV " << candidate_UAV_MEC << " and it is connecting to a Edge Server!" << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                        EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_UAV_MEC = " << cost_UAV_MEC << endl;
                        double dis_CARtoUAV_x = curPosition.x - UAV_map[candidate_UAV_MEC].position.x;
                        double dis_CARtoUAV_y = curPosition.y - UAV_map[candidate_UAV_MEC].position.y;
                        double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                        double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + (parameter.E2_car * dis_CARtoUAV)) * top_task.packet_size;
                        energyCommunication += energy_CARtoUAV;
                        EV << "(CAR->UAV)Energy in communication = " << energy_CARtoUAV << " / energyCommunication = " << energyCommunication << endl;
                        std::string s = "UAV_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                        TraCIDemo11pMessage *task_MEC_msg = new TraCIDemo11pMessage;
                        populateWSM(task_MEC_msg);
                        task_MEC_msg->setByteLength(top_task.packet_size);
                        task_MEC_msg->setSenderAddress(myId);
                        task_MEC_msg->setRecipientAddress(candidate_UAV_MEC);
                        task_MEC_msg->setName(s.c_str());
                        sendDown(task_MEC_msg);
                        node_resource.waiting_tasks.push_back({top_task, 3});
                    }
                }
            }
        }
        else
        {
            PacketLossTime++;
            CantFindOffload++;
            averageDelayPercent += 1.0;
            EV << myId << " : My Task is expired, packet loss! Size = " << top_task.packet_size << " / Packet loss time : " << PacketLossTime << endl;
        }
        EV << "-------------------------" << endl;
    }
    for(const auto &selfscheduledMessage : selfscheduledMessages)
    {
        scheduleAt(simTime() + selfscheduledMessage.cal_time, selfscheduledMessage.msg);
    }
}


/*void CoCaCoCar::dispatchTask()
{
    std::vector<ScheduledMessage> selfscheduledMessages;
    int loop_time = node_resource.pending_tasks.size();
    for(int i=0; i<loop_time; i++)
    {
        task top_task = node_resource.pending_tasks.front();
        node_resource.pending_tasks.pop();
        if(top_task.expire_time > simTime().dbl()) // 任務尚未過期，嘗試決定任務要怎麼offload
        {
            if(node_resource.remain_cpu >= top_task.require_cpu && node_resource.remain_memory >= top_task.require_memory && top_task.packet_size <= 12750) // 車輛自行處理
            {
                double cal_time = top_task.packet_size / (node_resource.cal_capability * top_task.require_cpu / 100.0);
                node_resource.remain_cpu -= top_task.require_cpu;
                node_resource.remain_memory -= top_task.require_memory;
                node_resource.handling_tasks.push_back(top_task);
                std::string s = "myTask_"+ std::to_string(top_task.packet_size);
                EV << myId << ": handling the task! Handling time = " << cal_time << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                cMessage *Task_handlingTimer = new cMessage(s.c_str());
                selfscheduledMessages.push_back({Task_handlingTimer, cal_time});
            }
            else
            {
                if(!UAV_map.empty())
                {
                    for(auto UAV_in_my_Area : UAV_map)
                    {
                        if(top_task.require_cpu <= UAV_in_my_Area.second.remain_cpu && top_task.require_memory <= UAV_in_my_Area.second.remain_memory && UAV_in_my_Area.second.Delay_to_MEC == -1)
                        {
                            EV << myId << ": I find UAV " << UAV_in_my_Area.first << " to handle my task!" << endl;
                            std::string s = "UAV_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory);
                            TraCIDemo11pMessage *task_UAV_msg = new TraCIDemo11pMessage;
                            populateWSM(task_UAV_msg);
                            task_UAV_msg->setByteLength(top_task.packet_size);
                            task_UAV_msg->setSenderAddress(myId);
                            task_UAV_msg->setRecipientAddress(UAV_in_my_Area.first);
                            task_UAV_msg->setName(s.c_str());
                            sendDown(task_UAV_msg);
                            node_resource.waiting_tasks.push_back(top_task);
                            break;
                        }
                        else if(UAV_in_my_Area.second.Delay_to_MEC != -1)
                        {
                            EV << myId << " : I find UAV " << UAV_in_my_Area.first << " and it is connecting to a Edge Server!" << endl;
                            std::string s = "UAV_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory);
                            TraCIDemo11pMessage *task_MEC_msg = new TraCIDemo11pMessage;
                            populateWSM(task_MEC_msg);
                            task_MEC_msg->setByteLength(top_task.packet_size);
                            task_MEC_msg->setSenderAddress(myId);
                            task_MEC_msg->setRecipientAddress(UAV_in_my_Area.first);
                            task_MEC_msg->setName(s.c_str());
                            sendDown(task_MEC_msg);
                            node_resource.waiting_tasks.push_back(top_task);
                            break;
                        }
                        else
                        {
                            EV << myId << " : I find UAV " << UAV_in_my_Area.first << " in my area, but it isn't available!" << endl;
                            node_resource.pending_tasks.push(top_task);
                        }

                    }
                }
                else
                {
                    EV << myId << " : No UAV in my area to help me!" << endl;
                    node_resource.pending_tasks.push(top_task);
                }
            }
        }
        else
        {
            PacketLossTime++;
            EV << myId << " : My Task is expired, packet loss! Size = " << top_task.packet_size << " / Packet loss time : " << PacketLossTime << endl;
        }
    }
    for(const auto &selfscheduledMessage : selfscheduledMessages)
    {
        scheduleAt(simTime() + selfscheduledMessage.cal_time, selfscheduledMessage.msg);
    }
}*/

void GHDSFCar::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
}

void GHDSFCar::finish()
{
    clearExpiredTask();
    Car_map.erase(myId);
    double TransRate = (SuccessedTime / (SuccessedTime + PacketLossTime));
    double PacketLossRate = (PacketLossTime / (SuccessedTime + PacketLossTime));
    EV << "Total Packet : " << TotalPacket << endl;
    EV << "Packet loss Time = " << PacketLossTime << endl;
    EV << "Transmission Successes Time = " << SuccessedTime << endl;
    EV << "Transmission Rate = " << TransRate << endl;
    EV << "Packet Loss Rate = " << PacketLossRate << endl;
    EV << "averageDelayPercent = " << (averageDelayPercent / (SuccessedTime + PacketLossTime)) << endl;
    EV << "CAR_SELF = " << CAR_SELF << endl;
    EV << "CAR_UAV = " << CAR_UAV << endl;
    EV << "CAR_MEC = " << CAR_MEC << endl;
    EV << "CAR_UAV_MEC = " << CAR_UAV_MEC << endl;
    EV << "Successed_Car = " << Successed_Car << endl;
    EV << "Successed_UAV = " << Successed_UAV << endl;
    EV << "Successed_MEC = " << Successed_MEC << endl;
    EV << "Can't Find Offload = " << CantFindOffload << endl;
    EV << "Total Energy Consumption = " << energyComputing + energyCommunication + energyFlying << endl; // J
    EV << "Energy in Computing = " << energyComputing << endl;
    EV << "Energy in Communication = " << energyCommunication << endl;
    EV << "Energy in UAV Flying = " << energyFlying << endl;
    EV << "The size of successful Task = " << taskSize << endl;
    EV << "Energy Efficiency = " << taskSize * 8 / (energyComputing + energyCommunication + energyFlying) << endl; // bit/J

    EV << TransRate << endl;
    EV << (averageDelayPercent / (SuccessedTime + PacketLossTime)) << endl;
    EV << energyComputing + energyCommunication + energyFlying << endl;
    EV << energyComputing << endl;
    EV << energyCommunication << endl;
    EV << energyFlying << endl;
    EV << taskSize * 8 / (energyComputing + energyCommunication + energyFlying) << endl;

    recordScalar("TotalPacket", TotalPacket);
    recordScalar("Packet loss Time", PacketLossTime);
    recordScalar("Transmission Successes Time", SuccessedTime);
    recordScalar("Transmission Rate", TransRate);
    recordScalar("Packet Loss Rate", PacketLossRate);
    recordScalar("averageDelayPercent", (averageDelayPercent / (SuccessedTime + PacketLossTime)));
    recordScalar("CAR_SELF", CAR_SELF);
    recordScalar("CAR_UAV", CAR_UAV);
    recordScalar("CAR_MEC", CAR_MEC);
    recordScalar("CAR_UAV_MEC", CAR_UAV_MEC);
    recordScalar("Successed_Car", Successed_Car);
    recordScalar("Successed_UAV", Successed_UAV);
    recordScalar("Successed_MEC", Successed_MEC);
    recordScalar("Successed_UAV_MEC", Successed_UAV_MEC);
    recordScalar("Can't Find Offload", CantFindOffload);
    recordScalar("EnergyComputing", energyComputing);
    recordScalar("EnergyCommunication", energyCommunication);
    recordScalar("EnergyFlying", energyFlying);
    recordScalar("taskSize", taskSize);
    recordScalar("energyEfficiency", taskSize * 8 / (energyComputing + energyCommunication + energyFlying));
}
