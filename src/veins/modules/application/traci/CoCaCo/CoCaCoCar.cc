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

#include "veins/modules/application/traci/CoCaCo/CoCaCoCar.h"

#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include "veins/modules/messages/DemoSafetyMessage_m.h"

#include "veins/modules/messages/BeaconMessage_m.h"

#include "veins/modules/application/traci/common.h" // 所有全域變數宣告在common.h中，並在MyTest11p.cc中定義

using namespace veins;

Define_Module(veins::CoCaCoCar);

CoCaCoCar::CoCaCoCar() : node_resource(100,100) {} // 讓每個node都有自己的一個node_resource


void CoCaCoCar::initialize(int stage)
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
        cMessage *taskMsg = new cMessage("generate_task");
        scheduleAt(simTime() + uniform(0.1 , 2.5), taskMsg);
        /*cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.1, resourceMsg);*/
        cMessage *UAV_mapTimer = new cMessage("UAV_map");
        scheduleAt(simTime() + 2, UAV_mapTimer);
    }
}

void CoCaCoCar::onWSA(DemoServiceAdvertisment* wsa)
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

void CoCaCoCar::onWSM(BaseFrame1609_4* frame)
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
                        averageDelayPercent += (1.0 - (it->first.expire_time - simTime().dbl()) / it->first.delay_limit);
                        taskSize += it->first.packet_size;
                        EV << myId << ": Size = " << it->first.packet_size << " : handling success!" << " / Success Time : " << SuccessedTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
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
                        averageDelayPercent += (1.0 - (it->first.expire_time - simTime().dbl()) / it->first.delay_limit);
                        taskSize += it->first.packet_size;
                        EV << myId << ": Size = " << it->first.packet_size << " : handling success!" << " / Success Time : " << SuccessedTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
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
                        averageDelayPercent += (1.0 - (it->first.expire_time - simTime().dbl()) / it->first.delay_limit);
                        taskSize += it->first.packet_size;
                        EV << myId << ": Size = " << it->first.packet_size << " : handling success!" << " / Success Time : " << SuccessedTime << endl;
                        EV << "The expire time : " << it->first.expire_time << ", and now is : " << simTime() << endl;
                    }

                    it = node_resource.waiting_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                    continue;
                }
            }
            ++it;  // 如果當前元素不符合條件，則遞增迭代器
        }
    }

}

void CoCaCoCar::onBSM(DemoSafetyMessage* wsm)
{

}

void CoCaCoCar::onBM(BeaconMessage* bsm)
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
        if(!node_resource.followed && !bsm->getFollowing())
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
        }
    }
}

void CoCaCoCar::handleSelfMsg(cMessage* msg)
{
    if(!strcmp(msg->getName(), "generate_task"))
    {
        int numtasks = intuniform(3,8);
        TotalPacket += numtasks;
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
            EV << "I'm " << myId << " and I generate a task: QoS = " << node_resource.pending_tasks.back().qos << " , Delay_limit : " << node_resource.pending_tasks.back().delay_limit <<  " , start_time = " << node_resource.pending_tasks.back().start_time << " , expire_time = " << node_resource.pending_tasks.back().expire_time << " , size = " << node_resource.pending_tasks.back().packet_size << endl;
        }
        CoCaCoTaskOffloading();
        delete msg;
        cMessage *taskMsg = new cMessage("generate_task");
        scheduleAt(simTime() + uniform(0.1 , 2.5), taskMsg);
    }
    else if(!strcmp(msg->getName(), "check_resource"))
    {
        delete msg;
        EV << "I'm " << myId << " and my remained cpu = " << node_resource.remain_cpu << ", remained memory = " << node_resource.remain_memory << endl;
        cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.1, resourceMsg);
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
        EV << myId << ": Handling finish. Size = " << finish_size << endl;
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
                                averageDelayPercent += (1.0 - (it2->first.expire_time - simTime().dbl()) / it2->first.delay_limit);
                                taskSize += it2->first.packet_size;
                                EV << myId << ": Full packet Size = " << it2->first.packet_size << " : handling success!" << " Success Time : " << SuccessedTime << endl;
                                EV << "The expire time : " << it2->first.expire_time << ", and now is : " << simTime() << endl;
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

            CoCaCoTaskOffloading();
            //dispatchTaskConsiderEnergy();
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

void CoCaCoCar::clearExpiredTask()
{
    for(auto it = node_resource.waiting_tasks.begin(); it != node_resource.waiting_tasks.end();)
    {
        if (it->first.expire_time < simTime().dbl())
        {
            PacketLossTime++;
            averageDelayPercent += 1.0;
            EV << myId << " : My Task is expired, packet loss! Size = " << it->first.packet_size << " / Packet loss time : " << PacketLossTime << endl;
            it = node_resource.waiting_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
            continue;
        }
        ++it;  // 如果當前元素不符合條件，則遞增迭代器
    }
}

void CoCaCoCar::CoCaCoTaskOffloading()
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
                    double dis_CARtoUAV_x = curPosition.x - UAV_map[minDelayUAV].position.x;
                    double dis_CARtoUAV_y = curPosition.y - UAV_map[minDelayUAV].position.y;
                    double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                    double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + (parameter.E2_car * dis_CARtoUAV)) * (top_task.packet_size / 3) * 2;
                    energyCommunication += energy_CARtoUAV;
                    EV << "(CAR->UAV)Energy in communication = " << energy_CARtoUAV << " / energyCommunication = " << energyCommunication << endl;
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
                        double dis_CARtoUAV_x = curPosition.x - UAV_map[minDelayUAV].position.x;
                        double dis_CARtoUAV_y = curPosition.y - UAV_map[minDelayUAV].position.y;
                        double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                        double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + (parameter.E2_car * dis_CARtoUAV)) * (top_task.packet_size / 3);
                        energyCommunication += energy_CARtoUAV;
                        EV << "(CAR->UAV)Energy in communication = " << energy_CARtoUAV << " / energyCommunication = " << energyCommunication << endl;
                        std::string s = "UAV_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                        TraCIDemo11pMessage *task_UAV_msg = new TraCIDemo11pMessage;
                        populateWSM(task_UAV_msg);
                        task_UAV_msg->setByteLength(top_task.packet_size / 3);
                        task_UAV_msg->setSenderAddress(myId);
                        task_UAV_msg->setRecipientAddress(minDelayUAV);
                        task_UAV_msg->setName(s.c_str());
                        sendDown(task_UAV_msg);
                        EV << myId << ": Send a task to MEC " << Nearest_MEC << ", full packet size = " << top_task.packet_size << ", handle size = " << top_task.packet_size / 3 << endl;
                        double energy_CARtoMEC = (parameter.E1_car + parameter.E3_MEC + (parameter.E2_car * Distance_to_MEC * Distance_to_MEC)) * (top_task.packet_size / 3);
                        energyCommunication += energy_CARtoMEC;
                        EV << "(CAR->MEC)Energy in communication = " << energy_CARtoMEC << " / energyCommunication = " << energyCommunication << endl;
                        std::string s1 = "Car_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                        TraCIDemo11pMessage *task_CartoMEC_msg = new TraCIDemo11pMessage;
                        populateWSM(task_CartoMEC_msg);
                        task_CartoMEC_msg->setByteLength(top_task.packet_size / 3);
                        task_CartoMEC_msg->setSenderAddress(myId);
                        task_CartoMEC_msg->setRecipientAddress(Nearest_MEC);
                        task_CartoMEC_msg->setName(s1.c_str());
                        sendDown(task_CartoMEC_msg);
                    }
                    else if(top_task.must_send_MEC == false)// UAV沒和MEC連線，車輛自己也沒和MEC連線 => 任務分為車輛和UAV計算
                    {
                        CAR_UAV++;
                        EV << myId << ": handle the task by myself and UAV! " << " Packet size = " << top_task.packet_size << " / handle size by car = " << top_task.packet_size / 2 << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                        node_resource.waiting_tasks.push_back({top_task, 3}); // 將任務放進等待任務欄中，3代表車輛(1)運算+UAV(2)運算
                        node_resource.queuing_tasks.push({top_task, top_task.packet_size / 2}); // 將車輛需要自己算的部分packet size放入
                        EV << myId << ": Send a task to UAV " << minDelayUAV << ", full packet size = " << top_task.packet_size << ", handle size = " << top_task.packet_size / 2 << endl;
                        double dis_CARtoUAV_x = curPosition.x - UAV_map[minDelayUAV].position.x;
                        double dis_CARtoUAV_y = curPosition.y - UAV_map[minDelayUAV].position.y;
                        double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                        double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + (parameter.E2_car * dis_CARtoUAV)) * (top_task.packet_size / 2);
                        energyCommunication += energy_CARtoUAV;
                        EV << "(CAR->UAV)Energy in communication = " << energy_CARtoUAV << " / energyCommunication = " << energyCommunication << endl;
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
                    double energy_CARtoMEC = (parameter.E1_car + parameter.E3_MEC + (parameter.E2_car * Distance_to_MEC * Distance_to_MEC)) * (top_task.packet_size / 2);
                    energyCommunication += energy_CARtoMEC;
                    EV << "(CAR->MEC)Energy in communication = " << energy_CARtoMEC << " / energyCommunication = " << energyCommunication << endl;
                    std::string s = "Car_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory) + "_" + std::to_string(top_task.packet_size);
                    TraCIDemo11pMessage *task_CartoMEC_msg = new TraCIDemo11pMessage;
                    populateWSM(task_CartoMEC_msg);
                    task_CartoMEC_msg->setByteLength(top_task.packet_size / 2);
                    task_CartoMEC_msg->setSenderAddress(myId);
                    task_CartoMEC_msg->setRecipientAddress(Nearest_MEC);
                    task_CartoMEC_msg->setName(s.c_str());
                    sendDown(task_CartoMEC_msg);
                }
                else if(top_task.must_send_MEC == false) // 車輛沒連線UAV也沒連線MEC => 車輛自行運算
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
            averageDelayPercent += 1.0;
            EV << myId << " : My Task is expired, packet loss! Size = " << top_task.packet_size << " / Must MEC = " << top_task.must_send_MEC << " / Packet loss time : " << PacketLossTime << endl;
        }
    }
    handleQueuingTask();
}

void CoCaCoCar::handleQueuingTask()
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
            energyComputing += cal_time * parameter.P_car;
            EV << myId << ": EnergyComsumption = " << cal_time * parameter.P_car << " / energyComputing = " << energyComputing << endl;
            node_resource.remain_cpu -= top_task.require_cpu;
            node_resource.remain_memory -= top_task.require_memory;
            node_resource.handling_tasks.push_back(top_task);
            std::string s = "myTask_"+ std::to_string(top_task.packet_size);
            EV << myId << ": handling the task! Handling time = " << cal_time << " / packet size = " << top_task.packet_size << " / handle size = " << top_task_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
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

void CoCaCoCar::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
}

void CoCaCoCar::finish()
{
    clearExpiredTask();
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
    EV << "Successed_UAV_MEC = " << Successed_UAV_MEC << endl;
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
    recordScalar("EnergyComputing", energyComputing);
    recordScalar("EnergyCommunication", energyCommunication);
    recordScalar("EnergyFlying", energyFlying);
    recordScalar("taskSize", taskSize);
    recordScalar("energyEfficiency", taskSize * 8 / (energyComputing + energyCommunication + energyFlying));
}
