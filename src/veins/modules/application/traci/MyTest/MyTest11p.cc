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

#include <veins/modules/application/traci/MyTest/MyTest11p.h>
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include "veins/modules/messages/DemoSafetyMessage_m.h"

#include "veins/modules/messages/BeaconMessage_m.h"

#include "veins/modules/application/traci/common.h" // 所有全域變數宣告在common.h中，並在MyTest11p.cc中定義

using namespace veins;

double PacketLossTime = 0;
double SuccessedTime = 0;
double UAV_cal_capability = 1000000;
double MEC_cal_capability = 2000000;

std::mt19937 rnd_generator;

LAB_par parameter =
{
    1,    // E1_car = 發送器所耗功率
    1.25, // E2_car = 放大器所耗功率
    1.5,  // E3_car = 接收器所耗功率

    1,    // E1_UAV
    1.25, // E2_UAV
    1.5,  // E3_UAV

    1,    // E1_MEC
    1.25, // E2_MEC
    1.5,  // E3_MEC

    1,    // P_car = 處理任務所耗功率
    1.25, // P_UAV
    1.75, // P_MEC

    0.5,  // DelayRatio
    0.5   // EnergyRatio
};

Define_Module(veins::MyTest11p);

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

MyTest11p::MyTest11p() : node_resource(100,100) {} // 讓每個node都有自己的一個node_resource


void MyTest11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        srand((unsigned int)time(NULL));
        rnd_generator = std::mt19937(rand());
    }
    else if(stage == 1)
    {
        cMessage *taskMsg = new cMessage("generate_task");
        scheduleAt(simTime() + uniform(0.1 , 5.0), taskMsg);
        /*cMessage *resourceMsg = new cMessage("check_resource");
        scheduleAt(simTime() + 0.1, resourceMsg);*/
        cMessage *UAV_mapTimer = new cMessage("UAV_map");
        scheduleAt(simTime() + 2, UAV_mapTimer);
    }
}

void MyTest11p::onWSA(DemoServiceAdvertisment* wsa)
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

void MyTest11p::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);
    //EV_INFO << myId << ": Receive a packet from: " << wsm->getSenderAddress() << " at time: " << wsm->getTimestamp()/* << " And the data: " << wsm->getDemoData() */<< " Delay = " << simTime() - wsm->getTimestamp();
    if(std::string(wsm->getName()).substr(0, 12) == "TaskSendBack")
    {
        int Back_PacketSize = std::stoi(std::string(wsm->getName()).substr(13));
        LAddress::L2Type Back_UAVID = wsm->getSenderAddress();
        for (auto it = node_resource.waiting_tasks.begin(); it != node_resource.waiting_tasks.end();)
        {
            if (it->packet_size == Back_PacketSize)
            {
                if(simTime() > it->expire_time)
                {
                    PacketLossTime++;
                    EV << myId << ": Size = " << it->packet_size << " : packet loss!" << " This packet is handled by other node and send back from: " << Back_UAVID << " / Packet Loss Time : " << PacketLossTime << endl;
                    EV << "The expire time : " << it->expire_time << ", and now is : " << simTime() << endl;
                }
                else
                {
                    SuccessedTime++;
                    EV << myId << ": Size = " << it->packet_size << " : handling success!" << " This packet is handled by other node and send back from: " << Back_UAVID << " / Success Time : " << SuccessedTime << endl;
                    EV << "The expire time : " << it->expire_time << ", and now is : " << simTime() << endl;
                }

                it = node_resource.waiting_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                break;
            }
            else
            {
                ++it;  // 如果當前元素不符合條件，則遞增迭代器
            }
        }
    }

}

void MyTest11p::onBSM(DemoSafetyMessage* wsm)
{

}

void MyTest11p::onBM(BeaconMessage* bsm)
{
    //BeaconMessage* bm = check_and_cast<BeaconMessage*>(bsm);
    //findHost()->getDisplayString().setTagArg("i", 1, "blue");
    EV << myId << ": I Receive a beacon from UAV " << bsm->getSenderAddress() << " and it's position : " << bsm->getSenderPos() << " / speed : " << bsm->getSenderSpeed() << " / direction : " << bsm->getSenderDirection() << " / Delay to MEC : " << bsm->getDelay_to_MEC() << " / Distance to MEC : " << bsm->getDistance_to_MEC();
    UAV_map[bsm->getSenderAddress()] = {simTime().dbl(), bsm->getSenderPos(), (simTime() - bsm->getTimestamp()).dbl(), bsm->getDelay_to_MEC(), bsm->getDistance_to_MEC(), bsm->getRemain_cpu(), bsm->getRemain_mem()};
}

void MyTest11p::handleSelfMsg(cMessage* msg)
{
    if(!strcmp(msg->getName(), "generate_task"))
    {
        int numtasks = intuniform(3,8);
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
            else if (task_p >= 21 && task_p <= 50)
            {
                task t(2);
                t.id = myId;
                t.start_time = simTime().dbl();
                t.expire_time = t.start_time + t.delay_limit;
                node_resource.pending_tasks.push(t);
            }
            else if (task_p >= 51 && task_p <= 75)
            {
                task t(3);
                t.id = myId;
                t.start_time = simTime().dbl();
                t.expire_time = t.start_time + t.delay_limit;
                node_resource.pending_tasks.push(t);
            }
            else if (task_p >= 76 && task_p <= 100)
            {
                task t(4);
                t.id = myId;
                t.start_time = simTime().dbl();
                t.expire_time = t.start_time + t.delay_limit;
                node_resource.pending_tasks.push(t);
            }
            EV << "I'm " << myId << " and I generate a task: QoS = " << node_resource.pending_tasks.back().qos << " , Delay_limit : " << node_resource.pending_tasks.back().delay_limit <<  " , start_time = " << node_resource.pending_tasks.back().start_time << " , expire_time = " << node_resource.pending_tasks.back().expire_time << " , size = " << node_resource.pending_tasks.back().packet_size << endl;
        }
        dispatchTaskConsiderEnergy();
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
                if(simTime() > it->expire_time)
                {
                    PacketLossTime++;
                    EV << myId << ": Size = " << it->packet_size << " : packet loss!" << " Now remain cpu = " << node_resource.remain_cpu << " / memory = " << node_resource.remain_memory << " / Packet Loss Time : " << PacketLossTime << endl;
                    EV << "The expire time : " << it->expire_time << ", and now is : " << simTime() << endl;
                }
                else
                {
                    SuccessedTime++;
                    EV << myId << ": Size = " << it->packet_size << " : handling success!" << " Now remain cpu = " << node_resource.remain_cpu << " / memory = " << node_resource.remain_memory << " / Success Time : " << SuccessedTime << endl;
                    EV << "The expire time : " << it->expire_time << ", and now is : " << simTime() << endl;
                }
                it = node_resource.handling_tasks.erase(it);  // 刪除符合條件的元素並更新迭代器
                break;
            }
            else
            {
                ++it;  // 如果當前元素不符合條件，則遞增迭代器
            }
        }
        if(!node_resource.pending_tasks.empty())
            dispatchTaskConsiderEnergy();
    }
}

struct ScheduledMessage
{
    cMessage* msg;
    double cal_time;
};

void MyTest11p::dispatchTaskConsiderEnergy()
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
            double cost_car = -1;
            double delay_UAV = -1;
            double energy_UAV = -1;
            double cost_UAV = DBL_MAX;
            LAddress::L2Type candidate_UAV = -1;
            double delay_MEC = -1;
            double energy_MEC = -1;
            double cost_MEC = DBL_MAX;
            double delay_normalize = -DBL_MAX; // 用來將delay正規化
            double energy_normalize = -DBL_MAX; // 用來將energy正規化
            LAddress::L2Type candidate_MEC = -1; // candidate_MEC指的是負責relay給MEC的UAV的id(因為要先傳給該該UAV)
            if(node_resource.remain_cpu >= top_task.require_cpu && node_resource.remain_memory >= top_task.require_memory) // 車輛自行運算
            {
                double cal_time = top_task.packet_size / (node_resource.cal_capability * top_task.require_cpu / 100.0);
                if(cal_time <= (top_task.expire_time - simTime().dbl())) // 來得及算完才將其列考慮之一
                {
                    delay_car = cal_time;
                    energy_car = parameter.P_car * top_task.packet_size;
                    cost_car = parameter.DelayRatio * delay_car + parameter.EnergyRatio * energy_car;
                }
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
                        double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + parameter.E1_UAV + parameter.E3_car + (parameter.E2_car + parameter.E2_UAV) * dis_CARtoUAV); // 傳去和傳回來都一起先算
                        double energy_UAVtoMEC = (parameter.E1_UAV + parameter.E3_MEC + parameter.E1_MEC + parameter.E3_UAV + (parameter.E2_UAV + parameter.E2_MEC) * UAV_in_my_Area.second.Distance_to_MEC * UAV_in_my_Area.second.Distance_to_MEC); // 傳去和傳回來都一起先算
                        double tmp_energy_MEC = parameter.P_MEC * top_task.packet_size + energy_CARtoUAV + energy_UAVtoMEC;
                        // MEC 總處理時間 = 車輛到UAV傳輸時間(來回) + UAV到MEC傳輸時間(來回) + MEC處理時間
                        // Delay和Delay_to_MEC是beacon(packet_size=300)的傳輸時間，在這邊算大略的傳輸耗時直接根據packet_size等比例放大計算
                        double tmp_delay_MEC = 2 * ((UAV_in_my_Area.second.Delay + UAV_in_my_Area.second.Delay_to_MEC) / 300 * top_task.packet_size) + top_task.packet_size / (MEC_cal_capability * top_task.require_cpu / 100.0);
                        double tmp_cost_MEC = parameter.DelayRatio * tmp_delay_MEC + parameter.EnergyRatio * tmp_energy_MEC;
                        if(tmp_cost_MEC <= cost_MEC)
                        {
                            cost_MEC = tmp_cost_MEC;
                            energy_MEC = tmp_energy_MEC;
                            delay_MEC = tmp_delay_MEC;
                            candidate_MEC = UAV_in_my_Area.first; // candidate_MEC指的是負責relay給MEC的UAV的id(因為要先傳給該該UAV)
                        }
                    }
                    // 計算車輛offload到該UAV的成本(車輛->UAV)
                    if(UAV_in_my_Area.second.remain_cpu >= top_task.require_cpu && UAV_in_my_Area.second.remain_memory >= top_task.require_memory)
                    {
                        double dis_CARtoUAV_x = curPosition.x - UAV_in_my_Area.second.position.x;
                        double dis_CARtoUAV_y = curPosition.y - UAV_in_my_Area.second.position.y;
                        double dis_CARtoUAV = dis_CARtoUAV_x * dis_CARtoUAV_x + dis_CARtoUAV_y * dis_CARtoUAV_y; //距離原本應該要開根號，但是在耗能部分距離越長耗能設定為平方成長，故這邊就直接用
                        double energy_CARtoUAV = (parameter.E1_car + parameter.E3_UAV + parameter.E1_UAV + parameter.E3_car + (parameter.E2_car + parameter.E2_UAV) * dis_CARtoUAV); // 傳去和傳回來都一起先算(這邊不包含運算耗能)
                        double tmp_energy_UAV = parameter.P_UAV * top_task.packet_size + energy_CARtoUAV;
                        // UAV 總處理時間 = 車輛到UAV傳輸時間(來回) + UAV處理時間
                        // Delay是beacon(packet_size=300)(UAV->車輛)的傳輸時間，在這邊算大略的傳輸耗時直接根據packet_size等比例放大計算
                        double tmp_delay_UAV = 2 * (UAV_in_my_Area.second.Delay / 300 * top_task.packet_size) + top_task.packet_size / (UAV_cal_capability * top_task.require_cpu / 100.0);
                        double tmp_cost_UAV = parameter.DelayRatio * delay_UAV + parameter.EnergyRatio * energy_UAV;
                        if(tmp_cost_UAV <= cost_UAV)
                        {
                            cost_UAV = tmp_cost_UAV;
                            energy_UAV = tmp_energy_UAV;
                            delay_UAV = tmp_delay_UAV;
                            candidate_UAV = UAV_in_my_Area.first;
                        }
                    }
                }
            }
            // 正規化，先取得最大的energy和delay，接著再重新計算一次正規化過後的cost
            if (cost_car != -1)
            {
                energy_normalize = std::max(energy_normalize, energy_car);
                delay_normalize = std::max(delay_normalize, delay_car);
            }
            if (cost_UAV != DBL_MAX)
            {
                energy_normalize = std::max(energy_normalize, energy_UAV);
                delay_normalize = std::max(delay_normalize, delay_UAV);
            }
            if (cost_MEC != DBL_MAX)
            {
                energy_normalize = std::max(energy_normalize, energy_MEC);
                delay_normalize = std::max(delay_normalize, delay_MEC);
            }

            EV << "delay_car = " << delay_car << " / energy_car = " << energy_car << endl;
            EV << "delay_UAV = " << delay_UAV << " / energy_UAV = " << energy_UAV << endl;
            EV << "delay_MEC = " << delay_MEC << " / energy_MEC = " << energy_MEC << endl;
            EV << "delay_normalize = " << delay_normalize << " / energy_normalize = " << energy_normalize << endl;
            // 正規化，重新計算一次正規化過後的cost
            if(cost_car != -1)
            {
                cost_car = parameter.DelayRatio * (delay_car / delay_normalize) + parameter.EnergyRatio * (energy_car / energy_normalize);
            }
            if(cost_UAV != DBL_MAX)
            {
                cost_UAV = parameter.DelayRatio * (delay_UAV / delay_normalize) + parameter.EnergyRatio * (energy_UAV / energy_normalize);
            }
            if(cost_MEC != DBL_MAX)
            {
                cost_MEC = parameter.DelayRatio * (delay_MEC / delay_normalize) + parameter.EnergyRatio * (energy_MEC / energy_normalize);
            }

            if(cost_car != -1) // 車輛的成本不為-1代表車輛來得及自行運算(所以cost被更新)，比較時要三種方式一起比
            {
                double min_cost = std::min({cost_car, cost_UAV, cost_MEC}); // 就算沒和UAV連線，也是選cost_car讓車輛自己算
                if(min_cost == cost_car) // 車輛自行運算成本最低
                {
                    double cal_time = top_task.packet_size / (node_resource.cal_capability * top_task.require_cpu / 100.0);
                    node_resource.remain_cpu -= top_task.require_cpu;
                    node_resource.remain_memory -= top_task.require_memory;
                    node_resource.handling_tasks.push_back(top_task);
                    std::string s = "myTask_"+ std::to_string(top_task.packet_size);
                    EV << myId << ": handling the task! Handling time = " << cal_time << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_MEC = " << cost_MEC << endl;
                    cMessage *Task_handlingTimer = new cMessage(s.c_str());
                    selfscheduledMessages.push_back({Task_handlingTimer, cal_time});

                }
                else if(min_cost == cost_UAV) // 車輛傳給UAV運算成本最低
                {
                    EV << myId << ": I find UAV " << candidate_UAV << " to handle my task!"  << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_MEC = " << cost_MEC << endl;
                    std::string s = "UAV_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory);
                    TraCIDemo11pMessage *task_UAV_msg = new TraCIDemo11pMessage;
                    populateWSM(task_UAV_msg);
                    task_UAV_msg->setByteLength(top_task.packet_size);
                    task_UAV_msg->setSenderAddress(myId);
                    task_UAV_msg->setRecipientAddress(candidate_UAV);
                    task_UAV_msg->setName(s.c_str());
                    sendDown(task_UAV_msg);
                    node_resource.waiting_tasks.push_back(top_task);
                }
                else // 車輛傳給UAV再給MEC運算成本最低
                {
                    // candidate_MEC指的是負責relay給MEC的UAV的id(因為要先傳給該該UAV)
                    EV << myId << " : I find UAV " << candidate_MEC << " and it is connecting to a Edge Server!" << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                    EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_MEC = " << cost_MEC << endl;
                    std::string s = "UAV_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory);
                    TraCIDemo11pMessage *task_MEC_msg = new TraCIDemo11pMessage;
                    populateWSM(task_MEC_msg);
                    task_MEC_msg->setByteLength(top_task.packet_size);
                    task_MEC_msg->setSenderAddress(myId);
                    task_MEC_msg->setRecipientAddress(candidate_MEC);
                    task_MEC_msg->setName(s.c_str());
                    sendDown(task_MEC_msg);
                    node_resource.waiting_tasks.push_back(top_task);
                }
            }
            else // 車輛的成本為-1代表車輛沒辦法自行運算(cost_car沒被更新)，比較時只比UAV和MEC的cost，但如果兩者都是DBL_MAX(沒和UAV連線，這個任務就只能先放著)
            {
                if(cost_UAV == DBL_MAX && cost_MEC == DBL_MAX)
                {
                    EV << myId << ": I can't handle this task and no one can help me!" << endl;
                    node_resource.pending_tasks.push(top_task);
                    continue;
                }
                else
                {
                    if(cost_UAV <= cost_MEC) // 車輛傳給UAV運算成本最低
                    {
                        EV << myId << ": I find UAV " << candidate_UAV << " to handle my task!" << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                        EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_MEC = " << cost_MEC << endl;
                        std::string s = "UAV_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory);
                        TraCIDemo11pMessage *task_UAV_msg = new TraCIDemo11pMessage;
                        populateWSM(task_UAV_msg);
                        task_UAV_msg->setByteLength(top_task.packet_size);
                        task_UAV_msg->setSenderAddress(myId);
                        task_UAV_msg->setRecipientAddress(candidate_UAV);
                        task_UAV_msg->setName(s.c_str());
                        sendDown(task_UAV_msg);
                        node_resource.waiting_tasks.push_back(top_task);
                    }
                    else // 車輛傳給UAV再給MEC運算成本最低
                    {
                        // candidate_MEC指的是負責relay給MEC的UAV的id(因為要先傳給該該UAV)
                        EV << myId << " : I find UAV " << candidate_MEC << " and it is connecting to a Edge Server!" << " / size = " << top_task.packet_size << " / remain cpu = " << node_resource.remain_cpu << " remain memory = " << node_resource.remain_memory << endl;
                        EV << "cost_CAR = " << cost_car << " / cost_UAV = " << cost_UAV << " / cost_MEC = " << cost_MEC << endl;
                        std::string s = "UAV_MEC_handle_" + std::to_string(top_task.require_cpu) + "_" + std::to_string(top_task.require_memory);
                        TraCIDemo11pMessage *task_MEC_msg = new TraCIDemo11pMessage;
                        populateWSM(task_MEC_msg);
                        task_MEC_msg->setByteLength(top_task.packet_size);
                        task_MEC_msg->setSenderAddress(myId);
                        task_MEC_msg->setRecipientAddress(candidate_MEC);
                        task_MEC_msg->setName(s.c_str());
                        sendDown(task_MEC_msg);
                        node_resource.waiting_tasks.push_back(top_task);
                    }
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
}

/*void MyTest11p::dispatchTask()
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

void MyTest11p::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
}

void MyTest11p::finish()
{
    double TransRate = (SuccessedTime / (SuccessedTime + PacketLossTime));
    double PacketLossRate = (PacketLossTime / (SuccessedTime + PacketLossTime));
    EV << "Packet loss Time = " << PacketLossTime << endl;
    EV << "Transmission Successes Time = " << SuccessedTime << endl;
    EV << "Transmission Rate = " << TransRate << endl;
    EV << "Packet Loss Rate = " << PacketLossRate << endl;
}
