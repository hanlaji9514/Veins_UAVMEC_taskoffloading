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

#include <veins/modules/application/traci/MyMethod/MyMethodRSU.h>
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

#include "veins/modules/messages/DemoSafetyMessage_m.h"

#include "veins/modules/messages/BeaconMessage_m.h"
#include "veins/modules/application/traci/common.h" // 所有全域變數宣告在common.h中，並在MyTest11p.cc中定義

#include <unordered_map>

using namespace veins;

std::unordered_map<LAddress::L2Type, Car_info> Car_map;
std::unordered_map<LAddress::L2Type, UAV_info> UAV_maps;
std::unordered_map<LAddress::L2Type, Coord> Dispatch_Coord;
std::vector<Cluster> all_clusters;
std::vector<std::vector<double>> hungarian_weight;
std::vector<cOvalFigure*> all_circles;

Define_Module(veins::MyMethodRSU);

const double INF = 1e9; // 代表無限大，用於hungarian初始化

void MyMethodRSU::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        canvas = getParentModule()->getCanvas();
        if (canvas)
        {
            EV << "Canvas is OK." << endl;
        }
        else
        {
            EV << "Canvas is not available." << endl;
        }
        cMessage *checkcarMsg = new cMessage("check_car");
        scheduleAt(simTime() + 0.5, checkcarMsg);
        cMessage *beaconTimer = new cMessage("beacon");
        scheduleAt(simTime() + 0.1, beaconTimer);
    }
}

MyMethodRSU::MyMethodRSU() : RSU_resource(10000,10000) {} // 讓每一個RSU都有其獨立的RSU_resource

void MyMethodRSU::onWSA(DemoServiceAdvertisment* wsa)
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

void MyMethodRSU::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);


    //EV << "MEC " << myId << ": Receive a packet from: " << wsm->getSenderAddress() << " at time: " << wsm->getTimestamp() << " And the Name: " << wsm->getName() << " Delay = " << simTime() - wsm->getTimestamp() << endl;
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
        // seglist[4] 是 full packet size
        // seglist[5] 是 source_id
        task received_t;
        received_t.source_id = std::stoi(seglist[5]);
        received_t.relay_id = wsm->getSenderAddress();
        received_t.packet_size = wsm->getByteLength();
        received_t.require_cpu = std::stoi(seglist[2]);
        received_t.require_memory = std::stoi(seglist[3]);
        received_t.full_packet_size = std::stoi(seglist[4]);
        RSU_resource.pending_tasks.push(received_t);
        EV << "RSU " << myId << ": Received a task from UAV " << wsm->getSenderAddress() << ", and the source car is " << received_t.source_id << " / handle size = " << received_t.packet_size << ", Full packet size = " << received_t.full_packet_size << endl;
        handleReceivedTask();
    }
    else if(std::string(wsm->getName()).substr(0, 14) == "Car_MEC_handle") // MEC收到車輛直接傳過來的任務請求
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
        // seglist[3] 是 require_cpu
        // seglist[4] 是 require_memory
        // seglist[5] 是 full packet size
        task received_t;
        received_t.relay_id = -1;
        received_t.source_id = wsm->getSenderAddress();
        received_t.packet_size = wsm->getByteLength();
        received_t.require_cpu = std::stoi(seglist[3]);
        received_t.require_memory = std::stoi(seglist[4]);
        received_t.full_packet_size = std::stoi(seglist[5]);
        RSU_resource.pending_tasks.push(received_t);
        EV << "RSU " << myId << ": Received a task from Car " << wsm->getSenderAddress() << " / handle size = " << received_t.packet_size << ", Full packet size = " << received_t.full_packet_size << endl;
        handleReceivedTask();
    }
}

void MyMethodRSU::onBM(BeaconMessage* bsm)
{
    if(bsm->getBeaconType() == 1)
    {
        //findHost()->getDisplayString().setTagArg("i", 1, "blue");
        EV_INFO << myId << " RSU: I Receive a beacon from UAV " << bsm->getSenderAddress() << " and it's position : " << bsm->getSenderPos() << " / speed : " << bsm->getSenderSpeed() << " / direction : " << bsm->getSenderDirection() << " / Delay to MEC : " << bsm->getDelay_to_MEC() << " / Distance to MEC : " << bsm->getDistance_to_MEC();
        LAddress::L2Type recipientAddress = bsm->getSenderAddress();
        BeaconMessage *bm = new BeaconMessage("UAV_beacon_RSUACK");
        populateWSM(bm);
        bm->setBeaconType(2);//Beacon_ACK回傳給UAV
        bm->setSenderAddress(myId);
        bm->setByteLength(300);
        bm->setTimestamp(simTime());
        bm->setRecipientAddress(recipientAddress);
        bm->setSenderPos(curPosition);
        sendDown(bm);
    }
}

void MyMethodRSU::handleSelfMsg(cMessage* msg)
{
    if (std::string(msg->getName()).substr(0, 5) == "Task_") // 任務處理完成，須回傳給轉傳過來的UAV讓他回傳給車輛或是直接回傳給車輛
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
                std::string s = "MECTaskSendBack_" + std::to_string(it->full_packet_size);
                TraCIDemo11pMessage *SendBack = new TraCIDemo11pMessage;
                populateWSM(SendBack);
                SendBack->setByteLength(it->packet_size);
                SendBack->setSenderAddress(myId);
                SendBack->setName(s.c_str());
                if(it->relay_id == -1) // 代表這個task是由車輛直接送來的
                {
                    SendBack->setRecipientAddress(it->source_id);
                    EV << "RSU " << myId << ": Handling finish. Full Packet Size = " << it->full_packet_size << " / Handle Size = " << finish_size << ", send back to Car " << it->source_id << endl;
                }
                else // relay_id有value，代表這個task是由UAV轉傳過來的
                {
                    SendBack->setRecipientAddress(it->relay_id);
                    EV << "RSU " << myId << ": Handling finish. Full Packet Size = " << it->full_packet_size << " / Handle Size = " << finish_size << ", send back to UAV " << it->relay_id << ", and Relay back to " << it->source_id << endl;
                }
                sendDown(SendBack);

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
    else if(!strcmp(msg->getName(), "check_car"))
    {
        EV << "MEC " << myId << ": All car in the map" << endl;
        for (const auto& car : Car_map)
        {
            LAddress::L2Type id = car.first;
            Coord coord = car.second.Position;
            EV << "CAR ID : " << id << ", Coord : (" << coord.x << ", " << coord.y << ")" << endl;
        }
        EV << "MEC " << myId << ": All UAV in the map" << endl;
        for (const auto& uav : UAV_maps)
        {
            LAddress::L2Type id = uav.first;
            Coord coord = uav.second.Position;
            EV << "UAV ID : " << id << ", Coord : (" << coord.x << ", " << coord.y << ")" << endl;
        }
        if(Car_map.size() >= 4)
        {
            all_clusters = agglomerative_clustering(Car_map, 400);
            for(auto &cluster : all_clusters)
            {
                cluster.centroid = calculate_centroid(cluster);
                EV << "Car In Cluster --> centroid = " << cluster.centroid << ": ";
                // 建立一個 cOvalFigure 物件
                cOvalFigure *circle = new cOvalFigure("circle");
                // 設定圓形的位置和大小，參數為 (左上角 x 座標, 左上角 y 座標, 寬度, 高度)
                circle->setBounds(cFigure::Rectangle(cluster.centroid.x, cluster.centroid.y, 200, 200));
                circle->setLineWidth(10);
                // 設定圓形的顏色，參數為 cFigure::Color 物件，可以使用 RGB 值或預定義的顏色名稱
                circle->setLineColor(cFigure::RED);
                // 將圓形加入到地圖上，參數為地圖的指標
                canvas->addFigure(circle);
                all_circles.push_back(circle);
                cluster.Total_task = 0;
                for(const auto car : cluster.CarInCluster)
                {
                    EV << car << " ";
                    cluster.Total_task += Car_map[car].Num_Task;
                }
                EV << "/ Total task in this cluster: " << cluster.Total_task << endl;
            }
            compute_hungarian_weight();
        }
        for(auto& car : Car_map) // 將每台車輛的任務計數器歸0
        {
            car.second.Num_Task = 0;
        }
        cMessage *checkcarMsg = new cMessage("check_car");
        scheduleAt(simTime() + 0.5, checkcarMsg);
    }
    else if(!strcmp(msg->getName(), "beacon"))
    {
        BeaconMessage *bsm = new BeaconMessage("MEC_beacon");
        // populate some common properties with the BaseWaveApplLayer method
        populateWSM(bsm);
        bsm->setSenderAddress(myId);
        // set the sender position with the mobility module position
        bsm->setSenderPos(curPosition);
        // set the speed with the mobility module speed
        // send the BSM to the MAC layer immediately
        bsm->setByteLength(300); //beacon message 大約為300Bytes
        bsm->setTimestamp(simTime());
        bsm->setRemain_cpu(RSU_resource.remain_cpu);
        bsm->setRemain_mem(RSU_resource.remain_memory);
        sendDown(bsm);
        cMessage *beaconTimer = new cMessage("beacon");
        scheduleAt(simTime() + 0.1, beaconTimer);
    }
    delete msg;
}

void MyMethodRSU::handleReceivedTask()
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
            EV << "RSU " << myId << ": handling the task! Handling time = " << cal_time << " / handle size = " << top_task.packet_size << " / full packet size = " << top_task.full_packet_size << " / remain cpu = " << RSU_resource.remain_cpu << " remain memory = " << RSU_resource.remain_memory << endl;
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

std::vector<Cluster> MyMethodRSU::agglomerative_clustering(std::unordered_map<LAddress::L2Type, Car_info> Car_map, double threshold)
{
    std::vector<Cluster> clusters;
    for(const auto &p : Car_map) // 初始化每一個車輛都是一個獨立的cluster
    {
        Cluster c;
        c.CarInCluster.push_back(p.first);
        c.tmp_MergeOrNot = false;
        clusters.push_back(c);
    }
    // 重複直到沒有可以合併的cluster
    bool merged = true;
    while(merged)
    {
        merged = false;
        for(auto &c : clusters) // 每一輪先將cluster是否已經合併重製成false
            c.tmp_MergeOrNot = false;
        // 建立一個新的簇的列表
        std::vector<Cluster> new_clusters;
        for(int i=0; i<clusters.size(); i++)
        {
            if(clusters[i].tmp_MergeOrNot)
                continue;
            // 設定一個變數來記錄最小的最大距離
            double min_max_distance = DBL_MAX;
            // 設定一個變數來記錄最小的最大距離對應的簇的索引值
            double min_max_index = -1;
            for(int j=i+1; j<clusters.size(); j++)
            {
                if(clusters[j].tmp_MergeOrNot)
                    continue;
                // 計算兩個簇之間的最大距離
                double distance = complete_linkage(clusters[i], clusters[j]);
                // 如果最大距離小於最小的最大距離，則更新最小的最大距離和最小的最大距離對應的簇的索引值
                if (distance < min_max_distance)
                {
                    min_max_distance = distance;
                    min_max_index = j;
                }
            }
            // 如果最小的最大距離小於閾值，則合併這兩個簇
            if (min_max_distance < threshold)
            {
                // 將要合併的這兩個cluster標記為已合併
                clusters[i].tmp_MergeOrNot = true;
                clusters[min_max_index].tmp_MergeOrNot = true;

                // 合併這兩個簇並加入新的簇的列表
                Cluster merged_cluster = merge_clusters(clusters[i], clusters[min_max_index]);
                new_clusters.push_back(merged_cluster);

                // 將已經合併的簇從舊的簇的列表中移除
                clusters.erase(clusters.begin() + min_max_index);
                clusters.erase(clusters.begin() + i);

                // 更新迴圈的索引值
                i--;
                min_max_index--;

                // 設定合併的標記為 true
                merged = true;

                // 繼續執行內層迴圈
                continue;
            }
            if (!clusters[i].tmp_MergeOrNot)
            {
                new_clusters.push_back(clusters[i]);
            }
        }
        clusters = new_clusters;
    }
    EV << "clusters.size() = " << clusters.size() << " / UAV_maps.size() = " << UAV_maps.size() << endl;
    while(clusters.size() < UAV_maps.size()) // cluster數量太少，將車輛最多的cluster根據中心點的X座標切成兩半
    {
        EV << "Need to split!" << endl;
        int max_index;
        int max_cnt = -1;
        for(int i=0; i<clusters.size(); i++)
        {
            int n = clusters[i].CarInCluster.size();
            if(n > max_cnt)
            {
                max_index = i;
                max_cnt = n;
            }
        }
        if(max_cnt == 1)
        {
            EV << "The max_cnt = 1, Can't split!" << endl;
            break;
        }
        else
            spilt_Cluster(clusters, max_index);
    }
    while(clusters.size() > UAV_maps.size()) // cluster數量太多，將車輛數量最少的cluster刪除，不讓它成為UAV的目的地候選(不參與匈牙利演算法)
    {
        EV << "Need to abort the cluster with lowest number!" << endl;
        int min_index;
        int min_cnt = INT_MAX;
        for(int i=0; i<clusters.size(); i++)
        {
            int n = clusters[i].CarInCluster.size();
            if(n < min_cnt)
            {
                min_index = i;
                min_cnt = n;
            }
        }
        clusters.erase(clusters.begin() + min_index);
    }
    return clusters;
}

void MyMethodRSU::spilt_Cluster(std::vector<Cluster>& clusters, int index)
{
    Coord centroid = calculate_centroid(clusters[index]);
    Cluster need_to_spilt = clusters[index];
    Cluster left_cluster, right_cluster;
    clusters.erase(clusters.begin() + index);
    for(int i=0; i<need_to_spilt.CarInCluster.size(); i++)
    {
        if(Car_map[need_to_spilt.CarInCluster[i]].Position.x > centroid.x)
            right_cluster.CarInCluster.push_back(need_to_spilt.CarInCluster[i]);
        else
            left_cluster.CarInCluster.push_back(need_to_spilt.CarInCluster[i]);
    }
    clusters.push_back(left_cluster);
    clusters.push_back(right_cluster);
}

// 一個函式，用來計算兩個cluster之間的全連結距離
double MyMethodRSU::complete_linkage(Cluster& c1, Cluster& c2)
{
    double max_distance = 0;
    for (const LAddress::L2Type& p1 : c1.CarInCluster)
    {
        for (const LAddress::L2Type& p2 : c2.CarInCluster)
        {
            double distance = euclidean_distance(Car_map[p1].Position, Car_map[p2].Position);
            if (distance > max_distance)
            {
                max_distance = distance;
            }
        }
    }
    return max_distance;
}

Cluster MyMethodRSU::merge_clusters(Cluster& c1, Cluster& c2)
{
    Cluster c;
    c.CarInCluster.insert(c.CarInCluster.end(), c1.CarInCluster.begin(), c1.CarInCluster.end());
    c.CarInCluster.insert(c.CarInCluster.end(), c2.CarInCluster.begin(), c2.CarInCluster.end());
    return c;
}

double MyMethodRSU::euclidean_distance(Coord &p1, Coord &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

Coord MyMethodRSU::calculate_centroid(const Cluster& c)
{
    Coord cen;
    double x_sum = 0;
    double y_sum = 0;
    for (const LAddress::L2Type& id : c.CarInCluster)
    {
        // 從 Car_map 中查找車輛的座標
        Coord p = Car_map.at(id).Position;
        x_sum += p.x;
        y_sum += p.y;
    }
    cen.x = x_sum / c.CarInCluster.size();
    cen.y = y_sum / c.CarInCluster.size();
    return cen;
}

void MyMethodRSU::compute_hungarian_weight()
{
    hungarian_weight.clear();
    double max_resource = DBL_MIN;
    double max_calculate = DBL_MIN;
    std::vector<LAddress::L2Type> UAV_id;
    for(auto &pair : UAV_maps) // 取得UAV中資源及計算能力的最大值，用於normalize
    {
        UAV_id.push_back(pair.first);
        double tmp_re = pair.second.remain_cpu+pair.second.remain_mem;
        max_resource = std::max(max_resource, tmp_re);
        max_calculate = std::max(max_calculate, pair.second.cal_capability);
    }
    for(int i=0; i<all_clusters.size(); i++)
    {
        std::vector<double> distances;
        std::vector<double> calculate;
        std::vector<double> weight;
        // 遍歷所有的UAV
        for (LAddress::L2Type UAVid : UAV_id)
        {
            auto UAVinfo = UAV_maps[UAVid];
            // 獲取UAV的座標
            Coord UAV_coord = UAVinfo.Position;
            // 計算這個cluster的中心點和這個UAV的座標之間的距離
            double distance = euclidean_distance(all_clusters[i].centroid, UAV_coord);
            EV << "Cluster : " << all_clusters[i].centroid << " / UAV_coord : " << UAV_coord << " / Distance : " << distance << endl;
            // 將距離添加到一維vector中
            distances.push_back(distance);
        }
        for(LAddress::L2Type UAVid : UAV_id)
        {
            auto UAVinfo = UAV_maps[UAVid];
            // 先將UAV的計算能力做normalize
            double status = 0.5 * ((UAVinfo.remain_cpu+UAVinfo.remain_mem) / max_resource) + 0.5 * ((UAVinfo.cal_capability) / max_calculate);
            double c = all_clusters[i].Total_task / status; // 運算成本(目前CPU,MEM資源越多、UAV運算能力越強的成本越低)
            EV << "Task in Cluster: " << all_clusters[i].Total_task << " / remain_cpu = " << UAVinfo.remain_cpu << " remain_mem = " << UAVinfo.remain_mem << " / UAV_status : " << status << " / calculate_cost : " << c << endl;
            calculate.push_back(c);
        }

        double dis_min = *std::min_element(distances.begin(), distances.end());
        double dis_max = *std::max_element(distances.begin(), distances.end());
        double cal_min = *std::min_element(calculate.begin(), calculate.end());
        double cal_max = *std::max_element(calculate.begin(), calculate.end());
        for(auto &x : distances) // 距離成本normalize
        {
            x = parameter.DistanceRatio * ((x - dis_min) / (dis_max - dis_min));
        }
        for(auto &x : calculate) // 計算成本normalize
        {
            if(cal_max == cal_min)
            {
                x = 0;
            }
            else
            {
                x = parameter.CalculateRatio * ((x - cal_min) / (cal_max - cal_min));
            }
        }
        EV << "distance_cost weight:" << endl;
        for(auto &i : distances)
        {
            EV << i << " ";
        }
        EV << endl;
        EV << "calculate_cost weight:" << endl;
        for(auto &i : calculate)
        {
            EV << i << " ";
        }
        EV << endl;
        weight.resize(distances.size());
        std::transform(distances.begin(), distances.end(), calculate.begin(), weight.begin(), std::plus<double>()); // 將兩種weight加起來
        hungarian_weight.push_back(weight);
    }
    EV << "hungarian weight:" << endl;
    for(auto &i : hungarian_weight)
    {
        for(auto &j : i)
        {
            EV << j << " ";
        }
        EV << endl;
    }

    std::vector<int> assignment = hungarian(hungarian_weight);
    // 輸出結果
    double total = 0; // 總成本
    EV << "UAV Dispatching:" << endl;
    // 輸出每個集群分配給無人機的結果和成本
    for (int i = 0; i < assignment.size(); ++i)
    {
        EV << "Cluster " << i << " assigned to UAV " << UAV_id[assignment[i]] << " and cost : " << hungarian_weight[i][assignment[i]] << endl;
        total += hungarian_weight[i][assignment[i]]; // 計算總成本
        Dispatch_Coord[UAV_id[assignment[i]]] = all_clusters[i].centroid;
    }
    EV << "UAV and its dispatched coordination :" << endl;
    for(auto pair : Dispatch_Coord)
    {
        EV << "UAV " << pair.first << " : " << pair.second << endl;
    }
    EV << "Total cost = " << total << endl; // 輸出總成本

}

// 匈牙利演算法的實現
std::vector<int> MyMethodRSU::hungarian(const std::vector<std::vector<double>>& cost)
{
    // 初始化變量
    int n = cost.size(), m = cost[0].size(); // n和m分別代表成本矩陣的行數和列數
    std::vector<double> u(n+1), v(m+1), p(m+1), way(m+1); // u和v用於潛在的行和列減法操作，p用於存儲匹配信息，way用於記錄最佳路徑
    std::vector<int> ans(n); // 存儲最終的分配結果

    // 主要的匈牙利演算法
    for (int i=1; i<=n; ++i)
    {
        p[0] = i;
        int j0 = 0; // j0用於記錄當前找到的最小值所在的列
        std::vector<double> minv(m+1, INF); // minv用於存儲每列的最小值
        std::vector<bool> used(m+1, false); // used標記某列是否已被選擇
        do {
            used[j0] = true;
            int i0 = p[j0], j1; // i0是當前行，j1是下一列
            double delta = INF; // delta用於記錄最小的未選擇列的最小值
            // 尋找未選擇列的最小值
            for (int j=1; j<=m; ++j)
            {
                if (!used[j])
                {
                    double cur = cost[i0-1][j-1]-u[i0]-v[j]; // 計算調整後的成本
                    if (cur < minv[j])
                    {
                        minv[j] = cur;
                        way[j] = j0; // 更新最佳路徑
                    }
                    if (minv[j] < delta)
                    {
                        delta = minv[j];
                        j1 = j; // 更新最小值所在的列
                    }
                }
            }
            // 更新u和v以及未選擇列的最小值
            for (int j=0; j<=m; ++j)
            {
                if (used[j])
                {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else
                {
                    minv[j] -= delta;
                }
            }
            j0 = j1; // 移動到下一列
        } while (p[j0] != 0);
        // 更新匹配信息
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    // 填充答案向量
    for (int j=1; j<=m; ++j)
    {
        if (p[j] != 0)
        {
            ans[p[j]-1] = j-1;
        }
    }
    return ans; // 返回最終的分配結果
}

void MyMethodRSU::handlePositionUpdate(cObject* obj)
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

