/**
* LBR算法:
 * 1 算法中,当前节点优先选择距离接收器近的节点作为gateway
*/
#include "ns3/flow-monitor-module.h"
#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"

#include "math.h"
#include "time.h"
#include "stdlib.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <stack>
#include <string>
#include <iomanip>
//ADD BY ZLB
#include "global.h"
#include "energy.h"
#include "final_record.h"
#include "GraphList.h"
using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE("LBRScript");

//函数声明
void resetRemain();
int getJump(uint32_t id);
double GetdistanFromMSink(Ptr<Node> srcN);
void startNeighborFindFrom(Ptr<Node> n);
void startBuildJump(Ptr<Node> n);
static inline int32_t ProcessJumpPacket(Ptr<Node> thisNode, Ptr<Packet> packet,Ipv4Header h);
double GetdistanOf2Nodes(Vector one,Vector two);
double GetdistanOf2Id(uint32_t id1,uint32_t id2);
void buildMapIdAdr();
void ContinueJumpRouting(Ptr<Node> n, uint32_t jumps, Ipv4Address sinkAdr);
uint32_t AnalyzeSptPacket(stringstream &ss, Ipv4Address &source);
uint32_t AnalyzeSptPacket(stringstream &ss);
void initial();
void updateNeighbor();
void startRoute(Ptr<Node>thisNode,Ipv4Address srcAdr);
void printNeighbor();
// void printPaths();
// void findRoutes(int id);
// void buildRoutes();
void updateNeighborTable();
void update();
void setUpdateDoneFlag(bool flag);


clock_t lifeTime;
string mobilityModel="";
bool isLifeCycle;
bool isEntity;

/*可调参数*/
double dataInterval = 8.0; //发包间隔
double updateInterval = 8.0;//每隔多久更新一次邻居表
double jumpInterval = 8.0;
double maxDis = 15.0;      //最大通信距离,要考虑区域面积和部署相关问题
double buildNeighborDone = 2.0;//新建立邻居表结束时间

int send = 0;
Time netDelay;


//add for min jump
map<uint32_t,set<uint32_t>>routes;
set<uint32_t>remains;
set<uint32_t>oneSet;
set<uint32_t>twoSet;
set<uint32_t>threeSet;
set<uint32_t>fourSet;
set<uint32_t>fiveSet;
uint32_t nNodes = 20;
int maxPackets = 900;
static bool buildjumpSetDone = false;
/*
 * 定义UdpSocketFactory为socket的类型和socket的广播地址
 */
TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
InetSocketAddress broadcastAdr = InetSocketAddress(Ipv4Address::GetBroadcast(), 80);
//全局网络设置
static std::string phyMode("DsssRate11Mbps");
static double Prss = -80; //dBm
static double offset = 81;  //I don't know
/*
 * Simulator initials global parameters
 */
double totalTime = 9999.0;  //max simulation time
clock_t simStartRealTime;
clock_t simFinishRealTime;
double simStopTime=0.0;
stringstream ssi;
//Topology
double maxX;//x length of the scenario
double maxY;
double mSinkTraceY;
bool gridTest=true;
//Data packet
uint32_t pktSize = 1500;//1000字节  9个节点 包含6个sense和3个sink，一共发10次 9*10*1000=90000,一轮发90000.
double totalBytesGenerated;
double initialJ = 5.0;
double totalConsumed=0;
uint32_t energyTraceIndex=0;//The global id of node to trace energy, default 0 means no trace
double TxPara=0.000006;//0.000006J/Bytes for Tx
double RxPara=0.000006;//0.000006J/Bytes for Rx
uint32_t nDrain=0;
double Tgather=0.65;
//Algorithm
string algType;
//Others
bool Banim=false;
bool Brotate=true;//是否启动动态网关
bool Bdrain=true;
bool isStatic = true;
AnimationInterface *Panim = 0;
uint32_t firstDrainedNodeId=0;
bool firstDrainFlag=true;
uint32_t firstDrainedSinkId=0;
double firstDrainedNodeTime=0;
string thisSimPath;//simulation path
string exeName="LBRMain";
string simFolderName="/home/wsn/sim_temp/";
uint32_t totalBytesGathered;
double totalEnergyConsumed;
double networkLifetime;
double dataGatherPercentage=1.0;
double RngRun;
double simRunTime;
double EnergyConsumeRate;
uint32_t totalJumps;
double sinkVariance;
bool rewrite = false;
string timeStr;
/*
 * 声明NodeContainer、NetDeviceContainer、Ipv4InterfaceContainer
 * 和JumpCountTableChain等的全局变量
 */
//Node container
static NodeContainer senseNodes;
static NodeContainer mobileSinkNode;
//Net device container
NetDeviceContainer senseDevices;
NetDeviceContainer mobileSinkDevice;
//Ipv4 related container
Ipv4InterfaceContainer senseIfs;
Ipv4InterfaceContainer mobileSinkIf;
//Energy source pointer
double remaingJ[MAX_NUM];
// set<uint32_t>remains;
// vector<uint32_t>totalNodes; 
GraphList graph;


static bool updateDone = false;

//存普通节点ip和对应的ip地址容器
map<uint32_t,Ipv4Address>mapIdAdr;
/*
 *数据发送到下一跳网关
 */
void TransmitDataPacket(Ptr<Node> localNode, Ipv4Address sourceAdr,
		Ipv4Address sinkAdr) {
	NS_LOG_LOGIC(
			std::endl<<TIME_STAMP_FUC <<sourceAdr <<" to "<<sinkAdr<<" by "
			<<GetNodeIpv4Address(localNode));
	// NS_LOG_LOGIC(TIME_STAMP_FUC<<"CheckRemainingJ for "<<localAdr<<" passed!");
	if (CheckRemainingJ(localNode)) {
		// if(!updateDone){
		// 	cout<<"邻居表未更新完成--------------------------------------------------"<<endl;
		// 	return;
		// }
		//printNeighbor();
		Ipv4Address localAdr = GetNodeIpv4Address(localNode);
		uint32_t localId = localNode->GetId();
		uint32_t gatewayId = 1024;
		//根据邻居表获取下一跳的id
		double distance1 = 0.0;
		double minWeight = 1024;//设定一个比较大的值.
		bool flag = false;
		for (int i = 0; i < graph.m_vCount; ++i){
			int curId = graph.m_vVertex[i].id;
			if((uint32_t)curId != localId)
				continue;



			//第一步,先过滤出跳数最少的节点
			int minJump = 1024;
			Edge* edge1 = graph.m_vVertex[i].next;
			while(edge1){
				int jump = edge1->jump;
				if(jump < minJump){
					minJump = jump;
				}
				edge1 = edge1->next;
				//break;
			}
			//第二步,找到最小跳数并且最大交付概率的,RSSI
			Edge* edge = graph.m_vVertex[i].next;
			while(edge){
				if(edge->jump != minJump){
					edge = edge->next;
					continue;
				}
				distance1 = GetdistanOf2Id(curId,(uint32_t)edge->id);
				if(distance1 <= maxDis){
					flag = true;
				}
				if(!flag)
						gatewayId = 1024;
				double weight = edge->weight;
				if(weight < minWeight){
					minWeight = weight;
					gatewayId = (uint32_t)edge->id;
				}
				edge = edge->next;
				//break;
			}
			break;
		}
		if(gatewayId == 1024){
			return;
		}

		pktType pktType = data_Type;
		//通过传过来的路由表获取
		Ipv4Address gatewayAdr = mapIdAdr[gatewayId];
		Ptr<Node>nodePtr = GetNodePtrFromGlobalId(gatewayId,senseNodes,mobileSinkNode);
		if (localAdr == sourceAdr) {
			NS_LOG_INFO(
					TIME_STAMP_FUC<<sourceAdr<<"(sense) sent a data packet to "<<sinkAdr);
			totalBytesGenerated += pktSize;
		}
		NS_LOG_LOGIC(
				TIME_STAMP_FUC<<"localAdr = "<<localAdr <<", sinkAdr = "<<sinkAdr<<", gatewayAdr = "<<gatewayAdr);
		Ptr<Packet> dataPkt = Create<Packet>(pktSize);
		Ipv4Header h;
		h.SetDestination(sinkAdr);
		h.SetSource(sourceAdr);
		h.SetIdentification(pktType);
		dataPkt->AddHeader(h);
		InetSocketAddress gateAdr = InetSocketAddress(gatewayAdr, 80);
		Ptr<Socket> srcSoc = Socket::CreateSocket(localNode, tid);
		srcSoc->Connect(gateAdr);
		srcSoc->Send(dataPkt);
		UpdateEnergySources(localNode, dataPkt, 0,mobileSinkNode);
		NS_LOG_LOGIC(TIME_STAMP_FUC<<"Socket from "<<localAdr<<" launched!");
	} else {
		NS_LOG_LOGIC(
				TIME_STAMP_FUC <<GetNodeIpv4Address(localNode)
				<<" failed in transmitting data pkt for lack of energy");
	}
}
/*
 * 全体senseNodes将按一定的数据生成率向某一个sinkNode发送感知到的数据
 */
void DataToSink() {
	double interval = 0;	//产生随机的发包间隔
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
			i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
			if (!CheckRemainingJ(thisNode)) {
				continue;
			} else {
				//找到下一跳ip,将当前节点数据包传输到下一跳.
				Ipv4Address dstAdr = GetNodeIpv4Address(mobileSinkNode.Get(0));
				cout<<TIME_STAMP_FUC<<"datatosink--数据发出,节点id为:"<<thisNode->GetId()<<" ,num = "<<send<<endl;
				Simulator::Schedule(Seconds(interval), &TransmitDataPacket,
						thisNode, srcAdr, dstAdr);
						send++;
				interval += 3;
			}
	}
	// cout<<"------------------------------------------------>"<<send<<endl;
	// cout<<maxPackets<<endl;
	if(send == maxPackets){
		lifeTime = Simulator::Now().GetSeconds();
		Simulator::Stop(Seconds(0.0));
	}
	Simulator::Schedule(Seconds(dataInterval), &DataToSink);
}
/*
 * 用于senseNode收到data_Type的packet后的处理方法
 */
static inline int32_t ProcessDataPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h){
	//分析包的source和destination
	Ipv4Address srcAdr = h.GetSource();
	Ipv4Address dstAdr =h.GetDestination();
	Ipv4Address localAdr = GetNodeIpv4Address(thisNode);

	if (localAdr != dstAdr) {//gateway node received the packet, transmit it to the next gateway node
		//产生随机的发包间隔
		NS_LOG_LOGIC(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)<<"received a data packet from "
					<<h.GetSource()<<" to "<<h.GetDestination()<<srcAdr<<"to"<<dstAdr);
		double interval = RandomDoubleVauleGenerator(0.0, 0.5);
		Simulator::Schedule(Seconds(interval), &TransmitDataPacket, thisNode,
				srcAdr, dstAdr);
		return 0;
	}
	NS_LOG_INFO(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
	return 0;
}

/*
 * 用于senseNode收到jump packet之后处理的内联函数
 */
static inline int32_t ProcessJumpPacket(Ptr<Node> thisNode, Ptr<Packet> packet,Ipv4Header h){
	double interval = RandomDoubleVauleGenerator(0.0, 0.5);	//产生随机的发包间隔
	Ipv4Address gateway = h.GetSource();	//gateway Ipv4Address
	stringstream recvData;
	packet->CopyData(&recvData, packet->GetSize());
	Ipv4Address source;
	uint32_t pktJumps = AnalyzeSptPacket(recvData, source);//Get jumps and source address from SPT packet
	NS_LOG_LOGIC(
			TIME_STAMP_FUC<< thisNode->GetId()<<" received a JUMP packet, gateway= "<<gateway <<", pktJumps = "
			<<pktJumps<<", source = "<<source);
	uint32_t jump = pktJumps+1;
	uint32_t _id = thisNode->GetId();
	if(jump == 1){
		oneSet.insert(_id);
	}
	if(jump == 2){
		twoSet.insert(_id);
	}
	if(jump == 3){
		threeSet.insert(_id);
	}
	if(jump == 4){
		fourSet.insert(_id);
	}
	if(jump == 5){
		fiveSet.insert(_id);
	}
	if(oneSet.size() + twoSet.size() + threeSet.size() + fourSet.size() + fiveSet.size() == nNodes){
		set<uint32_t>::iterator itset; 
		// cout<<"1跳对应的节点:"<<endl;
		// for(itset = oneSet.begin();itset != oneSet.end();itset++){
		// 	cout<<*itset<<" ";
		// }
		// cout<<endl;
		// cout<<"2跳对应的节点:"<<endl;
		// for(itset = twoSet.begin();itset != twoSet.end();itset++){
		// 	cout<<*itset<<" ";
		// }
		// cout<<endl;
		// cout<<"3跳对应的节点:"<<endl;
		// for(itset = threeSet.begin();itset != threeSet.end();itset++){
		// 	cout<<*itset<<" ";
		// }
		// cout<<endl;
		// cout<<"4跳对应的节点:"<<endl;
		// for(itset = fourSet.begin();itset != fourSet.end();itset++){
		// 	cout<<*itset<<" ";
		// }
		// cout<<endl;
		// cout<<"5跳对应的节点:"<<endl;
		// for(itset = fiveSet.begin();itset != fiveSet.end();itset++){
		// 	cout<<*itset<<" ";
		// }
		// cout<<endl;
		routes.insert(pair<uint32_t,set<uint32_t>>(1,oneSet));
		routes.insert(pair<uint32_t,set<uint32_t>>(2,twoSet));
		routes.insert(pair<uint32_t,set<uint32_t>>(3,threeSet));
		routes.insert(pair<uint32_t,set<uint32_t>>(4,fourSet));
		routes.insert(pair<uint32_t,set<uint32_t>>(4,fiveSet));
		//清空
		oneSet.clear();
		twoSet.clear();
		threeSet.clear();
		fourSet.clear();
		fiveSet.clear();
		buildjumpSetDone = true;
		//startNeighborFindFrom(mobileSinkNode.Get(0));
		update();
	}
	Simulator::Schedule(Seconds(interval), &ContinueJumpRouting, thisNode,jump, source);
	remains.erase(find(remains.begin(),remains.end(),thisNode->GetId()));
	return 0;
}
/*
 * 收到packet的回调函数
 */
void RecvPacketCallback(Ptr<Socket> socket) {
	// cout<<"进入回调-------------------"<<endl;
	Ptr<Packet> pkt;
	Address from;
	while ((pkt = socket->RecvFrom(from))) {
		Ptr<Packet> packet = pkt->Copy();	
		nodeType nType;
		pktType pType;
		if (packet->GetSize() > 0) {
			Ptr<Node> thisNode = socket->GetNode();	
				//这里这个socket是received socket，不是发送的那个
			Ipv4Header h;
			packet->PeekHeader(h);
			packet->RemoveHeader(h);
			pType = pktType(h.GetIdentification());
			nType = CheckNodeType(thisNode,senseNodes,mobileSinkNode.Get(0));
			switch (nType) {
			case mobileSink_Type: {	//mobileSinkNode收到packet
				switch (pType) {
					case dataGathering_Type:
						//ProcessDataGatheringPacket(thisNode, packet, h);
						break;
					case data_Type:
						//ProcessDataGatheringPacket(thisNode, packet, h);
						// cout<<"进入回调--接收数据包的节点id是:"<<thisNode->GetId()<<endl;
						totalBytesGathered += pktSize;
						break;
					case neighbor_Type:
						{
							// cout<<"插入邻居表,源节点为------->"<<GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,mobileSinkNode)->GetId()<<"目的节点为------>"<<thisNode->GetId()<<"跳数为------>"<<1<<endl;
							Ptr<Node>sourceNode = GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,mobileSinkNode);
							//TODO
							Vector location2;
							if (isStatic){
								location2 = sourceNode->GetObject<MobilityModel>()->GetPosition();
							}else{
								location2 = sourceNode->GetObject<MobilityModel>()->GetPosition();
							}
							//接收器节点
							Ptr<Node>msNode = mobileSinkNode.Get(0);
							Vector location1 = msNode->GetObject<MobilityModel>()->GetPosition();
							double weight = GetdistanOf2Nodes(location2,location1);
							graph.addNewEdgeToList(GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,mobileSinkNode)->GetId(),weight,1,mobileSinkNode.Get(0)->GetId());
						}
						break;
					default:
						break;
					}
				break;
			}
			case sense_Type: {	//senseNode收到packet
				switch (pType) {
					case data_Type: {///收到data_Type的packet，将执行中继
						if (CheckRemainingJ(thisNode, packet)) {
							UpdateEnergySources(thisNode, packet, 1,mobileSinkNode);
							// cout<<"进入回调--接收数据包的节点id是:"<<thisNode->GetId()<<endl;
							ProcessDataPacket(thisNode, packet, h);
						}
						break;
					}
					case neighbor_Type: {///收到data_Type的packet，将执行中继
						/*LBR更新邻居表方案*/
						uint32_t curId = thisNode->GetId();
						//发送广播包的节点
						Ipv4Address src = h.GetSource();
						Ptr<Node>node = GetNodePtrFromIpv4Adr(src,senseNodes,mobileSinkNode);
						uint32_t id = node->GetId();//发送广播包的id
						// cout<<"当前发送广播包的节点ip为："<<src<<"当前发送广播包的节点id为："<<id<<"接收到广播包的id为："<<curId<<endl;
						Ptr<Node>msNode = mobileSinkNode.Get(0);
						Vector location = msNode->GetObject<MobilityModel>()->GetPosition();						
						Vector location1;//接收
						Vector location2;//发送
						if (isStatic){
							location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
						}else{
							location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
						}
						double distance1 = GetdistanOf2Nodes(location1,location);//接收广播包到ms的距离
						if(id == mobileSinkNode.Get(0)->GetId()){
							graph.addNewEdgeToList(curId,distance1,1,id);
							return;
						}
						if (isStatic){
							location2 = node->GetObject<MobilityModel>()->GetPosition();
						}else{
							location2 = node->GetObject<MobilityModel>()->GetPosition();
						}
						double distance2 = GetdistanOf2Nodes(location2,location);
						if(distance1 < distance2){
							//TODO Get jump
							int jump = getJump(curId);
							// cout<<"插入邻居表,源节点为------->"<<id<<"目的节点为------>"<<curId<<"跳数为------>"<<jump<<endl;
							//cout<<"源节点:"<<curId<<"--->获取当前待插入邻居表的节点"<<id<<",跳数为--------------------->"<<jump<<endl;
							if(jump != 1024){
								graph.addNewEdgeToList(id,distance1,jump,curId);
							}
						}
						break;
					}
					case jump_Type:{
						if(remains.size() == 0)
							return;
						set<uint32_t>::iterator it;
						it = find(remains.begin(),remains.end(),thisNode->GetId());
						if(it != remains.end()){//存在
							ProcessJumpPacket(thisNode, packet, h);
						}
						break;
					}
					default: {
						break;
					}
				}
				break;
			}
			default:	//不可能的情况
				break;
			}
		}
	}
}

/*
 * 创建系统仿真文件夹
 */
void createSimFolder(){
	time_t rawTime;
	time(&rawTime);
	struct tm *timeInfo;
	timeInfo = localtime(&rawTime);
	char pblgtime[20];
	strftime(pblgtime, 20, "%Y-%m-%d %X", timeInfo);
    timeStr = pblgtime;
	//创建sim结果文件夹，日期为目录名
	ssi<<simFolderName;
	ssi<<(timeInfo->tm_year+1900)<<"-";
	ssi<<setw(2)<<setfill('0')<<right<<(timeInfo->tm_mon+1)<<"-";
	ssi<<setw(2)<<setfill('0')<<right<<timeInfo->tm_mday;
	mkdir(ssi.str().c_str(), S_IRWXU);
	//创建sim结果子文件夹，时间为目录名
	ssi << "/";
	ssi << setw(2) << setfill('0') << right << timeInfo->tm_hour;
	ssi << setw(2) << setfill('0') << right << timeInfo->tm_min;
	ssi << setw(2) << setfill('0') << right << timeInfo->tm_sec;
	mkdir(ssi.str().c_str(), S_IRWXU);
	ssi<<"/";
	thisSimPath=ssi.str();//文件夹路径，下层是具体文件
	ssi.str("");
	ssi.clear();
}
/*
 * 不同文件log级别设置并使能log
 */
void setLog(){
	LogComponentEnable("LBRScript", LOG_LEVEL_DEBUG);
	LogComponentEnable("Energy", LOG_LEVEL_DEBUG);
	LogComponentEnable("AdhocWifiMac", LOG_LEVEL_DEBUG);
}
/*
 * 基本网络设置
 */
void netSet(){
	/// disable fragmentation for frames below 2200 bytes
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
			StringValue("2200"));
	/// turn off RTS/CTS for frames below 2200 bytes
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
			StringValue("2200"));
	/// Fix non-unicast data rate to be the same as that of unicast
	Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
			StringValue(phyMode));
}
/*
 * 创建节点
 */
void createNode(){
	//Create nodes
	senseNodes.Create(nNodes);
	// std::string traceFile = "scratch/NOMADIC/test.ns_movements";
	// Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	// ns2.Install ();

	if(!isEntity){
		std::string traceFile = "scratch/"+mobilityModel+"/test.ns_movements";
		Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
		ns2.Install ();
	}

	mobileSinkNode.Create(1);
	NS_LOG_DEBUG("Create nodes done!");
}
/*
 * 创建移动模型并安装到节点
 */
void createMobilityModel(){
	//test grid position allocator
	//Install mobility
	MobilityHelper mobility;
	mobility.SetPositionAllocator("ns3::GridPositionAllocator", 
		"GridWidth",UintegerValue(5), 
		"MinX", DoubleValue(0.0), 
		"MinY", DoubleValue(0.0), 
		"DeltaX", DoubleValue(150.0), 
		"DeltaY",DoubleValue(150.0));
	if (isStatic) {
		mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	}else {
		if(mobilityModel == "RWP"){
			ObjectFactory pos1;
			pos1.SetTypeId ("ns3::RandomRectanglePositionAllocator");
			pos1.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=600.0]"));
			pos1.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=450.0]"));
			Ptr<PositionAllocator> taPositionAlloc1 = pos1.Create ()->GetObject<PositionAllocator> ();
			mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
											"Speed", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=40.0]"),
											"Pause", StringValue ("ns3::ConstantRandomVariable[Constant=20.0]"),
											"PositionAllocator", PointerValue (taPositionAlloc1)
											);
		}else{
			mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
				"Speed", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=4.0] "),
				"Pause", StringValue ("ns3::ConstantRandomVariable[Constant=20.0]"),
				"Bounds", StringValue ("0|600|0|450"));
		}

	}
	if(isEntity){
		mobility.Install(senseNodes);
	}
	mobility.Install(mobileSinkNode);

	
	//接收器安装移动模型
	// Ptr<ListPositionAllocator> lpa = CreateObject<ListPositionAllocator>();
	// lpa->Add(Vector(300, 500, 0));
	// mobility.SetPositionAllocator(lpa);

	// mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	// mobility.Install(mobileSinkNode);
	mobileSinkNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(300, 250, 0));

	NS_LOG_DEBUG("Install mobility done!");
}
/*
 * 创建wifi通信设备
 */
void createWifiDevice(){
	stringstream ssi;
	//Create devices
	WifiHelper wifi;
	wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
	/** Wifi PHY **/
	/***************************************************************************/
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	wifiPhy.Set("RxGain", DoubleValue(-10));
	wifiPhy.Set("TxGain", DoubleValue(offset + Prss));
	wifiPhy.Set("CcaMode1Threshold", DoubleValue(0.0));
	/***************************************************************************/
	/** wifi channel 同时设置最大通信距离 **/
	YansWifiChannelHelper wifiChannel;
	wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
			DoubleValue(maxDis));
/// create wifi channel
	Ptr<YansWifiChannel> wifiChannelPtr = wifiChannel.Create();
	wifiPhy.SetChannel(wifiChannelPtr);
	/** MAC layer **/
// Add a non-QoS upper MAC, and disable rate control
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
			StringValue(phyMode), "ControlMode", StringValue(phyMode));
/// Set it to ad-hoc mode
	wifiMac.SetType("ns3::AdhocWifiMac");
/** install PHY + MAC **/
	senseDevices = wifi.Install(wifiPhy, wifiMac, senseNodes);
	mobileSinkDevice = wifi.Install(wifiPhy, wifiMac, mobileSinkNode);
	ssi<<thisSimPath<<exeName;
	wifiPhy.EnablePcap(ssi.str().c_str(), mobileSinkNode.Get(0)->GetId(), 0, true);
	ssi.str("");
	ssi.clear();
	NS_LOG_DEBUG("Create devices done!");
	NS_LOG_DEBUG("Set pcap and anmi done!");
}
/*
 * 安装网络协议栈，比如AODV等协议，需要安装到该处。
 */
void installInternetStack(){
	InternetStackHelper stack2;
	stack2.Install (mobileSinkNode);
	stack2.Install (senseNodes);//注意，AODV协议安装到nodes节点上

	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	senseIfs = ipv4.Assign(senseDevices);
	mobileSinkIf = ipv4.Assign(mobileSinkDevice);
	NS_LOG_DEBUG("Install Internet stack done!");
}
/*
 *设置socket广播回调
 */
void createSocketCallBack(){
	vector<Ptr<Socket> > recvSocket(nNodes+1);
	recvSocket[0] = Socket::CreateSocket(mobileSinkNode.Get(0),tid);
	recvSocket[0]->Bind(InetSocketAddress(mobileSinkIf.GetAddress(0), 80));
	recvSocket[0]->SetRecvCallback(MakeCallback(&RecvPacketCallback));
	for (uint32_t i = 1; i < nNodes+1; i++) {
		recvSocket[i] = Socket::CreateSocket(senseNodes.Get(i-1), tid);
		recvSocket[i]->Bind(InetSocketAddress(senseIfs.GetAddress(i-1), 80));
		recvSocket[i]->SetRecvCallback(MakeCallback(&RecvPacketCallback));
	}
	NS_LOG_DEBUG("Set recvSocket done!");
}
/*
 * 是否生成XML动画文件
 */
void createXml(){
	/**Set anim**/
	if (Banim) {
		// if(isEntity){
		// 	Panim = new AnimationInterface("/home/wsn/sim_temp/LBR/RWP/output.xml");
		// }else{
			Panim = new AnimationInterface("/home/wsn/sim_temp/LBR/"+mobilityModel+"/output.xml");
		// }
		ssi.str("");
		ssi.clear();
		Panim->SetMaxPktsPerTraceFile(50000000);
		Panim->UpdateNodeColor(mobileSinkNode.Get(0), 0, 255, 0);
		Panim->UpdateNodeSize(mobileSinkNode.Get(0)->GetId(), 10.0, 10.0);
		for (NodeContainer::Iterator i = senseNodes.Begin();
				i != senseNodes.End(); i++) {
			Ptr<Node> n = *i;
			Panim->UpdateNodeSize(n->GetId(), 2.0, 2.0);
		}
	}
	NS_LOG_DEBUG("Set anmi done!");
}
void finalRecord(){
	// simFinishRealTime = clock();
	// cout<<endl;

	// cout<<"系统吞吐量:"<<totalBytesGathered*1.0/((simFinishRealTime-simStartRealTime)/1000)<<"(Byte/s)"<<endl;
	// cout<<"系统PDR指标:"<<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
	// EnergyFinalRecord(senseNodes, remaingJ);

	std::cout<<"\n\n***** OUTPUT *****\n\n";
	std::stringstream ss;
	if(isLifeCycle){
		if(isEntity){
			ss << "/home/wsn/sim_temp/LBR/"<<mobilityModel<<"/"<<"LIFE"<<"/"<<initialJ<<"-entity.record";//RWP/
		}else{
			ss << "/home/wsn/sim_temp/LBR/"<<mobilityModel<<"/"<<"LIFE"<<"/"<<initialJ<<"-"+mobilityModel+".record";
		}
	}else{
		if(isEntity){
			ss << "/home/wsn/sim_temp/LBR/"<<mobilityModel<<"/"<<maxPackets<<"-entity.record";
		}else{
			ss << "/home/wsn/sim_temp/LBR/"<<mobilityModel<<"/"<<maxPackets<<"-"+mobilityModel+".record";
		}
	}
	
	std::ofstream of(ss.str().c_str());
	simFinishRealTime = clock();
	if(mobilityModel.length()==0){
		cout<<"使用的移动模型:RWP"<<endl;
		ss << std::endl << "使用的移动模型:" << "RWP"<< std::endl;
	}else{
		cout<<"使用的移动模型:"<<mobilityModel<<endl;
		ss << std::endl << "使用的移动模型:" << mobilityModel<< std::endl;	
	}

	//如果测量网络生存周期,记录如下值
	if(isLifeCycle){
		cout<<"初始能量:"<<initialJ<<endl;
		cout<<"网络生存期:"<<lifeTime<<"(s)"<<endl;
		ss << std::endl << "初始能量:" << initialJ<<"(J)"<< std::endl;
		ss << std::endl << "网络生命周期:" << lifeTime<<"(s)"<< std::endl;
	}else{
		std::cout << "网络时延: " << netDelay<< "\n";
		std::cout<<"PDR指标:"<<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
		std::cout<<"网络吞吐量:"<<totalBytesGathered*1.0/((simFinishRealTime-simStartRealTime)/1000)<<"(Byte/s)"<<endl;
		ss << std::endl << "网络时延:" << netDelay<<"(ms)"<< std::endl;
		ss << std::endl << "网络吞吐量: " << totalBytesGathered*1.0/((simFinishRealTime-simStartRealTime)/1000)<<"(Byte/s)"<<endl;
		ss << std::endl << "PDR指标:" <<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
	}

	// ss << "Total comsumed energy = " << totalConsumed <<std::endl;

	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();

}

double GetdistanOf2Nodes(Vector one,Vector two) {
	return std::sqrt((one.x - two.x)*(one.x - two.x) + (one.y - two.y)*(one.y - two.y));
}


double GetdistanOf2Id(uint32_t id1,uint32_t id2){
	Ptr<Node>thisNode = GetNodePtrFromGlobalId(id1,senseNodes,mobileSinkNode);
	Ptr<Node>gatewayNode = GetNodePtrFromGlobalId(id2,senseNodes,mobileSinkNode);
	//TODO
	Vector location1;
	Vector location2;
	if (isStatic) {
		location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
		location2 = gatewayNode->GetObject<MobilityModel>()->GetPosition();
	}else{
		location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
		if(id2 == mobileSinkNode.Get(0)->GetId()){
			location2 = gatewayNode->GetObject<MobilityModel>()->GetPosition();
		}else{
			location2 = gatewayNode->GetObject<MobilityModel>()->GetPosition();
		}
	}
	return GetdistanOf2Nodes(location1,location2);
}

void startNeighborFindFrom(Ptr<Node> n){
	cout<<"当前是否建立跳数对应关系完成??"<<buildjumpSetDone<<endl;
	// if(buildjumpSetDone){
		Ipv4Address sinkSource =GetNodeIpv4Address(n);
		stringstream ss;
		ss<<"0/"<<sinkSource;
		stringstream pktContents(ss.str().c_str());//路由广播从SinkNode发出来的时候，数据里面的跳数是0，/后面是sinkSource的Ipv4Address
		Ptr<Packet> pkt = Create<Packet>((uint8_t *) pktContents.str().c_str(),
				pktContents.str().length());
		Ipv4Header ipv4Header;
		pktType pktType=neighbor_Type;
		//	ipv4Header.SetDestination(senseIfs.GetAddress(0));
		ipv4Header.SetSource(sinkSource);
		ipv4Header.SetIdentification(pktType);
		pkt->AddHeader(ipv4Header);

		Ptr<Socket> source = Socket::CreateSocket(n, tid);
		source->Connect(broadcastAdr);
		source->SetAllowBroadcast(true);	//socket发送广播必须有这么一个设置
		source->Send(pkt);
		cout<<"从msink节点开始广播建立邻居表："<<n->GetId()<<endl;
	// }else{
	// 	Simulator::Schedule(Seconds(0.5), &startNeighborFindFrom,mobileSinkNode.Get(0));
	// }

}

void startBuildJump(Ptr<Node> n){
	buildjumpSetDone =false;
	routes.clear();
	resetRemain();
	Ipv4Address sinkSource =GetNodeIpv4Address(n);
	stringstream ss;
	ss<<"0/"<<sinkSource;
	stringstream pktContents(ss.str().c_str());//路由广播从SinkNode发出来的时候，数据里面的跳数是0，/后面是sinkSource的Ipv4Address
	Ptr<Packet> pkt = Create<Packet>((uint8_t *) pktContents.str().c_str(),pktContents.str().length());
	Ipv4Header ipv4Header;
	pktType pktType=jump_Type;
	ipv4Header.SetSource(sinkSource);
	ipv4Header.SetIdentification(pktType);
	pkt->AddHeader(ipv4Header);
	Ptr<Socket> source = Socket::CreateSocket(n, tid);
	source->Connect(broadcastAdr);
	source->SetAllowBroadcast(true);	//socket发送广播必须有这么一个设置
	source->Send(pkt);
	// cout<<TIME_STAMP_FUC<<"-->从该节点开始广播建立跳数对应的数据结构："<<n->GetId()<<endl;
	Simulator::Schedule(Seconds(jumpInterval),&startBuildJump,mobileSinkNode.Get(0)); 
}

void buildMapIdAdr(){
	mapIdAdr.clear();
	pair<uint32_t,Ipv4Address>pairNode;
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
	i++) {
		Ptr<Node> thisNode = *i;
		uint32_t id = thisNode->GetId();
		Ipv4Address sourceAdr = GetNodeIpv4Address(thisNode);
		pairNode.first = id;
		pairNode.second = sourceAdr;
		mapIdAdr.insert(pairNode);
	}
	//插入目的地
	Ptr<Node>msNode = mobileSinkNode.Get(0);
	mapIdAdr.insert(pair<uint32_t,Ipv4Address>(msNode->GetId(),GetNodeIpv4Address(msNode)));
	//ceshi
	map <uint32_t, Ipv4Address>::iterator itr;
	for ( itr = mapIdAdr.begin( ); itr != mapIdAdr.end( ); itr++ ){
		cout<<itr->first<<":"<<itr->second<<endl;
	}
	cout<<mobileSinkNode.Get(0)->GetId()<<":"<<endl;
}


void ContinueJumpRouting(Ptr<Node> n, uint32_t jumps, Ipv4Address sinkAdr) {
	NS_LOG_LOGIC(
					endl<<TIME_STAMP_FUC
					<<"From sinkNode "
					<<sinkAdr<<" by "<<GetNodeIpv4Address(n));
	stringstream ss;
	pktType pktType=jump_Type;
	ss<<jumps<<'/'<<sinkAdr;
	string data = ss.str();
	stringstream pktContents(data);
	Ptr<Packet> pkt = Create<Packet>((uint8_t *) pktContents.str().c_str(),
			pktContents.str().length());
	Ipv4Header ipv4Header;
	ipv4Header.SetSource(GetNodeIpv4Address(n));
	ipv4Header.SetIdentification(pktType);
	pkt->AddHeader(ipv4Header);
	Ptr<Socket> srcSoc = Socket::CreateSocket(n, tid);
	srcSoc->Connect(broadcastAdr);
	srcSoc->SetAllowBroadcast(true);	//socket发送广播必须有这么一个设置
	srcSoc->Send(pkt);
}
/*
 *解析SPT包的跳数以及sinkSource的地址
 */
uint32_t AnalyzeSptPacket(stringstream &ss, Ipv4Address &source) {
	uint32_t jumps;
	stringstream si;
	char c[ss.str().size()];
	ss >> c;
	char *p = c;
	while (*p != '/') {
		si << *p;
		p++;
	}
	si >> jumps;
	si.str("");
	si.clear();
	p++;
	while (*p != '\0') {
		si << *p;
		p++;
	}
	source = Ipv4Address(si.str().c_str());
	NS_LOG_LOGIC(FUC<<"source = "<<source<<", jumps = "<<jumps);
	return jumps;
}

uint32_t AnalyzeSptPacket(stringstream &ss) {
	uint32_t jumps;
	stringstream si;
	char c[ss.str().size()];
	ss >> c;
	char *p = c;
	while (*p != '/') {
		si << *p;
		p++;
	}
	si >> jumps;
	si.str("");
	si.clear();
	//NS_LOG_DEBUG("-------------------------------------, jumps = "<<jumps);
	return jumps;
}
void initial(){
	for (NodeContainer::Iterator i = senseNodes.Begin();
			i != senseNodes.End(); i++) {
		Ptr<Node>node = *i;
		// totalNodes.push_back(node->GetId());
		remains.insert(node->GetId());
	}
	graph.m_vCount = nNodes;
	graph.makeVertexArray();
	cout<<"neighbor struct initial done!!"<<endl;
}
void resetRemain(){
	for (NodeContainer::Iterator i = senseNodes.Begin();
			i != senseNodes.End(); i++) {
		Ptr<Node>node = *i;
		// totalNodes.push_back(node->GetId());
		remains.insert(node->GetId());
	}
}

/*一个节点邻居发现的开始*/
void startRoute(Ptr<Node>thisNode,Ipv4Address srcAdr){
	Ptr<Packet> pkt = Create<Packet>(pktSize);
	Ipv4Header ipv4Header;
	pktType pktType=neighbor_Type;
	ipv4Header.SetSource(srcAdr);
	ipv4Header.SetIdentification(pktType);
	pkt->AddHeader(ipv4Header);

	Ptr<Socket> source = Socket::CreateSocket(thisNode, tid);
	source->Connect(broadcastAdr);
	source->SetAllowBroadcast(true);	//socket发送广播必须有这么一个设置
	source->Send(pkt);
	// cout<<"测试一个节点的广播："<<thisNode->GetId()<<endl;
}
void updateNeighbor(){
	double interval = 0;	//产生随机的发包间隔
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
			i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
		Simulator::Schedule(Seconds(interval), &startRoute,thisNode,srcAdr);
		// cout<<"从一个节点开始广播建立邻居："<<thisNode->GetId()<<endl;
		interval += 0.1;
		graph.clearNode(thisNode->GetId());
	}
}
void printNeighbor(){
	graph.print();
	Simulator::Schedule(Seconds(updateInterval), &printNeighbor);
}
void updateNeighborTable(){
	graph.updateArray();
}
/*定时任务,定时交换邻居信息,更新邻居表*/
void update(){
	updateDone = false;
	updateNeighbor();
	Simulator::Schedule(Seconds(buildNeighborDone), &setUpdateDoneFlag,true);
	//Simulator::Schedule(Seconds(updateInterval), &update);
}

void setSimStartTime(){
	simStartRealTime = clock();
}

/*当前节点距离接收器的距离*/
double GetdistanFromMSink(Ptr<Node> srcN) {
	Vector srcP;
	if(isStatic){
		srcP = srcN->GetObject<MobilityModel>()->GetPosition();
	}else{
		srcP = srcN->GetObject<MobilityModel>()->GetPosition();
	}
	Ptr<Node>msNode = mobileSinkNode.Get(0);
	Ptr<MobilityModel> remCpmm = msNode->GetObject<MobilityModel>();
	Vector remP = remCpmm->GetPosition();
	return std::sqrt((srcP.x - remP.x)*(srcP.x - remP.x) + (srcP.y - remP.y)*(srcP.y - remP.y));
}

void setUpdateDoneFlag(bool flag){
	updateDone = flag;
}

int getJump(uint32_t id){
	for(map<uint32_t, set<uint32_t>>::iterator iter = routes.begin();iter != routes.end();iter++){
		set<uint32_t>curNodes = iter->second;
		uint32_t curHop = iter->first;
		for(set<uint32_t>::iterator itset = curNodes.begin();itset != curNodes.end();itset++){
			uint32_t curId = *itset;	
			if(curId == id){
				return (int)curHop;
			}
		}
	}
	return 1024;
}
/*
 * 工程入口
 */
int main(int argc, char* argv[]) {
	//step0:全局变量初始化
	mSinkTraceY = maxY/2; //the y value of sink path
	//step 1 :定义main中局部变量
	double simStartTime = 0.0;  //seconds
	//step 2:解析命令行输入参数
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of Nodes", nNodes);
	cmd.AddValue("mobilityModel","use mobilityModel", mobilityModel);
	cmd.AddValue("isLifeCycle","is test lifeCycle", isLifeCycle);
	cmd.AddValue("isEntity","is entity", isEntity);
	cmd.AddValue("totalTime", "Simulation time length", totalTime);
	cmd.AddValue("simStartTime", "Simulation start time", simStartTime);
	cmd.AddValue("maxDis", "max distance to transmit", maxDis);
	cmd.AddValue("initialJ","Initial energy of each BaiscEnergySource", initialJ);
	cmd.AddValue("rewrite", "Whether to rewrite the result-sptMain.record",rewrite);
	cmd.AddValue("pktSize","Size of data packe",pktSize);
	cmd.AddValue("maxPackets","Size of data packe",maxPackets);
	cmd.AddValue("gridTest","Whether to use grid tes",gridTest);
	cmd.AddValue("Banim","Whether to generate animatit",Banim);
	cmd.AddValue("energyTraceIndex","The global id of node to trace energy",energyTraceIndex);
	cmd.AddValue("dataInterval","The time interval of data generation",dataInterval);
	cmd.AddValue("TxPara","The parameter of Tx",TxPara);
	cmd.AddValue("RxPara","The parameter of Rx",RxPara);
	cmd.AddValue("Brotate","Whether to enable gateway rotation ",Brotate);
	cmd.AddValue("Bdrain","Whether to enable drain notice broadcast ",Bdrain);
	cmd.AddValue("isStatic","Whether install static mobilitymodel ",isStatic);
	cmd.AddValue("Tgather","The percentage of threshold of data gathering percentage,(0,1) ",Tgather);
	cmd.Parse(argc, argv);
	NS_LOG_DEBUG("Configure done!");
	//放在cmd.Parse()后面才会生效的变量赋值
	simStopTime=totalTime;
	//step 3:创建仿真文件夹
	createSimFolder();
	//step 4:设置不同文件log级别
	setLog();
	//step 5:设置基础网络
	netSet();
	//step 6:创建节点
	createNode();
	//step 7:创建并安装移动模型
	createMobilityModel();
	//step 8:创建wifi设备
	createWifiDevice();
	//step 9:安装网络协议栈
	installInternetStack();
	//step 10:设置socket回调
	createSocketCallBack();
	//step 11:生成xml动画文件
	createXml();
	//step 12:节点安装能量
	InstallEnergy(senseNodes);
	cout<<"install energy done!"<<endl;
	//build map   节点id和地址的映射关系。

		//FlowMonitor set------    
	Ptr<FlowMonitor> flowMonitor;    
	FlowMonitorHelper flowHelper;    
	flowMonitor = flowHelper.InstallAll();

	buildMapIdAdr();
	cout<<"build idAdrMap done!"<<endl;
	initial(); 
	//从路由发现到数据传输 开始
	Simulator::Schedule(Seconds(0.0), &startBuildJump,mobileSinkNode.Get(0));
	Simulator::Schedule(Seconds(2.0), &update);
	Simulator::Schedule(Seconds(4.0),&setSimStartTime);
	Simulator::Schedule(Seconds(4.0),&DataToSink);
	//从路由发现到数据传输 结束
	//测试周期性邻居交换信息
	//Simulator::Schedule(Seconds(7.0),&update);  

	// Simulator::Schedule(Seconds(5.7),&updateNeighborTable);
	// Simulator::Schedule(Seconds(6.0),&buildRoutes);
	// Simulator::Schedule(Seconds(6.5),&printPaths); 
	// Simulator::Schedule(Seconds(6.0),&printNeighbor); 
	//测试结束
	// Simulator::Schedule(Seconds(1.0),&startRoute,senseNodes.Get(3),GetNodeIpv4Address(senseNodes.Get(3)));
	// Simulator::Schedule(Seconds(1.0),&testBuildNeighbor);   
	//step 16:仿真相关
	Simulator::Stop(Seconds(totalTime));
	Simulator::Run();

	Time delay;    
	uint32_t num;    
	flowMonitor->CheckForLostPackets ();   
	FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();    
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {    num++;                        
		delay=delay+i->second.delaySum;    
		num++;
	}
	cout<<"网络时延:"<<delay/num<<endl;//单跳的平均时延
	netDelay = delay/num;

	Simulator::Destroy();
	//step 17:数据收集
	finalRecord();
	return 0;
}





