/**
* 研究点一算法实现代码
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

#include <cmath>

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
#include "global.h"
#include "energy.h"
#include "final_record.h"
#include "GraphList.h"
using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE("GROUPROCOMARcript");

//函数声明
void createRoute();
Vector getMiddle(Vector one,Vector two);
void TransmitDataToRelay(uint32_t robotId,uint32_t id,Ipv4Address sourceAdr,Ipv4Address sinkAdr);
static inline int32_t ProcessRelayDataPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h);
void TransmitDataPacketByRelay(Ptr<Node> localNode, Ipv4Address sourceAdr,Ipv4Address sinkAdr);
void startRouteFindFrom(Ptr<Node> n);
double GetdistanOf2Nodes(Vector one,Vector two);
void buildMapIdAdr();
void initial();
void updateNeighbor();
void startRoute(Ptr<Node>thisNode,Ipv4Address srcAdr);
int ConnSum = 0;
int ConnTimes = 0;
void printPaths();
void findRoutes(int id);
void buildRoutes();
void updateNeighborTable();
void update();
void setUpdateDoneFlag(bool flag);

uint32_t findMinDistanceRelay(Ptr<Node> local);
double GetdistanFromRelay(Ptr<Node> srcN,uint32_t num);


void setRobotPosition(uint32_t id,Vector newPosition);
void changeColor(uint32_t id,bool flag);
void setRobotState(uint32_t id, bool flag);

string speed = "20";
bool isRelay = true;
clock_t lifeTime;
string mobilityModel="";
bool isLifeCycle;
bool isEntity;

map<int,vector<vector<int>>>routePaths;
map<uint32_t,uint32_t>nodeRobotMap;
Time netDelay;
/*可调参数*/
double robotSpeed = 80;
double dataInterval = 8.0; //发包间隔
double maxDis = 225.0;      //最大通信距离,要考虑区域面积和部署相关问题
double buildNeighborDone = 2.0;//新建立邻居表结束时间

double updateInterval = 8.0;//每隔多久更新一次邻居表
uint32_t nNodes = 20;
int maxNodes = 50;  // lyh
int node_num = (int)nNodes+1;
uint32_t nRelayNodes = 5;
// double neighborFindDone = 2.0;//当前节点建立邻居表的时间
double weakless = 0;//小规模实时更新,解决长链路和弱链路的问题.
double sendInterval = 0.8;
uint16_t relaySend = 0;
int maxPackets = 1000;
int send = 0;

// 定义UdpSocketFactory为socket的类型和socket的广播地址
 
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
double simStartRealTime;
double simFinishRealTime;
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
bool isStatic = false;
AnimationInterface *Panim = 0;
uint32_t firstDrainedNodeId=0;
bool firstDrainFlag=true;
uint32_t firstDrainedSinkId=0;
double firstDrainedNodeTime=0;
string thisSimPath;//simulation path
string exeName="GROUPROCOMARMain";
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
// static NodeContainer correctNodes;
static NodeContainer senseNodes;
static NodeContainer mobileSinkNode;
static NodeContainer relayNodes;
static NodeContainer netConnHelper;

//Net device container
NetDeviceContainer senseDevices;
NetDeviceContainer mobileSinkDevice;
NetDeviceContainer relayDevices;
//Ipv4 related container
Ipv4InterfaceContainer senseIfs;
Ipv4InterfaceContainer mobileSinkIf;
Ipv4InterfaceContainer relayIfs;
//Energy source pointer
double remaingJ[MAX_NUM];

//Connectivity index
int netConn[MAX_NUM];

set<uint32_t>remains;
// vector<uint32_t>totalNodes; 
GraphList graph;


static bool updateDone = false;

// TestPoint Switch
bool isTest=false;

// TestPoint function
void testEnergy();
void TestNodes();
void testIP();
void testNeighbor();
void testRobot();

//存普通节点ip和对应的ip地址容器
map<uint32_t,Ipv4Address>mapIdAdr;

map<uint32_t,bool>robotState;//false  自由态,可中继状态.
/*
 *数据发送到下一跳网关
 */
void TransmitDataPacket(Ptr<Node> localNode, Ipv4Address sourceAdr,Ipv4Address sinkAdr) {
	NS_LOG_LOGIC(
			std::endl<<TIME_STAMP_FUC <<sourceAdr <<" to "<<sinkAdr<<" by "
			<<GetNodeIpv4Address(localNode));
			
	if(isTest == true)
		cout<<"TransmitDataPacket "<<sourceAdr <<" to "<<sinkAdr<<" by "
			<<GetNodeIpv4Address(localNode)<<endl;
	
	if (CheckRemainingJ(localNode)){
		Ipv4Address localAdr = GetNodeIpv4Address(localNode);
		uint32_t localId = localNode->GetId();
		uint32_t gatewayId = 1024;
		//根据邻居表获取下一跳的id
		for (int i = 0; i < graph.m_vCount; ++i){
			int curId = graph.m_vVertex[i].id;
			if((uint32_t)curId != localId)
				continue;
			Node1* edge = graph.m_vVertex[i].nodeNext;
			if(edge != NULL)
				gatewayId = (uint32_t)edge->id;
			break;
		}
		
	    if(isTest == true)
		{
			cout<<"下一跳节点选择:" <<gatewayId<<endl;
		}
		if (localAdr == sourceAdr) {
			NS_LOG_LOGIC(
					TIME_STAMP_FUC<<sourceAdr<<"(sense) sent a data packet to "<<sinkAdr);
			totalBytesGenerated += pktSize;
		}
		Vector location_1;//当前节点
		Vector location_2;//下一跳节点
		double distance_1=0.0; 
		Ptr<Node>gateTmp = GetNodePtrFromGlobalId(gatewayId,senseNodes,mobileSinkNode);
		if(gatewayId != 1024){
			//checded
			if (isStatic){
				location_1 = localNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
				location_2 = gateTmp->GetObject<ConstantPositionMobilityModel>()->GetPosition();
			}else{
				location_1 = localNode->GetObject<MobilityModel>()->GetPosition();
				location_2 = gateTmp->GetObject<MobilityModel>()->GetPosition();
			}

			distance_1 = GetdistanOf2Nodes(location_1,location_2);//接收广播包到ms的距离
			// 算法一:重复链路增强算法
			if(distance_1 < maxDis){//解除机器人占用的条件
				if(nodeRobotMap.find(localId) != nodeRobotMap.end()){
					// cout<<"解除占用------------------>"<<endl;
					robotState[nodeRobotMap[localId]]=false;
					// uint32_t idTmp = nodeRobotMap[localId];
					nodeRobotMap.erase(localId);
					Simulator::Schedule(Seconds(0.0),&changeColor,nodeRobotMap[localId],true);
					
				}
			}
			
		}

		// cout<<"gatewayId:"<<gatewayId<<" ,distance:"<<distance_1<<endl;
		//算法二：移动中继到两者位置中心
		if(gatewayId == 1024 || distance_1>maxDis){
			// cout<<"请求中继----->"<<endl;
			Vector mid = getMiddle(location_1,location_2);
			uint32_t id = findMinDistanceRelay(localNode);
			//checded
			// cout<<"--------------------------->"<<id<<endl;
			if(id == 1024){
				// id = nNodes + rand()%nRelayNodes + 1; //0到nCandidates-1的范围的整数
				// cout<<id<<endl;
				return;
			}
			Vector locationRobot = GetNodePtrFromGlobalId(id,relayNodes,mobileSinkNode)->GetObject<ConstantPositionMobilityModel>()->GetPosition();
			double distance1 = GetdistanOf2Nodes(location_1,locationRobot);

			// cout<<"===找到的中继ID是="<<id<<endl;
			//链路保持算法
			if(nodeRobotMap.find(localId) != nodeRobotMap.end()){
				// cout<<"存在------------------>"<<endl;
				Simulator::Schedule(Seconds(distance1/robotSpeed),&setRobotPosition,id,mid);
				Simulator::Schedule(Seconds(distance1/robotSpeed),&changeColor,id,false);
				//数据包发到中继
				Simulator::Schedule(Seconds(distance1/robotSpeed+0.2),&TransmitDataToRelay,id,gatewayId,sourceAdr,sinkAdr);
				Simulator::Schedule(Seconds(distance1/robotSpeed+0.5),&changeColor,id,true);//中继完成,状态切换
			}
			if(nodeRobotMap.find(localId) != nodeRobotMap.end()){
				return;
			}

			robotState[id]=true;

			if(nodeRobotMap.find(localId) == nodeRobotMap.end()){
				nodeRobotMap.insert(pair<uint32_t,uint32_t>(localId,id));
			}


			Simulator::Schedule(Seconds(distance1/robotSpeed),&setRobotPosition,id,mid);
			Simulator::Schedule(Seconds(0.0),&changeColor,id,false);
			//数据包发到中继
			Simulator::Schedule(Seconds(distance1/robotSpeed+0.2),&TransmitDataToRelay,id,gatewayId,sourceAdr,sinkAdr);
			Simulator::Schedule(Seconds(distance1/robotSpeed+0.5),&changeColor,id,true);//中继完成,状态切换

			return;
		}

		pktType pktType = data_Type;
		//通过传过来的路由表获取
		Ipv4Address gatewayAdr = mapIdAdr[gatewayId];
		Ptr<Node>nodePtr = GetNodePtrFromGlobalId(gatewayId,senseNodes,mobileSinkNode);

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
 *数据发送到下一跳网关
 */
void TransmitDataPacketByRelay(Ptr<Node> localNode, Ipv4Address sourceAdr,Ipv4Address sinkAdr) {
	NS_LOG_LOGIC(
		std::endl<<TIME_STAMP_FUC <<sourceAdr <<" to "<<sinkAdr<<" by "
		<<GetNodeIpv4Address(localNode));
	Ipv4Address localAdr = GetNodeIpv4Address(localNode);
	uint32_t localId = localNode->GetId();
	uint32_t gatewayId = 1024;
	//根据邻居表获取下一跳的id
	for (int i = 0; i < graph.m_vCount; ++i){
		int curId = graph.m_vVertex[i].id;
		if((uint32_t)curId != localId)
			continue;
		Edge* edge = graph.m_vVertex[i].next;
		while(edge){
			if(edge!=NULL){
				gatewayId = edge->id;
				break;
			}
			edge = edge->next;
		}
		break;
	}
	pktType pktType = data_Type;
	//通过传过来的路由表获取
	// cout<<"--------------"<<gatewayId<<endl;
	Ipv4Address gatewayAdr = mapIdAdr[gatewayId];
	Ptr<Node>nodePtr = GetNodePtrFromGlobalId(gatewayId,senseNodes,mobileSinkNode);
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
}

void TransmitDataToRelay(uint32_t robotId,uint32_t id,Ipv4Address sourceAdr,Ipv4Address sinkAdr){
	Ptr<Node>localNode = GetNodePtrFromGlobalId(robotId,relayNodes,mobileSinkNode);
	Ipv4Address gatewayAdr = mapIdAdr[id];
	// cout<<"传输到的下一跳的ip:"<<gatewayAdr<<endl;
	pktType pktType = data_Type;
	Ptr<Packet> dataPkt = Create<Packet>(pktSize);
	relaySend += pktSize;
	Ipv4Header h;
	h.SetDestination(sinkAdr);
	h.SetSource(sourceAdr);
	h.SetIdentification(pktType);
	dataPkt->AddHeader(h);
	InetSocketAddress gateAdr = InetSocketAddress(gatewayAdr, 80);
	Ptr<Socket> srcSoc = Socket::CreateSocket(localNode, tid);
	srcSoc->Connect(gateAdr);
	srcSoc->Send(dataPkt);
}

/*
 * 计算网络连通度函数
 * lyh 
 */
void getNetConn(){
	// 清除netConn
	for(int m = 0; m < maxNodes; m++)
		netConn[m] = 0;
	// 首先遍历所有节点，将和MBase连接的节点的netConn置1 // 是否需要区分MRelay和MNode执行不同的MobilityModel
	Ptr<Node>msNode = mobileSinkNode.Get(0);
	Vector ms_location = msNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
	for(NodeContainer::Iterator i = netConnHelper.Begin(); i != netConnHelper.End(); i++)
	{
		Ptr<Node> thisNode = *i;
		Vector node_location;
		node_location = thisNode->GetObject<MobilityModel>()->GetPosition();
		double distance = GetdistanOf2Nodes(node_location, ms_location);
		if(distance < maxDis)
			netConn[thisNode->GetId()] = 1;		
	}
	// 循环遍历节点，看已经连通的节点是否和其在通信范围内，如有存在，则改变数组
	int times = 0;
	while(times < 2){
		times++;
		for(NodeContainer::Iterator j = netConnHelper.Begin(); j != netConnHelper.End(); j++){
			Ptr<Node> preNode = *j;
			if(netConn[preNode->GetId()] != 1)
				continue;
			for(NodeContainer::Iterator s = netConnHelper.Begin(); s != netConnHelper.End(); s++)
			{
				Ptr<Node> backNode = *s;
				if(preNode->GetId() == backNode->GetId() || netConn[backNode->GetId()] == 1)
					continue;				
				Vector pre_location = preNode->GetObject<MobilityModel>()->GetPosition();
				Vector back_location =  backNode->GetObject<MobilityModel>()->GetPosition();
				double distance_pb = GetdistanOf2Nodes(pre_location, back_location);				
				if( distance_pb < maxDis){
					netConn[backNode->GetId()] = 1;
				}				
			}
		}
	}
	int conn = 0;
	for (int index = 0; index < (int)nNodes; index++)
		conn += netConn[index];
	ConnSum += conn;
	ConnTimes += 1;
	cout<<"网络连通性为" <<conn <<endl;	
	Simulator::Schedule(Seconds (10.0), &getNetConn);
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
				cout<<TIME_STAMP_FUC<<"datatosink--数据发出,节点id为:"<<thisNode->GetId()<<",num is-->"<<send<<endl;
				Simulator::Schedule(Seconds(interval), &TransmitDataPacket,
						thisNode, srcAdr, dstAdr);
				send++;
				interval += sendInterval;
			}
	}
	if(send == maxPackets){
		lifeTime = Simulator::Now().GetSeconds();
		Simulator::Stop(Seconds(5.0));
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
		NS_LOG_LOGIC(TIME_STAMP_FUC<<
					GetNodeIpv4Address(thisNode)<<" received a data packet from "
					<<h.GetSource()<<" to "<<h.GetDestination()<<srcAdr<<"to"<<dstAdr);
		double interval = RandomDoubleVauleGenerator(0.0, 0.5);
		Simulator::Schedule(Seconds(interval), &TransmitDataPacket, thisNode,srcAdr, dstAdr);
		return 0;
	}
	NS_LOG_INFO(TIME_STAMP_FUC<<
						GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
	return 0;
}

static inline int32_t ProcessRelayDataPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h){
	//分析包的source和destination
	Ipv4Address srcAdr = h.GetSource();
	Ipv4Address dstAdr =h.GetDestination();
	Ipv4Address localAdr = GetNodeIpv4Address(thisNode);

	if (localAdr != dstAdr) {//gateway node received the packet, transmit it to the next gateway node
		//产生随机的发包间隔
		NS_LOG_LOGIC(TIME_STAMP_FUC<<
					GetNodeIpv4Address(thisNode)<<" received a data packet from "
					<<h.GetSource()<<" to "<<h.GetDestination()<<srcAdr<<"to"<<dstAdr);
		double interval = RandomDoubleVauleGenerator(0.0, 0.5);
		// cout<<"找下一跳中继邻居进行数据发送--->"<<endl;
		Simulator::Schedule(Seconds(interval), &TransmitDataPacketByRelay, thisNode,srcAdr, dstAdr);
		return 0;
	}
	NS_LOG_INFO(TIME_STAMP_FUC<<
						GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
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
			Ptr<Node> thisNode = socket->GetNode();	//这里这个socket是received socket，不是发送的那个
			Ipv4Header h;
			packet->PeekHeader(h);
			packet->RemoveHeader(h);
			pType = pktType(h.GetIdentification());
			nType = CheckNodeType(thisNode,senseNodes,relayNodes,mobileSinkNode.Get(0));
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
							// cout<<"插入邻居表,源节点为------->"<<GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,mobileSinkNode)->GetId()<<"目的节点为------>"<<thisNode->GetId()<<endl;
							Ptr<Node>sourceNode = GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,relayNodes,mobileSinkNode);
							//TODO
							Ptr<Node>msNode = mobileSinkNode.Get(0);
							Vector location2;
							Vector location1;
							//checked
							if (isStatic){
								location2 = sourceNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
								location1 = msNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
							}else{
								location2 = sourceNode->GetObject<MobilityModel>()->GetPosition();
								location1 = msNode->GetObject<MobilityModel>()->GetPosition();
							}
							
							double weight = 0;// GetdistanOf2Nodes(location2,location1);
							graph.addNewEdgeToList(GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,relayNodes,mobileSinkNode)->GetId(),weight,mobileSinkNode.Get(0)->GetId());
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
						uint32_t curId = thisNode->GetId();
						//发送广播包的节点
						Ipv4Address src = h.GetSource();
						Ptr<Node>node = GetNodePtrFromIpv4Adr(src,senseNodes,relayNodes,mobileSinkNode);
						uint32_t id = node->GetId();
						// cout<<"当前发送广播包的节点ip为："<<src<<"当前发送广播包的节点id为："<<id<<"接收到广播包的id为："<<curId<<endl;
						//checked
						Vector location1;
						Vector location2;
						if (isStatic){
							location1 = thisNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
							location2 = node->GetObject<ConstantPositionMobilityModel>()->GetPosition();
						}else{
							location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
							location2 = node->GetObject<MobilityModel>()->GetPosition();
						}
						Ptr<Node>msNode = mobileSinkNode.Get(0);
						Vector location;
						if(isStatic){
							location = msNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
						}else{
							location = msNode->GetObject<MobilityModel>()->GetPosition();
						}
						 
						double distance1 = GetdistanOf2Nodes(location1,location);//接收广播包到ms的距离
						double distance2 = GetdistanOf2Nodes(location2,location);
						if(distance1 < distance2){
							//TODO,权重效应
							// cout<<"插入邻居表,源节点为------->"<<id<<"目的节点为------>"<<curId<<endl;
							graph.addNewEdgeToList(id,distance1,curId);
						}
					}
					default: {
						break;
					}
				}
				break;
			}
			case relay_Type: {	
				switch (pType) {
					//缺少数据包包逻辑
				case data_Type: {///收到data_Type的packet，将执行中继
					ProcessRelayDataPacket(thisNode, packet, h);
					break;
				}
				default:
					break;
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
	LogComponentEnable("GROUPROCOMARcript", LOG_LEVEL_DEBUG);
	LogComponentEnable("Energy", LOG_LEVEL_DEBUG);
	LogComponentEnable("AdhocWifiMac", LOG_LEVEL_DEBUG);
}

/*
 * 基本网络设置
 */
void netSet(){
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
			StringValue("2200"));
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
			StringValue("2200"));
	Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
			StringValue(phyMode));
}

/*
 * 创建节点
 */
void createNode(){
	
	senseNodes.Create(nNodes);
	std::string traceFile = "scratch/"+mobilityModel+"/rocomar_speed"+speed+".ns_movements";
	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install ();
	mobileSinkNode.Create(1);
	relayNodes.Create(nRelayNodes);
	NS_LOG_DEBUG("Create nodes done!");
	netConnHelper.Add(senseNodes);
	netConnHelper.Add(relayNodes);
}
/*
 * 创建移动模型并安装到节点
 */
void createMobilityModel(){
	MobilityHelper mobility;
		ObjectFactory pos1;
		pos1.SetTypeId ("ns3::RandomRectanglePositionAllocator");
		pos1.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
		pos1.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
		Ptr<PositionAllocator> taPositionAlloc1 = pos1.Create ()->GetObject<PositionAllocator> ();
		mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(mobileSinkNode);
	mobileSinkNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));
	//为中继安装移动模型重新指定中继节点的位置.
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	// mobility.Install(mobileSinkNode);
	mobility.Install(relayNodes);
    vector <Ptr<ConstantPositionMobilityModel>> cpmm(nRelayNodes);
    for (uint32_t i = 0; i < nRelayNodes; i++)
 	cpmm[i] = relayNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
    cpmm[0]->SetPosition(Vector(400, 400, 0));
	cpmm[1]->SetPosition(Vector(300, 500, 0));
	cpmm[2]->SetPosition(Vector(500, 300, 0));
	cpmm[3]->SetPosition(Vector(500, 700, 0));
	cpmm[4]->SetPosition(Vector(700, 500, 0));

	mobileSinkNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));
}



/*
 * 创建wifi通信设备
 */
void createWifiDevice(){
	stringstream ssi;
	//Create devices
	WifiHelper wifi;
	wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
	// Wifi PHY 
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	wifiPhy.Set("RxGain", DoubleValue(-10));
	wifiPhy.Set("TxGain", DoubleValue(offset + Prss));
	wifiPhy.Set("CcaMode1Threshold", DoubleValue(0.0));
	// wifi channel 同时设置最大通信距离 
	YansWifiChannelHelper wifiChannel;
	wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
			DoubleValue(maxDis));			
/// create wifi channel
	Ptr<YansWifiChannel> wifiChannelPtr = wifiChannel.Create();
	wifiPhy.SetChannel(wifiChannelPtr);
// Add a non-QoS upper MAC, and disable rate control
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
			StringValue(phyMode), "ControlMode", StringValue(phyMode));			
/// Set it to ad-hoc mode
	wifiMac.SetType("ns3::AdhocWifiMac");
// install PHY + MAC
	senseDevices = wifi.Install(wifiPhy, wifiMac, senseNodes);
	relayDevices = wifi.Install(wifiPhy, wifiMac, relayNodes);
	mobileSinkDevice = wifi.Install(wifiPhy, wifiMac, mobileSinkNode);
	ssi<<thisSimPath<<exeName;
	ssi.str("");
	ssi.clear();
	NS_LOG_DEBUG("Create devices done!");
	NS_LOG_DEBUG("Set pcap and anmi done!");
}
/*
 * 安装网络协议栈
 */
void installInternetStack(){
	InternetStackHelper stack2;
	
	stack2.Install (senseNodes);
	
	stack2.Install (mobileSinkNode);
	stack2.Install (relayNodes);

	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	senseIfs = ipv4.Assign(senseDevices);
	
	mobileSinkIf = ipv4.Assign(mobileSinkDevice);
	relayIfs = ipv4.Assign(relayDevices);

	NS_LOG_DEBUG("Install Internet stack done!");
}

void createSocketCallBack(){
	vector<Ptr<Socket> > recvSocket(nNodes+1+nRelayNodes);
	recvSocket[0] = Socket::CreateSocket(mobileSinkNode.Get(0),tid);
	recvSocket[0]->Bind(InetSocketAddress(mobileSinkIf.GetAddress(0), 80));
	recvSocket[0]->SetRecvCallback(MakeCallback(&RecvPacketCallback));

	for (uint32_t i = 1; i < nNodes+1; i++) {
		recvSocket[i] = Socket::CreateSocket(senseNodes.Get(i-1), tid);
		recvSocket[i]->Bind(InetSocketAddress(senseIfs.GetAddress(i-1), 80));
		recvSocket[i]->SetRecvCallback(MakeCallback(&RecvPacketCallback));
	}
	//机器人节点设置回调
	for (uint32_t i = nNodes+1; i < nNodes+nRelayNodes+1; i++) {
		recvSocket[i] = Socket::CreateSocket(relayNodes.Get(i-1-nNodes), tid);
		recvSocket[i]->Bind(InetSocketAddress(relayIfs.GetAddress(i-1-nNodes), 80));
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
		if(isEntity){
			Panim = new AnimationInterface("/home/wsn/sim_temp/ROCOMAR/RWP/output.xml");
		}else{
			if(isRelay){
				Panim = new AnimationInterface("/home/wsn/sim_temp/ROCOMAR/RELAY/output.xml");
			}else{
				Panim = new AnimationInterface("/home/wsn/sim_temp/ROCOMAR/"+mobilityModel+"/output.xml");
			}
			
		}
		Panim->SetMaxPktsPerTraceFile(50000000);
		Panim->UpdateNodeColor(mobileSinkNode.Get(0), 0, 255, 0);
		Panim->UpdateNodeSize(mobileSinkNode.Get(0)->GetId(), 10.0, 10.0);
		for (NodeContainer::Iterator i = senseNodes.Begin();
				i != senseNodes.End(); i++) {
			Ptr<Node> n = *i;
			Panim->UpdateNodeSize(n->GetId(), 2.0, 2.0);
		}

		//机器人颜色
		uint32_t cnt = 0;
		for (NodeContainer::Iterator i = relayNodes.Begin();i != relayNodes.End(); i++) {
			Ptr<Node> n = *i;
			Panim->UpdateNodeSize(n->GetId(), 3.0, 3.0);
			Panim->UpdateNodeColor(relayNodes.Get(cnt++), 0, 200, 255);
		}
	}
	NS_LOG_DEBUG("Set anmi done!");
}
void finalRecord(){
	std::cout<<"\n\n***** OUTPUT *****\n\n";
	std::stringstream ss;			
	ss << "/home/wsn/sim_temp/ROCOMAR/"<<mobilityModel<<"/"<<speed<<"-"<<maxDis<<"-"<<maxPackets<<".record";		
	std::ofstream of(ss.str().c_str());
	simFinishRealTime = Simulator::Now().GetSeconds();

		std::cout << "网络时延: " << netDelay<< "\n";
		std::cout<<"PDR指标:"<<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
		std::cout<<"网络吞吐量:"<<totalBytesGathered*8.0/((simFinishRealTime-simStartRealTime)*1000.0)<<"(kbps)"<<endl;
		std::cout<<"网络平均连通度:"<<ConnSum*1.0/(ConnTimes * nNodes)<<endl;
		std::cout<<"totalBytesGathered: "<<totalBytesGathered<<" totalBytesGenerated: "<<totalBytesGenerated<<endl;
		ss << std::endl << "网络时延:" << netDelay<<"(ms)"<< std::endl;
		ss << std::endl << "网络吞吐量: " << totalBytesGathered*8.0/((simFinishRealTime-simStartRealTime)*1000.0)<<"(kbps)"<<endl;
		ss << std::endl << "PDR指标:" <<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
		ss << std::endl <<"网络平均连通度:"<<ConnSum*1.0/(ConnTimes * nNodes)<<endl;

	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();
}

double GetdistanOf2Nodes(Vector one,Vector two) {
	return std::sqrt((one.x - two.x)*(one.x - two.x) + (one.y - two.y)*(one.y - two.y));
}



void buildMapIdAdr(){
	mapIdAdr.clear();
	pair<uint32_t,Ipv4Address>pairNode;
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		uint32_t id = thisNode->GetId();
		Ipv4Address sourceAdr = GetNodeIpv4Address(thisNode);
		pairNode.first = id;
		pairNode.second = sourceAdr;
		mapIdAdr.insert(pairNode);
	}

	pair<uint32_t,Ipv4Address>pairNode1;
	for (NodeContainer::Iterator i = relayNodes.Begin(); i != relayNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		uint32_t id = thisNode->GetId();
		Ipv4Address sourceAdr = GetNodeIpv4Address(thisNode);
		pairNode1.first = id;
		pairNode1.second = sourceAdr;
		mapIdAdr.insert(pairNode1);
	}
	//插入目的地
	Ptr<Node>msNode = mobileSinkNode.Get(0);
	mapIdAdr.insert(pair<uint32_t,Ipv4Address>(msNode->GetId(),GetNodeIpv4Address(msNode)));
}

void initialRobotState(){
	robotState.clear();
	pair<uint32_t,bool>pairNode1;
	for (NodeContainer::Iterator i = relayNodes.Begin(); i != relayNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		uint32_t id = thisNode->GetId();
		pairNode1.first = id;
		pairNode1.second = false;
		robotState.insert(pairNode1);
	}
}


void initial(){
	for (NodeContainer::Iterator i = senseNodes.Begin();
			i != senseNodes.End(); i++) {
		Ptr<Node>node = *i;
		remains.insert(node->GetId());
	}
	//TODO
	graph.m_vCount = nNodes;  //nNodes+nRelayNodes;
	graph.makeVertexArray();
	nodeRobotMap.clear();
	cout<<"neighbor struct initial done!!"<<endl;
	for(int j=0; j<maxNodes; j++){
		netConn[j] = 0;
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
}

void updateNeighbor(){
	double interval = 0;	//产生随机的发包间隔
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
		Simulator::Schedule(Seconds(interval), &startRoute,thisNode,srcAdr);
		interval += 0.1;
		graph.clearNode(thisNode->GetId());
	}
}





void updateNeighborTable(){
	graph.updateArray();
}


/*定时任务,定时交换邻居信息,更新邻居表*/
void update(){	
	updateDone = false;
	updateNeighbor();
	Simulator::Schedule(Seconds(buildNeighborDone), &setUpdateDoneFlag,true);
	Simulator::Schedule(Seconds(buildNeighborDone), &createRoute);
	if(isTest == true)
		Simulator::Schedule(Seconds(buildNeighborDone + 0.1), &testNeighbor);
	Simulator::Schedule(Seconds(updateInterval), &update);	
}

void createRoute(){
	graph.createRoute();
}



void setSimStartTime(){
	simStartRealTime = Simulator::Now().GetSeconds();
}


void setUpdateDoneFlag(bool flag){
	updateDone = flag;
}

// 找到最近距离的中继节点
uint32_t findMinDistanceRelay(Ptr<Node> local){
	uint32_t min = 5000;
	bool flag = false;
	uint32_t index = 0;
	double distance;
	for(uint32_t i=0;i<relayNodes.GetN();i++){
		if(robotState[relayNodes.Get(i)->GetId()]==true){
			continue;
		}
		distance = GetdistanFromRelay(local,i);
		if(distance<maxDis){
			flag = true;
		}
		if(min > distance){
			min = distance;
			index = i;
		}
	}
	return flag==true?nNodes+index+1:1024;	
}

//checked
double GetdistanFromRelay(Ptr<Node> srcN,uint32_t num) {
	Vector srcP;
	if(!isStatic){
		Ptr<MobilityModel> srcCpmm = srcN->GetObject<MobilityModel>();
		srcP = srcCpmm->GetPosition();
	}else{
		Ptr<ConstantPositionMobilityModel> srcCpmm = srcN->GetObject<ConstantPositionMobilityModel>();
		srcP = srcCpmm->GetPosition();
	}
	Ptr<Node>relayNode = relayNodes.Get(num);
	Ptr<ConstantPositionMobilityModel> remCpmm = relayNode->GetObject<ConstantPositionMobilityModel>();
	Vector remP = remCpmm->GetPosition();
	return std::sqrt((srcP.x - remP.x)*(srcP.x - remP.x) + (srcP.y - remP.y)*(srcP.y - remP.y));
}

/*
* TestPoint1 : 测试节点的位置
*/
void TestNodes(){
	clock_t timeTestNode;
	timeTestNode = clock();
	cout<<"当前仿真时刻"<<timeTestNode-simStartRealTime<<endl;
	cout<<"移动节点"<<endl;
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
			i++) {
				Ptr<Node> thisNode = *i;
				Vector location = thisNode->GetObject<MobilityModel>()->GetPosition();
				cout<<"节点ID： "<<thisNode->GetId()<<",   x:  "<<location.x<<",   y:  "<<location.y<<",   z:  "<<location.z<<endl;
			}
	cout<<"移动中继"<<endl;
	for (NodeContainer::Iterator i = relayNodes.Begin(); i != relayNodes.End();
			i++) {
				Ptr<Node> thisNode = *i;
				Vector location = thisNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
				cout<<"中继ID： "<<thisNode->GetId()<<",   x:  "<<location.x<<",   y:  "<<location.y<<",   z:  "<<location.z<<endl;
			}
	cout<<"Sink节点"<<endl;
	Ptr<Node> thisNode = mobileSinkNode.Get(0);
	Vector location;
	location = thisNode->GetObject<MobilityModel>()->GetPosition();
	cout<<"sink ID： "<<thisNode->GetId()<<",   x:  "<<location.x<<",   y:  "<<location.y<<",   z:  "<<location.z<<endl;
	
}

// 测试能量
void testEnergy(){
	cout<<"能量测试"<<endl;
	for(uint32_t i = 0; i < nNodes; i++)
		cout<<"节点"<<i<<"的剩余能量为： "<<remaingJ[i]<<endl;

}

// 测试ip
void testIP(){
	cout<<"senseNodes 的 ip地址如下"<<endl;
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
		i++) {
			Ptr<Node> thisNode = *i;
			Ipv4Address iv = GetNodeIpv4Address(thisNode);
			cout<<thisNode->GetId()<<"---"<<iv<<endl;
	}
	cout<<"relayNodes 的 ip地址如下"<<endl;
	for (NodeContainer::Iterator i = relayNodes.Begin(); i != relayNodes.End();
		i++) {
			Ptr<Node> thisNode = *i;
			Ipv4Address iv = GetNodeIpv4Address(thisNode);
			cout<<thisNode->GetId()<<"---"<<iv<<endl;
	}
	cout<<"接收器 的 ip地址如下"<<endl;
	Ipv4Address iv1 = GetNodeIpv4Address(mobileSinkNode.Get(0));
	cout<<mobileSinkNode.Get(0)->GetId()<<"---"<<iv1<<endl;

}

void testNeighbor(){
	TestNodes();
	testEnergy();
	testRobot();
	cout<<"链式表  ： 展示形式为  当前节点id->邻居节点id(weight)" <<endl;
	graph.print();
	graph.printRoute();
}

void testRobot(){
	// uint32_t n = nNodes + 1;
	cout<<"移动中继"<<endl;
	for (NodeContainer::Iterator i = relayNodes.Begin(); i != relayNodes.End();	i++) {
				Ptr<Node> thisNode = *i;
				Vector location = thisNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
				cout<<"中继ID： "<<thisNode->GetId()<<"中继状态"<<robotState[thisNode->GetId()]<<",   x:  "<<location.x<<",   y:  "<<location.y<<",   z:  "<<location.z<<endl;
		}

}

void setRobotPosition(uint32_t id,Vector newPosition){
	GetNodePtrFromGlobalId(id,relayNodes,mobileSinkNode)->GetObject<ConstantPositionMobilityModel>()->SetPosition(newPosition);
}


void setRobotState(uint32_t id, bool flag){
	robotState[id] = flag;
}

void changeColor(uint32_t id,bool flag){
	if(id == 0){
		return;
	}
	if(!flag){//false的时候,更新为繁忙状态
		Panim->UpdateNodeColor(GetNodePtrFromGlobalId(id,relayNodes,mobileSinkNode), 200, 250, 0);
	}else{
		Panim->UpdateNodeColor(GetNodePtrFromGlobalId(id,relayNodes,mobileSinkNode), 0, 200, 255);
	}
	
}



Vector getMiddle(Vector one,Vector two){
	Vector t;
	t.x = (one.x+two.x)/2;
	t.y = (one.y+two.y)/2;
	return t;
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
	cmd.AddValue("isRelay","is relay", isRelay);
    cmd.AddValue("isTest", "open Test Point or not", isTest);
	cmd.AddValue("nRelayNodes", "Number of Nodes", nRelayNodes);
	cmd.AddValue("totalTime", "Simulation time length", totalTime);
	cmd.AddValue("simStartTime", "Simulation start time", simStartTime);
	cmd.AddValue("maxDis", "max distance to transmit", maxDis);
	cmd.AddValue("maxPackets", "max distance to transmit", maxPackets);
	cmd.AddValue("initialJ","Initial energy of each BaiscEnergySource", initialJ);
	cmd.AddValue("pktSize","Size of data packe",pktSize);
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
	cmd.AddValue("speed","Speed of nodes",speed);
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
	getNetConn();
	buildMapIdAdr();
	cout<<"build idAdrMap done!"<<endl;
	initial(); 
	if(isTest == true){
		testIP();
		TestNodes();
	}
	
	Simulator::Schedule(Seconds(0.0),&update);   
	Simulator::Schedule(Seconds(2.0),&setSimStartTime);
	Simulator::Schedule(Seconds(2.2),&DataToSink);
	
	// Simulator::Schedule(Seconds(3.0),&updateRobotNeighbor);

	// Simulator::Schedule(Seconds(0.0),&update);   
	// Simulator::Schedule(Seconds(5.0),&printNeighbor);
	// Simulator::Schedule(Seconds(8.0),&printPaths);
	// Simulator::Schedule(Seconds(10.0),&DataToSink);
	//step 16:仿真相关
	Simulator::Stop(Seconds(totalTime));
	Simulator::Run();

	Time delay;    
	long num;    
	flowMonitor->CheckForLostPackets ();   
	FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();    
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {    
        //num++;	
		delay=delay+i->second.delaySum;    
		num++;
		// cout<<num<<endl;
	}
	cout<<"网络时延:"<<delay/num<<endl;//单跳的平均时延
	netDelay = delay/num;
	// cout<<"网络生命周期:"<<TIME_STAMP_FUC<<"(s)"<<endl;
	//step 17:数据收集
	finalRecord();
	Simulator::Destroy();

	return 0;
}





