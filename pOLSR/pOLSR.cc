/**
* EXOR算法:
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
NS_LOG_COMPONENT_DEFINE("pOLSRScript");

//函数声明
double GetdistanFromMSink(Ptr<Node> srcN);
void startRouteFindFrom(Ptr<Node> n);
double GetdistanOf2Nodes(Vector one,Vector two);
double GetdistanOf2Id(uint32_t id1,uint32_t id2);
void buildMapIdAdr();
void ContinueSptRouting(Ptr<Node> n, uint32_t jumps, Ipv4Address sinkAdr);
uint32_t AnalyzeSptPacket(stringstream &ss, Ipv4Address &source);
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
void testPrintNeighbor();//数据传输之前,测试邻居表当前状态
void UpdateInstSpeed(double interval);

Time netDelay;
clock_t lifeTime;
string mobilityModel="";
bool isLifeCycle=false;
string moveSpeed="";
bool isSpeed=false;

/*可调参数*/
double dataInterval = 8; //发包间隔
double updateInterval = 12;//每隔多久更新一次邻居表
double maxDis = 150.0;     // 需要修改    //最大通信距离,要考虑区域面积和部署相关问题
double buildNeighborDone = 4.2;//新建立邻居表结束时间,测试最大通信距离225米时候,需要2秒,和节点数也有关
uint32_t nNodes = 20;
int maxPackets = 1500;
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
int UnitPacketNum = 200;
// int SendPacketsNum[(int)nNodes] = {0};
stringstream ssi;
//Topology
double maxX;//x length of the scenario
double maxY;
double mSinkTraceY;
bool gridTest=true;
//Data packet
uint32_t pktSize = 1024;//1000字节  9个节点 包含6个sense和3个sink，一共发10次 9*10*1000=90000,一轮发90000.
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
vector<vector<double> >etx(25,vector<double>());
double old_x[25];
double old_y[25];
double new_x[25];
double new_y[25];
vector<vector<double> >instSpeed(25,vector<double>());
vector<vector<double> >recv(25,vector<double>());
vector<vector<double> >trans(25,vector<double>());
vector<vector<double> >recvRatio(25,vector<double>());
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
string exeName="EXORMain";
string simFolderName="/home/wsn/sim_temp/pOLSR";
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
set<uint32_t>remains;
// vector<uint32_t>totalNodes; 
GraphList graph;
int send = 0;
static bool updateDone = false;
//存普通节点ip和对应的ip地址容器
map<uint32_t,Ipv4Address>mapIdAdr;
/*
 *数据发送到下一跳网关
 */
void TransmitDataPacket(Ptr<Node> localNode, Ipv4Address sourceAdr,
		Ipv4Address sinkAdr) {
		//	cout<<"TransmitDataPacket"<<endl;
	NS_LOG_LOGIC(
			std::endl<<TIME_STAMP_FUC <<sourceAdr <<" to "<<sinkAdr<<" by "
			<<GetNodeIpv4Address(localNode));
	// NS_LOG_LOGIC(TIME_STAMP_FUC<<"CheckRemainingJ for "<<localAdr<<" passed!");
	if (CheckRemainingJ(localNode)) {
		// if(!updateDone){
		// 	return;
		// }
		Ipv4Address localAdr = GetNodeIpv4Address(localNode);
		if (localAdr == sourceAdr) {
			NS_LOG_INFO(
					TIME_STAMP_FUC<<sourceAdr<<"(sense) sent a data packet to "<<sinkAdr);
			totalBytesGenerated += pktSize;
		}
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
			Edge* edge = graph.m_vVertex[i].next;
			/*针对EXOR,修改发送策略*/
			while(edge){
				distance1 = GetdistanOf2Id(curId,(uint32_t)edge->id);
				if(distance1 <= maxDis){
					flag = true;
				}
				if(!flag)
						gatewayId = 1024;
				// double weight = edge->weight;  
				double weight = etx[curId][edge->id];
				cout<<curId<<"的下一跳选择： "<< edge->id << "  此时etx为"<<weight<<endl;
				if(weight < minWeight ){
					minWeight = weight;
					gatewayId = (uint32_t)edge->id;
				}
				edge = edge->next;
				//break;
			}
			cout<< "选择节点:" << gatewayId<<endl;
			break;
		}
		if(gatewayId == 1024){
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
 * 全体senseNodes将按一定的数据生成率向某一个sinkNode发送感知到的数据
 */
void DataToSink() {
	double interval = 0;	//产生随机的发包间隔
	// if(!updateDone){
	// 	cout<<"邻居表未更新完成--------------------------------------------------"<<endl;
	// 	Simulator::Schedule(Seconds(0.5), &DataToSink);
	// 	return;
	// }
	 UpdateInstSpeed(5.0);
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
			i++) {
		Ptr<Node> thisNode = *i;
		//thisNode->GetId();
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
			if (!CheckRemainingJ(thisNode)) {
				continue;
			} else {
				//找到下一跳ip,将当前节点数据包传输到下一跳.
				Ipv4Address dstAdr = GetNodeIpv4Address(mobileSinkNode.Get(0));
				//if(SendPacketsNum[thisNode->GetId()] > 0)
				Simulator::Schedule(Seconds(interval), &TransmitDataPacket,
						thisNode, srcAdr, dstAdr);
				//totalBytesGenerated += pktSize;
				//SendPacketsNum[thisNode->GetId()]-=1;
				send++;
				cout<<TIME_STAMP_FUC<<"data num is-->"<<send<<endl;
				interval += 0.2;
			}		
	}

	if(send == maxPackets){
		// simStopTime = Simulator::Now().GetSeconds();
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
					GetNodeIpv4Address(thisNode)<<"received a data packet from "
					<<h.GetSource()<<" to "<<h.GetDestination()<<srcAdr<<"to"<<dstAdr);
		double interval = RandomDoubleVauleGenerator(0.0, 0.5);
		Simulator::Schedule(Seconds(interval), &TransmitDataPacket, thisNode,
				srcAdr, dstAdr);
		return 0;
	}
	NS_LOG_INFO(TIME_STAMP_FUC<<
						GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
	return 0;
}


void UpdateInstSpeed(double interval){
		for(int k=0;k<25;k++){
		etx[k].resize(25);
		instSpeed[k].resize(25);
		trans[k].resize(25);
		recv[k].resize(25);
		recvRatio[k].resize(25);
	}
	
		for (NodeContainer::Iterator i = senseNodes.Begin();
			i != senseNodes.End(); i++) {
		Ptr<Node> n = *i;
		// 拿到位置
		// Ptr<ConstantPositionMobilityModel> srcCpmm = n->GetObject<ConstantPositionMobilityModel>();
		Vector sourceLocation = n->GetObject<MobilityModel>()->GetPosition();
		// cout<<"Node"<<n->GetId()<<sourceLocation.x<<endl;
		cout<<"Node"<<n->GetId()<<sourceLocation.x<<endl;
		
			for (NodeContainer::Iterator j = senseNodes.Begin();
				j != senseNodes.End(); j++) {
			Ptr<Node> m = *j;
			// 拿到位置
			Vector senseLocation = m->GetObject<MobilityModel>()->GetPosition();
			double new_distance=GetdistanOf2Nodes(sourceLocation,senseLocation);
		//	trans[m->GetId()][n->GetId()]=trans[m->GetId()][n->GetId()]+1;
			if(new_distance<=250){
			//	recv[m->GetId()][n->GetId()]=recv[m->GetId()][n->GetId()]+1;
				//double recvRatio=recv[m->GetId()][n->GetId()]/trans[m->GetId()][n->GetId()];
				recvRatio[m->GetId()][n->GetId()]=0.05+(1-0.05)*recvRatio[m->GetId()][n->GetId()];
			}
			else{
				recvRatio[m->GetId()][n->GetId()]=(1-0.05)*recvRatio[m->GetId()][n->GetId()];
			}
			Vector old_location1;
			Vector old_location2;
			old_location1.x=old_x[m->GetId()];
			old_location1.y=old_y[m->GetId()];
			old_location2.x=old_x[n->GetId()];
			old_location2.y=old_y[n->GetId()];
			double old_distance=GetdistanOf2Nodes(old_location2,old_location1);
			double speed=(new_distance-old_distance)/interval;
			instSpeed[m->GetId()][n->GetId()]=0.04*speed+(1-0.04)*instSpeed[m->GetId()][n->GetId()];
			etx[m->GetId()][n->GetId()]=(exp(instSpeed[m->GetId()][n->GetId()]*0.2))/(recvRatio[m->GetId()][n->GetId()]*recvRatio[n->GetId()][m->GetId()]);
			//old_x[m->GetId()]=new_x[m->GetId()];
			//old_y[m->GetId()]=new_y[m->GetId()];
		    //new_x[m->GetId()]=senseLocation.x;
			//new_y[m->GetId()]=senseLocation.y;
			//cout<<"Node("<<m->GetId()<<n->GetId()<<")   Node的x："<<senseLocation.x<<"Node的y："<<senseLocation.y<<"instSpeed:"<<instSpeed[m->GetId()][n->GetId()]<<"new"<<new_distance<<"old"<<old_distance<<"r"<<recvRatio[m->GetId()][n->GetId()]<<"etx"<<etx[m->GetId()][n->GetId()]<<endl;
			} 
			old_x[n->GetId()]=new_x[n->GetId()];
			old_y[n->GetId()]=new_y[n->GetId()];
			new_x[n->GetId()]=sourceLocation.x;
			new_y[n->GetId()]=sourceLocation.y;


	} 
}


/*
 * 用于senseNode收到SPT packet之后处理的内联函数
 */
static inline int32_t ProcessSptPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h) {
	double interval = RandomDoubleVauleGenerator(0.0, 0.5);	//产生随机的发包间隔
	Ipv4Address gateway = h.GetSource();	//gateway Ipv4Address
	stringstream recvData;
	packet->CopyData(&recvData, packet->GetSize());
	Ipv4Address source;
	uint32_t pktJumps = AnalyzeSptPacket(recvData, source);//Get jumps and source address from SPT packet
	NS_LOG_DEBUG(
			TIME_STAMP_FUC<< thisNode->GetId()<<" received a SPT packet, gateway= "<<gateway <<", pktJumps = "
			<<pktJumps<<", source = "<<source);
	Ptr<Node>gate = GetNodePtrFromIpv4Adr(gateway,senseNodes,mobileSinkNode);
	uint32_t jump = pktJumps+1;
	Simulator::Schedule(Seconds(interval), &ContinueSptRouting, thisNode,
			jump, source);
	remains.erase(find(remains.begin(),remains.end(),thisNode->GetId()));
	return 0;
}
/*
 * 收到packet的回调函数
 */
void RecvPacketCallback(Ptr<Socket> socket) {
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
			nType = CheckNodeType(thisNode,senseNodes,mobileSinkNode.Get(0));
			switch (nType) {
			case mobileSink_Type: {	//mobileSinkNode收到packet
				switch (pType) {
					case dataGathering_Type:
						//ProcessDataGatheringPacket(thisNode, packet, h);
						break;
					case data_Type:
						totalBytesGathered += pktSize;
						break;
					case neighbor_Type:
						{
							// cout<<"插入邻居表,源节点为------->"<<GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,mobileSinkNode)->GetId()<<"目的节点为------>"<<thisNode->GetId()<<endl;
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
							//uint32_t id1=sourceNode->GetId();
							//Ptr<Node>msNode = mobileSinkNode.Get(0);
							//uint32_t id2=msNode->GetId();
							//double weight = etx[id1][id2];
							double weight=GetdistanOf2Nodes(location2,location1);
							graph.addNewEdgeToList(GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,mobileSinkNode)->GetId(),weight,mobileSinkNode.Get(0)->GetId());
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
						/*EXOR更新邻居表方案*/
						uint32_t curId = thisNode->GetId();
						//发送广播包的节点
						Ipv4Address src = h.GetSource();
						Ptr<Node>node = GetNodePtrFromIpv4Adr(src,senseNodes,mobileSinkNode);
						Ptr<Node>msNode = mobileSinkNode.Get(0);
						uint32_t id = node->GetId();
						// cout<<"当前发送广播包的节点ip为："<<src<<"当前发送广播包的节点id为："<<id<<"接收到广播包的id为："<<curId<<endl;
						Vector location1;
						Vector location2;
						if (isStatic){
							location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
							location2 = node->GetObject<MobilityModel>()->GetPosition();
						}else{
							location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
							location2 = node->GetObject<MobilityModel>()->GetPosition();
						}
						//Ptr<Node>msNode = mobileSinkNode.Get(0);
						//uint32_t id2 = msNode->GetId();
						//double weight1 = etx[id][id2];
						//double weight2 = etx[curId][id2];						
						Vector location = msNode->GetObject<MobilityModel>()->GetPosition();
						double distance1 = GetdistanOf2Nodes(location1,location);//接收广播包到ms的距离
						double distance2 = GetdistanOf2Nodes(location2,location);
						if(distance1<distance2){// && (distance2-distance1>60)
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
	LogComponentEnable("pOLSRScript", LOG_LEVEL_DEBUG);
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
	// senseNodes.Create(nNodes);
	// mobileSinkNode.Create(1);


	senseNodes.Create(nNodes);
	std::string traceFile;
	if(moveSpeed.length()>0){
		traceFile = "scratch/"+mobilityModel+"/test"+moveSpeed+".ns_movements";
	}else{
		traceFile = "scratch/"+mobilityModel+"/speed20.ns_movements";
	}
	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install ()
	
	;
	mobileSinkNode.Create(1);
	NS_LOG_DEBUG("Create nodes done!");
}
/*
 * 创建移动模型并安装到节点
 */
void createMobilityModel(){
	//test grid position allocator
	maxX = 20;
	maxY = 10*(std::ceil(nNodes*1.0/5));
	maxY=30;
	mSinkTraceY = maxY;
	// maxDis = 15;
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
		ObjectFactory pos1;
		pos1.SetTypeId ("ns3::RandomRectanglePositionAllocator");
		// pos1.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=600.0]"));
		// pos1.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=450.0]"));
		pos1.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=600.0]"));
		pos1.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=450.0]"));
		Ptr<PositionAllocator> taPositionAlloc1 = pos1.Create ()->GetObject<PositionAllocator> ();

		mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
										"Speed", StringValue ("ns3::ConstantRandomVariable[Constant=6.0]"),
										"Pause", StringValue ("ns3::ConstantRandomVariable[Constant=20.0]"),
										"PositionAllocator", PointerValue (taPositionAlloc1)
										);
	}
	// mobility.Install(senseNodes);

	mobility.Install(mobileSinkNode);
	mobileSinkNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));
	//接收器安装移动模型
	// Ptr<ListPositionAllocator> lpa = CreateObject<ListPositionAllocator>();
	// lpa->Add(Vector(300, 600, 0));
	// // lpa->Add(Vector(100, 300, 0));
	// // lpa->Add(Vector(150, 450, 0));
	// // lpa->Add(Vector(150, 450, 0));
	// mobility.SetPositionAllocator(lpa);

	// mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	// mobility.Install(mobileSinkNode);

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
	cout<<"==================maxDis"<<maxDis<<endl;
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
		if(isSpeed){
			if(isLifeCycle){
				Panim = new AnimationInterface("/home/wsn/sim_temp/pOLSR/"+mobilityModel+"/SPEED/LIFE/output.xml");
			}else{
				Panim = new AnimationInterface("/home/wsn/sim_temp/pOLSR/"+mobilityModel+"/SPEED/output.xml");
			}
		}else{
			if(isLifeCycle){
				Panim = new AnimationInterface("/home/wsn/sim_temp/pOLSR/"+mobilityModel+"/PACKAGE/LIFE/output.xml");
			}else{
				Panim = new AnimationInterface("/home/wsn/sim_temp/pOLSR/"+mobilityModel+"/PACKAGE/output.xml");
			}
		}
		// Panim = new AnimationInterface("/home/zlb/sim_temp/EXOR/"+mobilityModel+"/output.xml");
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
	// cout<<"共发送的数据包："<<send<<endl;
	// cout<<"开始时间:"<<simStartRealTime<<"(ms)"<<endl;
	// cout<<"结束时间:"<<simFinishRealTime<<"(ms)"<<endl;
	// cout<<"发送数据量:"<<totalBytesGenerated<<"(Bytes)"<<endl;
	// cout<<"接收数据量:"<<totalBytesGathered<<"(Bytes)"<<endl;
	// cout<<"系统吞吐量:"<<totalBytesGathered*1000.0/(simFinishRealTime-simStartRealTime)<<"(Bytes/s)"<<endl;
	// cout<<"系统PDR指标:"<<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
	// cout<<"总消耗能量"<<totalConsumed<<"J"<<endl;
	// EnergyFinalRecord(senseNodes, remaingJ);

	std::cout<<"\n\n***** OUTPUT *****\n\n";
	std::stringstream ss;
	if(isLifeCycle){
		if(isSpeed){
			ss << "/home/wsn/sim_temp/pOLSR/"<<mobilityModel<<"/SPEED/LIFE/"<<moveSpeed<<".record";
		}else{
			ss << "/home/wsn/sim_temp/pOLSR/"<<mobilityModel<<"/PACKAGE/"<<"LIFE"<<"/"<<initialJ<<"-"+mobilityModel+".record";
		}
	}else{
		if(isSpeed){
			ss << "/home/wsn/sim_temp/pOLSR/"<<mobilityModel<<"/SPEED/"<<moveSpeed<<"-"+mobilityModel+".record";
		}else{
			ss << "/home/wsn/sim_temp/pOLSR/"<<mobilityModel<<"/PACKAGE/"<<maxPackets<<"-"+mobilityModel+".record";
		}
	}
	
	std::ofstream of(ss.str().c_str());
	simFinishRealTime = clock();
	cout<<"使用的移动模型:"<<mobilityModel<<endl;
	ss << std::endl << "使用的移动模型:" << mobilityModel<< std::endl;	

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
	// cout<<"updateNeighbor"<<endl;
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
}
void updateNeighborTable(){
	graph.updateArray();
}
/*定时任务,定时交换邻居信息,更新邻居表*/
void update(){
	// cout<<"update()"<<endl;
	updateDone = false;
	updateNeighbor();
	// Simulator::Schedule(Seconds(buildNeighborDone), &testPrintNeighbor);
	Simulator::Schedule(Seconds(buildNeighborDone), &setUpdateDoneFlag,true);
	Simulator::Schedule(Seconds(updateInterval), &update);
}

void testPrintNeighbor(){
	cout<<"当前时间:"<<TIME_STAMP_FUC<<endl;
	printNeighbor();
	//Simulator::Schedule(Seconds(updateInterval), &testPrintNeighbor);
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
/*
 * 工程入口
 */
int main(int argc, char* argv[]) {
	//step0:全局变量初始化
	//step 1 :定义main中局部变量
	double simStartTime = 0.0;  //seconds
	//step 2:解析命令行输入参数
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of Nodes", nNodes);
	cmd.AddValue("isSpeed","is Speed", isSpeed);
	cmd.AddValue("moveSpeed","move Speed", moveSpeed);
	cmd.AddValue("mobilityModel","use mobilityModel", mobilityModel);
	cmd.AddValue("isLifeCycle","is test lifeCycle", isLifeCycle);
	cmd.AddValue("totalTime", "Simulation time length", totalTime);
	cmd.AddValue("simStartTime", "Simulation start time", simStartTime);
	cmd.AddValue("maxDis", "max distance to transmit", maxDis);
	cmd.AddValue("maxPackets", "max distance to transmit", maxPackets);
	cmd.AddValue("initialJ","Initial energy of each BaiscEnergySource", initialJ);
	cmd.AddValue("rewrite", "Whether to rewrite the result-sptMain.record",rewrite);
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
	cmd.Parse(argc, argv);
	NS_LOG_DEBUG("Configure done!");
	mSinkTraceY = maxY/2; //the y value of sink path
	// for(int i=0; i<(int)nNodes; i++)
	// 	SendPacketsNum[i] = UnitPacketNum ;
	//放在cmd.Parse()后面才会生效的变量赋值
	simStopTime=totalTime;
	//step 3:创建仿真文件夹
	// createSimFolder();
	//step 4:设置不同文件log级别
	setLog();
	//step 5:设置基础网络
	netSet();
	setSimStartTime();
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

	Ptr<FlowMonitor> flowMonitor;    
	FlowMonitorHelper flowHelper;    
	flowMonitor = flowHelper.InstallAll();
	//build map   节点id和地址的映射关系。
	//FlowMonitor set------                   
	// Ptr<FlowMonitor> flowMonitor;            
	// FlowMonitorHelper flowHelper;            
	// flowMonitor = flowHelper.InstallAll();   
	buildMapIdAdr();
	cout<<"build idAdrMap done!"<<endl;
	initial(); 
	//从路由发现到数据传输 开始
	//Simulator::Schedule(Seconds(1.0), &startRouteFindFrom,mobileSinkNode.Get(0));
	// Simulator::Schedule(Seconds(5.0),&printNeighbor);
	// Simulator::Schedule(Seconds(5.7),&updateNeighborTable);
	// Simulator::Schedule(Seconds(6.0),&buildRoutes);
	// Simulator::Schedule(Seconds(6.5),&printPaths);
	//Simulator::Schedule(Seconds(5.0),&setSimStartTime);
	// Simulator::Schedule(Seconds(5.0),&DataToSink);
	//从路由发现到数据传输 结束
	//测试周期性邻居交换信息
	Simulator::Schedule(Seconds(0.1),&update);   
	Simulator::Schedule(Seconds(buildNeighborDone),&DataToSink);
	//Simulator::Schedule(Seconds(6.0),&testPrintNeighbor); 
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





