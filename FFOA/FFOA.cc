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

#include <unistd.h>
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
#include <cmath>
#include "global.h"
#include "energy.h"
#include "final_record.h"
#include "GraphList.h"
using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE("GROUPFFOAcript");

//函数声明
void createRoute();
Vector getMiddle(Vector one,Vector two);
void TransmitDataToRelay(uint32_t robotId,uint32_t id,Ipv4Address sourceAdr,Ipv4Address sinkAdr);
void TransmitDataPacket(Ptr<Node> localNode, Ipv4Address sourceAdr,Ipv4Address gatewayAdr) ;
void startRouteFindFrom(Ptr<Node> n);
double GetdistanOf2Nodes(Vector one,Vector two);
void buildMapIdAdr();
void ContinueSptRouting(Ptr<Node> n, uint32_t jumps, Ipv4Address sinkAdr);
uint32_t AnalyzeSptPacket(stringstream &ss, Ipv4Address &source);
void initial();
void updateNeighbor();
void startRoute(Ptr<Node>thisNode,Ipv4Address srcAdr);
// void startRouteRelay(Ptr<Node>thisNode,Ipv4Address srcAdr);
void printNeighbor();
void printPaths();
void findRoutes(int id);
void buildRoutes();
void update();
void setUpdateDoneFlag(bool flag);
void testPrintNeighbor();
uint32_t findMinDistanceRelay(Ptr<Node> local);
double GetdistanFromRelay(Ptr<Node> srcN,uint32_t num);
//void testIP();
void setRobotPosition(uint32_t id,Vector newPosition);
void changeColor(uint32_t id,bool flag);
void setRobotState(uint32_t id, bool flag);

bool isRelay = true;
clock_t lifeTime;
string mobilityModel="";
bool isLifeCycle;
bool isEntity;
double sendInterval = 0.8;

map<int,vector<vector<int>>>routePaths;
map<uint32_t,uint32_t>nodeRobotMap;
Time netDelay;
// 可调参数
double robotSpeed = 80;
double dataInterval = 8.0; //发包间隔
double maxDis =225.0 ;      //最大通信距,要考虑区域面积和部署相关
double buildNeighborDone = 2.5;//新建立邻居表结束时间

double updateInterval = 8.0;//每隔多久更新一次邻居表
uint32_t nNodes = 20;
int node_num = (int)nNodes+1;
uint32_t nSinkNodes = 5;
// double neighborFindDone = 2.0;//当前节点建立邻居表的时间
double weakless = 0;

uint16_t relaySend = 0;
int maxPackets = 1000;
int send = 0;
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
uint32_t pktSize = 1500;
double totalBytesGenerated;
double initialJ = 150.0;
double totalConsumed=0;
uint32_t energyTraceIndex=0;//The global id of node to trace energy, default 0 means no trace
double TxPara=0.000006;//0.000006J/Bytes for Tx
double RxPara=0.000006;//0.000006J/Bytes for Rx
uint32_t nDrain=0;
double Tgather=0.65;

//Algorithm
string algType;
//Others
bool Banim=true;
bool Brotate=true;
bool Bdrain=true;
bool isStatic = true;
AnimationInterface *Panim = 0;
uint32_t firstDrainedNodeId=0;
bool firstDrainFlag=true;
uint32_t firstDrainedSinkId=0;
double firstDrainedNodeTime=0;
string thisSimPath;//simulation path
string exeName="FFOA";
string simFolderName="/home/wsn/sim_temp/FFOA/";
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
static NodeContainer sinkNodes;


//Net device container
NetDeviceContainer senseDevices;
NetDeviceContainer mobileSinkDevice;
NetDeviceContainer sinkDevices;

//Ipv4 related container
Ipv4InterfaceContainer senseIfs;
Ipv4InterfaceContainer mobileSinkIf;
Ipv4InterfaceContainer sinkIfs;
//Energy source pointer
double remaingJ[MAX_NUM];
set<uint32_t>remains;
// vector<uint32_t>totalNodes; 
GraphList graph;

// TestPoint Switch
bool isTest=false;

// TestPoint function
void testEnergy();
void TestNodes();
void testIP();
void testNeighbor();
void testSink();

static bool updateDone = false;
string speed="";


map<uint32_t,Ipv4Address>mapIdAdr;

map<uint32_t,bool>robotState;//false

void getDistrib();
double getMeanEnergy(nodeType nType);
bool isSatisfied(Ptr<Node>thisNode,Ptr<Node>anotherNode);
void setAttract();
uint32_t getConnect(Ptr<Node>thisNode,nodeType nType_1,nodeType nType_2);


double p=4;  
double r=10;   //感知节点平均射程
double e=1.7;   //
uint32_t segementNum=5;
//double senseDistance=15.0;
//Vector location2;
//Vector location1;    
//double distance1;




static NodeContainer netConnHelper;
int connSum=0;
int	connTimes=0;

/*
 * 计算网络连通度函数
 */
void getNetConn(){
	// 清除netConn
	int maxNodes=nNodes+nSinkNodes;
	int netConn[maxNodes];
	for(int m = 0; m < maxNodes; m++)
		netConn[m] = 0;
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
				if(preNode->GetId() == backNode->GetId())
					continue;
				if(netConn[backNode->GetId()] == 1)
					continue;

				Vector pre_location;
				Vector back_location;
				pre_location = preNode->GetObject<MobilityModel>()->GetPosition();
				back_location =  backNode->GetObject<MobilityModel>()->GetPosition();
				double distance_pb = GetdistanOf2Nodes(pre_location, back_location);
				
				if( distance_pb < maxDis){
					cout<<"preNode "<<preNode->GetId()<<"   backNode:  "<<backNode->GetId()<<endl;
					cout<<"距离为： "<<distance_pb <<endl;
					netConn[backNode->GetId()] = 1;
					break;
				}
					
			}
		}
	}
	int conn = 0;
	for (int index = 0; index < (int)nNodes; index++)
		conn += netConn[index];
	connSum += conn;
	connTimes += 1;	
	Simulator::Schedule(Seconds(10.0),&getNetConn);
}

//使sink节点在正方形上移动
void sinkMove(){
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		uint32_t id = thisNode->GetId();
		uint32_t sinkId=id-nNodes;    //sink节点在sinkNodes中的序号
		Vector location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
		Vector newLocation=location1;
		int moveInterval=(sinkId+1)*10;
		int bound1=50*(sinkId+1);
		if(location1.x==(500+bound1)||location1.x==(500-bound1)){
			if(location1.x==(500+bound1)){
				if(location1.y==(500-bound1))
					newLocation.x=location1.x-moveInterval;			
                else
					newLocation.y=location1.y-moveInterval;
			}
			else{
				if(location1.y==(500+bound1))
					newLocation.x=location1.x+moveInterval;
                else
					newLocation.y=location1.y+moveInterval;
			}
		}
		else{
			if(location1.y==(500+bound1))
				newLocation.x=location1.x+moveInterval;
			else
				newLocation.x=location1.x-moveInterval;
		}
		thisNode->GetObject<MobilityModel>()->SetPosition(newLocation);
	}
}


//节点id和地址映射
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
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		uint32_t id = thisNode->GetId();
		Ipv4Address sourceAdr = GetNodeIpv4Address(thisNode);
		pairNode1.first = id;
		pairNode1.second = sourceAdr;
		mapIdAdr.insert(pairNode1);
	}

	Ptr<Node>msNode = mobileSinkNode.Get(0);
	mapIdAdr.insert(pair<uint32_t,Ipv4Address>(msNode->GetId(),GetNodeIpv4Address(msNode)));
	//ceshi
	map <uint32_t, Ipv4Address>::iterator itr;
	cout<<"节点id和节点�?�应的ip地址如下:"<<endl;
	for ( itr = mapIdAdr.begin( ); itr != mapIdAdr.end( ); itr++ ){
		cout<<itr->first<<":"<<itr->second<<endl;
	}
}

void initial(){
	for (NodeContainer::Iterator i = senseNodes.Begin();
			i != senseNodes.End(); i++) {
		Ptr<Node>node = *i;
		// totalNodes.push_back(node->GetId());
		remains.insert(node->GetId());
	}
	//TODO
	//getDistrib();
	graph.nCount = nNodes;
	graph.nSinkCount=nSinkNodes;
	graph.makeNodeArray();
	cout<<"neighbor struct initial done!!"<<endl;
	
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
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();
			i++) {
				Ptr<Node> thisNode = *i;
				Vector location = thisNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
				cout<<"中继ID： "<<thisNode->GetId()<<",   x:  "<<location.x<<",   y:  "<<location.y<<",   z:  "<<location.z<<endl;
			}
	cout<<"mobliesink节点"<<endl;
	Ptr<Node> thisNode = mobileSinkNode.Get(0);
	Vector location;
	location = thisNode->GetObject<MobilityModel>()->GetPosition();
	cout<<"mobilesink ID： "<<thisNode->GetId()<<",   x:  "<<location.x<<",   y:  "<<location.y<<",   z:  "<<location.z<<endl;
}

// 测试能量
void testEnergy(){
	cout<<"能量测试"<<endl;
	for(uint32_t i = 0; i < nNodes; i++)
		cout<<"节点"<<i<<"的剩余能量为： "<<remaingJ[i]<<endl;

}

void testIP(){
	cout<<"senseNodes 的 ip地址如下"<<endl;
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
		i++) {
			Ptr<Node> thisNode = *i;
			Ipv4Address iv = GetNodeIpv4Address(thisNode);
			cout<<thisNode->GetId()<<"---"<<iv<<endl;
	}
	cout<<"sinkNodes 的 ip地址如下"<<endl;
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();
		i++) {
			Ptr<Node> thisNode = *i;
			Ipv4Address iv = GetNodeIpv4Address(thisNode);
			cout<<thisNode->GetId()<<"---"<<iv<<endl;
	}
	cout<<"接收的ip地址如下"<<endl;
	Ipv4Address iv1 = GetNodeIpv4Address(mobileSinkNode.Get(0));
	cout<<mobileSinkNode.Get(0)->GetId()<<"---"<<iv1<<endl;
	// Simulator::Schedule(Seconds(30.0),&testIP);  

}

void testNeighbor(){
	TestNodes();
	testEnergy();
	cout<<"链式表  ： 展示形式为  当前节点id->邻居节点id(weight)" <<endl;
	graph.print();
}

void testSink(){
	// uint32_t n = nNodes + 1;
	cout<<"移动中继"<<endl;
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();	i++) {
				Ptr<Node> thisNode = *i;
				Vector location = thisNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
				cout<<"中继ID： "<<thisNode->GetId()<<"中继位置 x:  "<<location.x<<",   y:  "<<location.y<<endl;
		}

}

void update(){
 	sinkMove();
    graph.clear();  //清除邻居表
	updateDone = false;
	updateNeighbor();
	Simulator::Schedule(Seconds(buildNeighborDone), &setUpdateDoneFlag,true);
	if(isTest == true)
		Simulator::Schedule(Seconds(buildNeighborDone + 0.1), &testNeighbor);
	Simulator::Schedule(Seconds(updateInterval), &update);
}


//sink节点和sense节点开始广播hello消息，来更新邻居表
void updateNeighbor(){
	double interval = 0;	//产生随机的发包间间隔
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
		Simulator::Schedule(Seconds(interval), &startRoute,thisNode,srcAdr);
		interval += 0.1;
	}
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
		Simulator::Schedule(Seconds(interval), &startRoute,thisNode,srcAdr);
		interval += 0.1;
	}
}

/*开始发送广播*/
void startRoute(Ptr<Node>thisNode,Ipv4Address srcAdr){
	Ptr<Packet> pkt = Create<Packet>(pktSize);
	Ipv4Header ipv4Header;
	pktType pktType=neighbor_Type;
	ipv4Header.SetSource(srcAdr);
	ipv4Header.SetIdentification(pktType);
	pkt->AddHeader(ipv4Header);
	Ptr<Socket> source = Socket::CreateSocket(thisNode, tid);
	source->Connect(broadcastAdr);
	source->SetAllowBroadcast(true);	//socket发送广播必须有这么一种设置
	source->Send(pkt);
}

/*
 * 全体senseNodes将按一定的数据生成率发送感知到的数据
 */
void DataToSink() {
	setAttract();    //设置各个节点之间的吸引度
	
	double interval = 0;	
	uint32_t gatewayId;
	Ipv4Address gatewayAdr;
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
			i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
		uint32_t localId =  thisNode->GetId();
			if (!CheckRemainingJ(thisNode)) {
				continue;
			} else {
				Ptr<Node>msNode = mobileSinkNode.Get(0);
		 		Vector location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
				Vector location2 = msNode->GetObject<MobilityModel>()->GetPosition();
				double distance1=GetdistanOf2Nodes(location2,location1);
				if(distance1<maxDis){   //在基站范围内，直接发送给基站
					gatewayAdr = GetNodeIpv4Address(msNode);
//					cout<<"send data from sense "<<localId<<" to ms "<<endl;
					if(isTest == true)
					{
						cout<<"下一跳节点为基站" <<endl;
					}
				}   
				else{
					if(graph.sense2Sense[localId].next){                        
                    gatewayId=graph.findBestAttract(localId,sense_Type,sense_Type);//返回吸引度最大sense节点的id值
					gatewayAdr = mapIdAdr[gatewayId]; 
					if(isTest == true)
					{
						cout<<"下一跳节点选择:" <<gatewayId<<endl;
					}					
					}
					else{
//						cout<<"send data from sense "<<localId<<" to sense failed"<<endl;
						continue;	//没有可发送sense节点	
					}
				}					
			}
				Simulator::Schedule(Seconds(interval), &TransmitDataPacket,
						thisNode, srcAdr, gatewayAdr);       //开始传输数据
				send++;
				interval += sendInterval;
				totalBytesGenerated += pktSize;
				if(send == maxPackets){
					lifeTime = Simulator::Now().GetSeconds();
					Simulator::Stop(Seconds(10.0));
		}
		
	}

	Simulator::Schedule(Seconds(dataInterval), &DataToSink);
}





void TransmitDataPacket(Ptr<Node> localNode, Ipv4Address sourceAdr,Ipv4Address gatewayAdr) {
	NS_LOG_LOGIC(
			std::endl<<TIME_STAMP_FUC <<sourceAdr <<" to "<<gatewayAdr<<" by "
			<<GetNodeIpv4Address(localNode));
	    if(isTest == true)
		{
			cout<<"TransmitDataPacket "<<sourceAdr <<" to "<<gatewayAdr<<" by "
			<<GetNodeIpv4Address(localNode)<<endl;
		}
	// NS_LOG_LOGIC(TIME_STAMP_FUC<<"CheckRemainingJ for "<<localAdr<<" passed!");
	if (CheckRemainingJ(localNode)){
		Ipv4Address localAdr = GetNodeIpv4Address(localNode);
		pktType pktType = data_Type;
		NS_LOG_LOGIC(
				TIME_STAMP_FUC<<"localAdr = "<<localAdr<<", gatewayAdr = "<<gatewayAdr);
		Ptr<Packet> dataPkt = Create<Packet>(pktSize);
		Ipv4Header h;
		Ipv4Address dstAdr = GetNodeIpv4Address(mobileSinkNode.Get(0));
		h.SetDestination(dstAdr);
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




double getMeanEnergy(nodeType nType){
	double totalEnergy;
	if(nType==sense_Type){
		for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
			i++) {
		Ptr<Node> thisNode = *i;
		totalEnergy+=remaingJ[thisNode->GetId()];
			}	
			return (totalEnergy/nNodes);
	}
	else{
		for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();
			i++) {
		Ptr<Node> thisNode = *i;
		totalEnergy+=remaingJ[thisNode->GetId()];
			}	
			return (totalEnergy/nSinkNodes);
	}	
}

double GetdistanOf2Nodes(Vector one,Vector two) {
	return std::sqrt((one.x - two.x)*(one.x - two.x) + (one.y - two.y)*(one.y - two.y));
}

//获取节点的连通度
uint32_t getConnect(uint32_t id,nodeType nType){
	uint32_t connectNum=0;
	if(nType==sense_Type){
		if (graph.sense2Sense[id].next){
			Node1* tmp =graph.sense2Sense[id].next;
			connectNum++;
			while(tmp->next){
				tmp = tmp->next;
				connectNum++;
			}
		}
		if (graph.sense2Sink[id].next){
			Node1* tmp=graph.sense2Sink[id].next;
			connectNum++;
			while(tmp->next){
				tmp = tmp->next;
				connectNum++;
			}
		}
	}
	else{
		if (graph.sink2Sink[id].next){
			Node1* tmp = graph.sink2Sink[id].next;
			connectNum++;
			while(tmp->next){
				tmp = tmp->next;
				connectNum++;
			}
		}
	}			
	return connectNum;
}

//根据连通度计算两个邻居节点之间的吸引度，并加入到邻居表中
void setAttract(){
	double Dr;
	uint32_t connectivity;
	Node1* tmp;
	for(uint32_t i = 0; i < nNodes; ++i) {
		 tmp = graph.sense2Sense[i].next;
		while(tmp){
			connectivity=getConnect(tmp->id,sense_Type);
			Dr=pow(e,(-nNodes*connectivity));
			tmp->Attract=initialJ*(pow(e,(-Dr*r*r)));
			tmp = tmp->next;
		}
	}
	for(uint32_t i = 0; i < nNodes; ++i) {
		tmp = graph.sense2Sink[i].next;
		while(tmp){
			connectivity=getConnect((tmp->id)-nNodes,sink_Type);
			Dr=pow(e,(-nNodes*connectivity));
			tmp->Attract=initialJ*(pow(e,(-Dr*p*p)));
			tmp = tmp->next;
		}
	}
	for(uint32_t i = 0; i < nSinkNodes; ++i){
		Node1* tmp = graph.sink2Sink[i].next;
		while(tmp){
			connectivity=getConnect((tmp->id)-nNodes,sink_Type);
			Dr=pow(e,(-nNodes*connectivity));
			tmp->Attract=initialJ*(pow(e,(-Dr*p*p)));
			tmp = tmp->next;
		}
	}
}

//sink节点接收到数据，传输到基站，或中继到sink节点
static inline int32_t ProcessSinkDataPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h){
	//分析包的source和destination
	Ipv4Address srcAdr = h.GetSource();
	Ipv4Address dstAdr =h.GetDestination();
	Ptr<Node>sourceNode = GetNodePtrFromIpv4Adr(srcAdr,senseNodes,sinkNodes,mobileSinkNode);
	uint32_t gatewayId;
	Ipv4Address gatewayAdr;
	uint32_t localId =  thisNode->GetId();
	//产生随机的发包间
	NS_LOG_LOGIC(TIME_STAMP_FUC<<
				GetNodeIpv4Address(thisNode)<<" received a data packet from "
				<<h.GetSource()<<" to "<<h.GetDestination()<<srcAdr<<"to"<<dstAdr);
	double interval = RandomDoubleVauleGenerator(0.0, 0.5);
	Ptr<Node>msNode = mobileSinkNode.Get(0);
	Vector location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
	Vector location2 = msNode->GetObject<MobilityModel>()->GetPosition();
	double distance1=GetdistanOf2Nodes(location2,location1);
	if(distance1<maxDis){   //在基站范围内，直接发送到基站
	gatewayAdr = GetNodeIpv4Address(msNode);
	if(isTest == true)
	{
		cout<<"下一跳节点为基站" <<endl;
	}
//	cout<<"send data from sink "<<localId<<" to ms "<<endl;
	}   
	else{
		if(graph.sink2Sink[localId].next){
			gatewayId=graph.findBestAttract(localId,sink_Type,sink_Type);  //返回吸引度最大sink节点的id
			if(gatewayId==100){
//			cout<<"send data from sink "<<localId<<" to sink failed"<<endl;
				return 0;
			}
			else{
				gatewayAdr = mapIdAdr[gatewayId];
				if(isTest == true)
				{
					cout<<"下一跳节点选择:" <<gatewayId<<endl;
				}
//				cout<<"send data from sink "<<localId<<" to sink "<<gatewayId<<endl;
			}
		}
		else{
			return 0;
		}
		
	}
    Simulator::Schedule(Seconds(interval), &TransmitDataPacket, thisNode,srcAdr, gatewayAdr);  //传输数据
	NS_LOG_INFO(TIME_STAMP_FUC<<
						GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
	return 0;
}

//sense节点接收到数据，中继到基站或sink节点
static inline int32_t ProcessDataPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h){
	//分析包的source和destination
	Ipv4Address srcAdr = h.GetSource();
	Ipv4Address dstAdr =h.GetDestination();
	Ipv4Address gatewayAdr;
	uint32_t localId=thisNode->GetId();
		//产生随机的发包间隔
		NS_LOG_LOGIC(TIME_STAMP_FUC<<
					GetNodeIpv4Address(thisNode)<<" received a data packet from "
					<<h.GetSource()<<" to "<<h.GetDestination()<<srcAdr<<"to"<<dstAdr);
		double interval = RandomDoubleVauleGenerator(0.0, 0.5);
		Ptr<Node>msNode = mobileSinkNode.Get(0);
		Vector location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
		Vector location2 = msNode->GetObject<MobilityModel>()->GetPosition();
		double distance1=GetdistanOf2Nodes(location2,location1);
		if(distance1<maxDis){   //在基站范围内，直接发送到基站
		gatewayAdr = GetNodeIpv4Address(msNode);
		if(isTest == true)
		{
			cout<<"下一跳节点为基站" <<endl;
		}
//		cout<<"send data from sink "<<localId<<" to ms "<<endl;
		}    
		else{
			if(graph.sense2Sink[localId].next){
				uint32_t gatewayId=graph.findBestAttract(localId,sense_Type,sink_Type);  //返回吸引度最大的sink节点id
				if(gatewayId==100){
//					cout<<"send data from sink "<<localId<<" to sink failed"<<endl;
					return 0;
				}
				else{
					gatewayAdr = mapIdAdr[gatewayId];
					if(isTest == true)
					{
						cout<<"下一跳节点选择:" <<gatewayId<<endl;
					}
				}
			}
			else{
				if(graph.sense2Sense[localId].next){                        
                    uint32_t gatewayId=graph.findBestAttract(localId,sense_Type,sense_Type);//返回吸引度最大sense节点的id值
					gatewayAdr = mapIdAdr[gatewayId]; 
					if(isTest == true){
						cout<<"下一跳节点选择:" <<gatewayId<<endl;
					}					
				}
				else
					return 0;
			}
			
		}
		Simulator::Schedule(Seconds(interval), &TransmitDataPacket, thisNode,srcAdr, gatewayAdr); //中继传输
	NS_LOG_INFO(TIME_STAMP_FUC<<
						GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
	return 0;
}




void RecvPacketCallback(Ptr<Socket> socket) {
	Ptr<Packet> pkt;
	Address from;
	while ((pkt = socket->RecvFrom(from))) {
		Ptr<Packet> packet = pkt->Copy();	
		nodeType nType;
		pktType pType;
		if (packet->GetSize() > 0) {
			Ptr<Node> thisNode = socket->GetNode();	
			Ipv4Header h;
			packet->PeekHeader(h);
			packet->RemoveHeader(h);
			pType = pktType(h.GetIdentification());
			nType = CheckNodeType(thisNode,senseNodes,sinkNodes,mobileSinkNode.Get(0));
			Ptr<Node>sourceNode = GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,sinkNodes,mobileSinkNode);
	
			switch (nType) {
			case mobileSink_Type: {	//mobileSinkNode收到packet
				switch (pType) {				
					case data_Type:
						cout<<"ms收到数据包，来自节点id:"<<sourceNode->GetId()<<endl;
						totalBytesGathered += pktSize;
						break;
				case neighbor_Type:		
					{
					
					}														
						break;						
					default:
						break;
					}
				break;
			}
			case sense_Type: {	//senseNode收到packe
				switch (pType) {
					case data_Type: {///收到data_Type的packet，将执行中继
						if (CheckRemainingJ(thisNode, packet)) {
							UpdateEnergySources(thisNode, packet, 1,mobileSinkNode);
													ProcessDataPacket(thisNode, packet, h);
						}
					}
						break;
					case neighbor_Type: {///sense节点收到neighbor_Type的packet，将更新邻居表
						nodeType sType = CheckNodeType(sourceNode,senseNodes,sinkNodes,mobileSinkNode.Get(0));
						if(sType==sense_Type){
						uint32_t curId = thisNode->GetId();
						//发送广播包的节点
						uint32_t id = sourceNode->GetId();			
						Vector location1;
						Vector location2;
						location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
						location2 = sourceNode->GetObject<MobilityModel>()->GetPosition();
						Ptr<Node>msNode = mobileSinkNode.Get(0);
						Vector location;							
						location = msNode->GetObject<MobilityModel>()->GetPosition();			 
						double distance1 = GetdistanOf2Nodes(location1,location);//接收广播包到ms的距离
						double distance2 = GetdistanOf2Nodes(location2,location);	
						if(distance1 < distance2)//如果该sense节点比发送广播的sense节点离基站更近，则加入邻居表
						graph.addSenseNodeToList(id,0,curId);
						}						                  						
					}
						break;
					default: 
						break;		
				}
				break;
			}
			case sink_Type: {	//sinkNode收到packet
				switch (pType) {
					//缺少数据包包逻辑
				case data_Type: {///收到data_Type的packet
					// relaySend += packet->GetSize();
					if (CheckRemainingJ(thisNode, packet)) {
					UpdateEnergySources(thisNode, packet, 1,mobileSinkNode);
					ProcessSinkDataPacket(thisNode, packet, h);
					}
				}
					break;
				case neighbor_Type:{//sink节点收到广播更新邻居表
						nodeType sType = CheckNodeType(sourceNode,senseNodes,sinkNodes,mobileSinkNode.Get(0));
						uint32_t curId = thisNode->GetId();						
						uint32_t id = sourceNode->GetId();							
						if(sType==sense_Type){
						graph.addSinkNodeToList(id,0,curId);
						}
						else{	
						Vector location1;
						Vector location2;
						location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
						location2 = sourceNode->GetObject<MobilityModel>()->GetPosition();		
						Ptr<Node>msNode = mobileSinkNode.Get(0);
						Vector location;							
						location = msNode->GetObject<MobilityModel>()->GetPosition();			 
						double distance1 = GetdistanOf2Nodes(location1,location);//接收广播包到ms的距离
						double distance2 = GetdistanOf2Nodes(location2,location);	
						if(distance1 < distance2)//如果该sink节点比发送广播的sink节点离基站更近，则加入邻居表									
						graph.addSinkSinkToList(id,0,curId);
						}
				}
					break;
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
 * 创建系统仿真文件
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
	//创建sim结果子文件夹，时间为�???录名
	ssi << "/";
	ssi << setw(2) << setfill('0') << right << timeInfo->tm_hour;
	ssi << setw(2) << setfill('0') << right << timeInfo->tm_min;
	ssi << setw(2) << setfill('0') << right << timeInfo->tm_sec;
	mkdir(ssi.str().c_str(), S_IRWXU);
	ssi<<"/";
	thisSimPath=ssi.str();//文件夹路径，下层�???具体文件
	ssi.str("");
	ssi.clear();
}
/*
 * 不同文件log级别设置并使能log
 */
void setLog(){
	LogComponentEnable("GROUPFFOAcript", LOG_LEVEL_DEBUG);
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
 * �???否生成XML动画文件
 */
void createXml(){
	/**Set anim**/
	if (Banim) {
		if(isEntity){
			Panim = new AnimationInterface("/home/wsn/sim_temp/FFOA/output.xml");
		}else{
			if(isRelay){
				Panim = new AnimationInterface("/home/wsn/sim_temp/FFOA/output.xml");
			}else{
				Panim = new AnimationInterface("/home/wsn/sim_temp/FFOA/"+mobilityModel+"/output.xml");
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

		//机器人�?�色
		uint32_t cnt = 0;
		for (NodeContainer::Iterator i = sinkNodes.Begin();i != sinkNodes.End(); i++) {
			Ptr<Node> n = *i;
			Panim->UpdateNodeSize(n->GetId(), 3.0, 3.0);
			Panim->UpdateNodeColor(sinkNodes.Get(cnt++), 0, 200, 255);
		}
	}
	NS_LOG_DEBUG("Set anmi done!");
}


void finalRecord(){
	std::cout<<"\n\n***** OUTPUT *****\n\n";
	std::stringstream ss;
	ss<<"/home/wsn/sim_temp/FFOA/"<<mobilityModel<<"/"<<speed<<"-"<<maxDis<<"-"<<maxPackets<<".record";
	std::ofstream of(ss.str().c_str());
	simFinishRealTime = Simulator::Now().GetSeconds();
	cout<<"使用的移动模型:"<<mobilityModel<<endl;
///////////////////////////////////////////
	std::cout << "网络时延: " << netDelay<< "\n";
	std::cout<<"PDR指标:"<<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
	std::cout<<"网络吞吐量:"<<totalBytesGathered*8.0/((simFinishRealTime-simStartRealTime)*1000.0)<<"Kbps"<<endl;
	std::cout<<"平均网络连通性:"<<connSum*1.0 / (connTimes * (int)nNodes * 1.0)<<endl;
	ss << std::endl << "使用的移动模型:" << mobilityModel<< std::endl;	
	ss << std::endl << "网络时延:" << netDelay<<"(ms)"<< std::endl;
	ss << std::endl << "网络吞吐量: " << totalBytesGathered*8.0/((simFinishRealTime-simStartRealTime)*1000.0)<<"Kbps"<<endl;
	ss << std::endl << "PDR指标:" <<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;	
	ss << std::endl<<"平均网络连通性:"<<connSum*1.0 / (connTimes * (int)nNodes * 1.0)<<endl;
	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();
}







void printNeighbor(){
	graph.print();
}




void testPrintNeighbor(){
	printNeighbor();
	Simulator::Schedule(Seconds(updateInterval), &testPrintNeighbor);
}

void setSimStartTime(){
	simStartRealTime = Simulator::Now().GetSeconds();
}


void setUpdateDoneFlag(bool flag){
	updateDone = flag;
}









void changeColor(uint32_t id,bool flag){
	if(id == 0){
		return;
	}
	if(!flag){
		Panim->UpdateNodeColor(GetNodePtrFromGlobalId(id,sinkNodes,mobileSinkNode), 200, 250, 0);
	}else{
		Panim->UpdateNodeColor(GetNodePtrFromGlobalId(id,sinkNodes,mobileSinkNode), 0, 200, 255);
	}
	
}
/*
	创建节点
*/
void createNode(){
	senseNodes.Create(nNodes);
	sinkNodes.Create(nSinkNodes);
	std::string traceFile = "scratch/"+mobilityModel+"/ffoa_speed"+speed+".ns_movements";
	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install ();	
	mobileSinkNode.Create(1);
	netConnHelper.Add(senseNodes);
	netConnHelper.Add(sinkNodes);
	NS_LOG_DEBUG("Create nodes done!");
}

/*
	安装移动模型
*/
void createMobilityModel(){
	MobilityHelper mobility;
	ObjectFactory pos1;
	pos1.SetTypeId ("ns3::RandomRectanglePositionAllocator");
	pos1.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
	pos1.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
	Ptr<PositionAllocator> taPositionAlloc1 = pos1.Create ()->GetObject<PositionAllocator> ();
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");	
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(mobileSinkNode);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(sinkNodes);
	mobileSinkNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));
	
	vector <Ptr<ConstantPositionMobilityModel> > cpmm(5);
		for (uint32_t i = 0; i < 5; i++)
			cpmm[i] =sinkNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
		cpmm[0]->SetPosition(Vector(550, 550, 0));
		cpmm[1]->SetPosition(Vector(400, 600, 0));
		cpmm[2]->SetPosition(Vector(650, 350, 0));
		cpmm[3]->SetPosition(Vector(300, 300, 0));
		cpmm[4]->SetPosition(Vector(750, 750, 0));
   NS_LOG_DEBUG("Create Mobility done!");
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
	/** wifi channel 同时设置最大通信距离**/
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
	mobileSinkDevice = wifi.Install(wifiPhy, wifiMac, mobileSinkNode);
	senseDevices = wifi.Install(wifiPhy, wifiMac, senseNodes);
	sinkDevices = wifi.Install(wifiPhy, wifiMac, sinkNodes);

	ssi<<thisSimPath<<exeName;
	ssi.str("");
	ssi.clear();
	NS_LOG_DEBUG("Create devices done!");
	NS_LOG_DEBUG("Set pcap and anmi done!");
}

/*
 * 安装路由协议
 */
void installInternetStack(){
	InternetStackHelper stack2;
	stack2.Install (senseNodes);
	stack2.Install (sinkNodes);
   stack2.Install (mobileSinkNode);
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");	
	senseIfs = ipv4.Assign(senseDevices);
	sinkIfs = ipv4.Assign(sinkDevices);
    mobileSinkIf = ipv4.Assign(mobileSinkDevice);
	NS_LOG_DEBUG("Install Internet stack done!");
}
/*
	设置Socket回调
*/
void createSocketCallBack(){
	vector<Ptr<Socket> > recvSocket(nNodes+1+nSinkNodes);
	recvSocket[0] = Socket::CreateSocket(mobileSinkNode.Get(0),tid);
	recvSocket[0]->Bind(InetSocketAddress(mobileSinkIf.GetAddress(0), 80));
	recvSocket[0]->SetRecvCallback(MakeCallback(&RecvPacketCallback));

	for (uint32_t i = 1; i < nNodes+1; i++) {
		recvSocket[i] = Socket::CreateSocket(senseNodes.Get(i-1), tid);
		recvSocket[i]->Bind(InetSocketAddress(senseIfs.GetAddress(i-1), 80));
		recvSocket[i]->SetRecvCallback(MakeCallback(&RecvPacketCallback));
	}
	//UAV节点设置回调
	for (uint32_t i = nNodes+1; i < nNodes+nSinkNodes+1; i++) {
		recvSocket[i] = Socket::CreateSocket(sinkNodes.Get(i-1-nNodes), tid);
		recvSocket[i]->Bind(InetSocketAddress(sinkIfs.GetAddress(i-1-nNodes), 80));
		recvSocket[i]->SetRecvCallback(MakeCallback(&RecvPacketCallback));
	}

	NS_LOG_DEBUG("Set recvSocket done!");
}




/*
 * 工程入口
 */
int main(int argc, char* argv[]) {
	//step0:全局变量初�?�化
	mSinkTraceY = maxY/2; //the y value of sink path
	//step 1 :定义main�???局部变�???
	double simStartTime = 0.0;  //seconds
	//step 2:解析命令行输入参�???
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of Nodes", nNodes);
	cmd.AddValue("mobilityModel","use mobilityModel", mobilityModel);
	cmd.AddValue("isLifeCycle","is test lifeCycle", isLifeCycle);
	cmd.AddValue("isEntity","is entity", isEntity);
	cmd.AddValue("isRelay","is relay", isRelay);
	cmd.AddValue("isTest", "open Test Point or not", isTest);
	cmd.AddValue("nSinkNodes", "Number of Nodes", nSinkNodes);
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
	cmd.AddValue("speed","Speed of nodes",speed);
	cmd.Parse(argc, argv);
	NS_LOG_DEBUG("Configure done!");
	simStopTime=totalTime;
	createSimFolder();
	setLog();
	netSet();
	createNode();
	createMobilityModel();
	createWifiDevice();
	installInternetStack();
	getNetConn();
	createSocketCallBack();
	createXml();
	InstallEnergy(senseNodes,sinkNodes);
	cout<<"install energy done!"<<endl;
	
	Ptr<FlowMonitor> flowMonitor;    
	FlowMonitorHelper flowHelper;    
	flowMonitor = flowHelper.InstallAll();
    //build map   节点id和地址的映射关系：
	buildMapIdAdr();
	cout<<"build idAdrMap done!"<<endl;
	initial(); 
	if(isTest == true){
		testIP();
		TestNodes();
	}
	// Simulator::Schedule(Seconds(2.0),&startRoute,senseNodes.Get(0),GetNodeIpv4Address(senseNodes.Get(0)));
	//测试周期性邻居交换信息
	Simulator::Schedule(Seconds(0.0),&update);   
	Simulator::Schedule(Seconds(2.0),&setSimStartTime);
	Simulator::Schedule(Seconds(2.5),&DataToSink);


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
	}
	cout<<"网络时延:"<<delay/num<<endl;
	netDelay = delay/num;
    cout<<"connSum:"<<connSum<<"connTimes:"<<connTimes<<"平均连通度"<<connSum/connTimes<<endl;
	finalRecord();
	Simulator::Destroy();

	return 0;
}





