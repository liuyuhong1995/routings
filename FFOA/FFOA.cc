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
//ADD BY ZLB
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
void testPrintNeighbor();//数据传输之前,测试邻居表当前状�???
uint32_t findMinDistanceRelay(Ptr<Node> local);
double GetdistanFromRelay(Ptr<Node> srcN,uint32_t num);
void testIP();
void setRobotPosition(uint32_t id,Vector newPosition);
void changeColor(uint32_t id,bool flag);
void setRobotState(uint32_t id, bool flag);

bool isRelay;
clock_t lifeTime;
string mobilityModel="";
bool isLifeCycle;
bool isEntity;

map<int,vector<vector<int>>>routePaths;
map<uint32_t,uint32_t>nodeRobotMap;
Time netDelay;
/*�???调参�???*/
double robotSpeed = 80;
double dataInterval = 8.0; //发包间隔
double maxDis =225.0 ;      //最大通信距�??,要考虑区域面积和部署相关问�???
double buildNeighborDone = 2.5;//新建立邻居表结束时间

double updateInterval = 8.0;//每隔多久更新一次邻居表
uint32_t nNodes = 20;
int node_num = (int)nNodes+1;
uint32_t nSinkNodes = 5;
// double neighborFindDone = 2.0;//当前节点建立邻居表的时间
double weakless = 0;//小�?�模实时更新,解决长链�???和弱链路的问�???.

uint16_t relaySend = 0;
int maxPackets = 1000;
int send = 0;
/*
 * 定义UdpSocketFactory为socket的类型和socket的广�???地址
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
double totalTime = 999.0;  //max simulation time
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
uint32_t pktSize = 1500;//1000字节  9�???节点 包含6个sense�???3个sink，一共发10�??? 9*10*1000=90000,一�???�???90000.
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
bool Brotate=true;//�???否启动动态网�???
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


static bool updateDone = false;

//存普通节点ip和�?�应的ip地址容器
map<uint32_t,Ipv4Address>mapIdAdr;

map<uint32_t,bool>robotState;//false  �???由�?,�???�???继状�???.

void getDistrib();
double getMeanEnergy(nodeType nType);
bool isSatisfied(Ptr<Node>thisNode,Ptr<Node>anotherNode);
void setAttract();
uint32_t getConnect(Ptr<Node>thisNode,nodeType nType_1,nodeType nType_2);


double p=4;  //平均�?径损�?
double r=10;   //感知节点平均射程
double e=1.7;   //
uint32_t segementNum=5;
//double senseDistance=15.0;
//Vector location2;
//Vector location1;    
//double distance1;


vector<vector<int>> distrib;

void getDistrib(){
	Vector location_1;//msNode
	Vector location_2;//senseNdoe
	double distance_1=0.0; 
	Ptr<Node>msNode = mobileSinkNode.Get(0);
	location_1 = msNode->GetObject<MobilityModel>()->GetPosition();
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		location_2 = thisNode->GetObject<MobilityModel>()->GetPosition(); 
		distance_1 = GetdistanOf2Nodes(location_1,location_2);//sense到ms的距�?
		int quotient=distance_1/(1000/segementNum);
		distrib[quotient].push_back(thisNode->GetId());
	}
}

/*bool doubleEqual(double x,double y){
    if(fabs(x-y)<0.0000001)
    	return true;
	else
	    return false;
}
*/

void sinkMove(){
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		uint32_t id = thisNode->GetId();
		uint32_t sinkId=id-nNodes;    //sink节点在sinkNodes�?的序�?
		Vector location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
		Vector newLocation=location1;
		int moveInterval=(sinkId+1)*20;
		int bound1=100*(sinkId+1);
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
  //    cout<<"sink"<<id<<" location:"<<location1.x<<","<<location1.y<<endl;
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

	//插入�???的地
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

void testIP(){
	cout<<"senseNodes �??? ip地址如下"<<endl;
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
		i++) {
			Ptr<Node> thisNode = *i;
			Ipv4Address iv = GetNodeIpv4Address(thisNode);
			cout<<thisNode->GetId()<<"---"<<iv<<endl;
	}
	cout<<"sinkNodes �??? ip地址如下"<<endl;
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();
		i++) {
			Ptr<Node> thisNode = *i;
			Ipv4Address iv = GetNodeIpv4Address(thisNode);
			cout<<thisNode->GetId()<<"---"<<iv<<endl;
	}
	cout<<"接收�??? �??? ip地址如下"<<endl;
	Ipv4Address iv1 = GetNodeIpv4Address(mobileSinkNode.Get(0));
	cout<<mobileSinkNode.Get(0)->GetId()<<"---"<<iv1<<endl;
	// Simulator::Schedule(Seconds(30.0),&testIP);  

}



void update(){

 	sinkMove();
    graph.clear();
	updateDone = false;
	updateNeighbor();
	Simulator::Schedule(Seconds(buildNeighborDone), &setUpdateDoneFlag,true);
	Simulator::Schedule(Seconds(updateInterval), &update);
}


void updateNeighbor(){
	double interval = 0;	//产生随机的发包间�???
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
		Simulator::Schedule(Seconds(interval), &startRoute,thisNode,srcAdr);
		// cout<<"从一�???节点开始广�???发现�???由："<<thisNode->GetId()<<endl;
		interval += 0.1;
	}
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
		Simulator::Schedule(Seconds(interval), &startRoute,thisNode,srcAdr);
		// cout<<"从一�???节点开始广�???发现�???由："<<thisNode->GetId()<<endl;
		interval += 0.1;
	}
}

/*一�???节点邻居发现的开�???*/
void startRoute(Ptr<Node>thisNode,Ipv4Address srcAdr){
	Ptr<Packet> pkt = Create<Packet>(pktSize);
	Ipv4Header ipv4Header;
	pktType pktType=neighbor_Type;
	ipv4Header.SetSource(srcAdr);
	ipv4Header.SetIdentification(pktType);
	pkt->AddHeader(ipv4Header);
	Ptr<Socket> source = Socket::CreateSocket(thisNode, tid);
	source->Connect(broadcastAdr);
	source->SetAllowBroadcast(true);	//socket发送广�???必须有这么一�???设置
	source->Send(pkt);
	// cout<<"测试一�???节点的广�???�???"<<thisNode->GetId()<<endl;
}

/*
 * 全体senseNodes将按一定的数据生成率向senseNodes发送感知到的数�?
 */
void DataToSink() {
	setAttract();
	//graph.print();
	double interval = 0;	//产生随机的发包间�?
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
			//	cout<<TIME_STAMP_FUC<<"datatosink--数据发出,节点id:"<<thisNode->GetId()<<",num is-->"<<send<<endl;
				if(distance1<maxDis){   //�?以连接直接到ms，是否直接发送？
				gatewayAdr = GetNodeIpv4Address(msNode);
					cout<<"send data from sense "<<localId<<" to ms "<<endl;
				}   
				else
					if(graph.sense2Sense[localId].next){  
			//		getDistrib();
			//		uint32_t segement_1=findSegement(thisNode);                        
                    gatewayId=graph.findBestAttract(localId,sense_Type,sense_Type);
					gatewayAdr = mapIdAdr[gatewayId];
					cout<<"send data from sense "<<localId<<" to sense "<<gatewayId<<endl;
					}
					else{
						cout<<"send data from sense "<<localId<<" to sense failed"<<endl;
						continue;	//没有�?以发送sense	
					}										
			}
				Simulator::Schedule(Seconds(interval), &TransmitDataPacket,
						thisNode, srcAdr, gatewayAdr);
				send++;
				interval += 3;
				totalBytesGenerated += pktSize;
		
	}
	if(send == maxPackets){
		lifeTime = Simulator::Now().GetSeconds();
		Simulator::Stop(Seconds(5.0));
	}
	Simulator::Schedule(Seconds(dataInterval), &DataToSink);
}




/*
 *发送数�???到下一跳网�???
 */
void TransmitDataPacket(Ptr<Node> localNode, Ipv4Address sourceAdr,Ipv4Address gatewayAdr) {
	NS_LOG_LOGIC(
			std::endl<<TIME_STAMP_FUC <<sourceAdr <<" to "<<gatewayAdr<<" by "
			<<GetNodeIpv4Address(localNode));

	// NS_LOG_LOGIC(TIME_STAMP_FUC<<"CheckRemainingJ for "<<localAdr<<" passed!");
	if (CheckRemainingJ(localNode)){
		Ipv4Address localAdr = GetNodeIpv4Address(localNode);
		pktType pktType = data_Type;
		//通过传过来的�???由表获取
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
	//  cout<<"from "<<sourceAdr<<"by"<<GetNodeIpv4Address(localNode)<<" to "<<gatewayAdr<<endl;
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

//sink到sink
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
	if(distance1<maxDis){   //�?以连接直接到ms，是否直接发送？
	gatewayAdr = GetNodeIpv4Address(msNode);
	cout<<"send data from sink "<<localId<<" to ms "<<endl;
	}   
	else{
		if(graph.sink2Sink[localId].next){
			gatewayId=graph.findBestAttract(localId,sink_Type,sink_Type);
			if(gatewayId==100){
			cout<<"send data from sink "<<localId<<" to sink failed"<<endl;
				return 0;
			}
			else{
				gatewayAdr = mapIdAdr[gatewayId];
				cout<<"send data from sink "<<localId<<" to sink "<<gatewayId<<endl;
			}
		}
		else{
			return 0;
		}
		
	}
    Simulator::Schedule(Seconds(interval), &TransmitDataPacket, thisNode,srcAdr, gatewayAdr);
	NS_LOG_INFO(TIME_STAMP_FUC<<
						GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
	return 0;
}


uint32_t findSegement(Ptr<Node>thisNode){
	int thisId=thisNode->GetId();
	uint32_t segement_1=0;
	for(vector<vector<int> >::iterator it1=distrib.begin();it1!=distrib.end(); ++it1){
		for(vector<int>::iterator it2=(*it1).begin(); it2!=(*it1).end(); ++it2){
			if(*it2==thisId){
				break;
			}
		}
		segement_1+=1;
	}
    return segement_1;
}



//bool isSatisfied(Ptr<Node>thisNode,Ptr<Node>anotherNode){   
	//条件9
//	uint32_t segement_1=findSegement(thisNode);//条件15�?
//	uint32_t segement_2=findSegement(anotherNode);
//	return true;
//	for(vector<int>::iterator it1=distrib[segement_1].begin();it1!=distrib[segement_1].end(); ++it1){
			
//		}

//}

//sense�?继到sink
static inline int32_t ProcessDataPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h){
	//分析包的source和destination
	Ipv4Address srcAdr = h.GetSource();;
	Ipv4Address dstAdr =h.GetDestination();
	Ipv4Address gatewayAdr;
	uint32_t localId=thisNode->GetId();
		//产生随机的发包间�???
		NS_LOG_LOGIC(TIME_STAMP_FUC<<
					GetNodeIpv4Address(thisNode)<<" received a data packet from "
					<<h.GetSource()<<" to "<<h.GetDestination()<<srcAdr<<"to"<<dstAdr);
		double interval = RandomDoubleVauleGenerator(0.0, 0.5);
		Ptr<Node>msNode = mobileSinkNode.Get(0);
		Vector location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
		Vector location2 = msNode->GetObject<MobilityModel>()->GetPosition();
		double distance1=GetdistanOf2Nodes(location2,location1);
		if(distance1<maxDis){   //�?以连接直接到ms，是否直接发送？
		gatewayAdr = GetNodeIpv4Address(msNode);
		cout<<"send data from sink "<<localId<<" to ms "<<endl;
		}    
		else{
			if(graph.sense2Sink[localId].next){
				uint32_t gatewayId=graph.findBestAttract(localId,sense_Type,sink_Type);
				if(gatewayId==100){
					cout<<"send data from sink "<<localId<<" to sink failed"<<endl;
					return 0;
				}
				else{
					gatewayAdr = mapIdAdr[gatewayId];
					cout<<"send data from sink "<<localId<<" to sink "<<gatewayId<<endl;
				}
			}
			else{
				return 0;
			}
			
		}
		Simulator::Schedule(Seconds(interval), &TransmitDataPacket, thisNode,srcAdr, gatewayAdr);
	NS_LOG_INFO(TIME_STAMP_FUC<<
						GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
	return 0;
}




void RecvPacketCallback(Ptr<Socket> socket) {
	 //cout<<"进入回调-------------------"<<endl;
	Ptr<Packet> pkt;
	Address from;
	while ((pkt = socket->RecvFrom(from))) {
		Ptr<Packet> packet = pkt->Copy();	
		nodeType nType;
		pktType pType;
		if (packet->GetSize() > 0) {
			Ptr<Node> thisNode = socket->GetNode();	//这里这个socket是received socket，不�???发送的那个
			Ipv4Header h;
			packet->PeekHeader(h);
			packet->RemoveHeader(h);
			pType = pktType(h.GetIdentification());
			nType = CheckNodeType(thisNode,senseNodes,sinkNodes,mobileSinkNode.Get(0));
			Ptr<Node>sourceNode = GetNodePtrFromIpv4Adr(h.GetSource(),senseNodes,sinkNodes,mobileSinkNode);
	//		Vector location2 = sourceNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
	//	   Vector location1 = thisNode->GetObject<ConstantPositionMobilityModel>()->GetPosition();
		//	double distance1=GetdistanOf2Nodes(location2,location1);
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
					case data_Type: {///收到data_Type的packet，将执�?�中�???
						if (CheckRemainingJ(thisNode, packet)) {
							UpdateEnergySources(thisNode, packet, 1,mobileSinkNode);
							// cout<<"进入回调--接收数据包的节点id�???:"<<thisNode->GetId()<<endl;
							ProcessDataPacket(thisNode, packet, h);
						}
					}
						break;
					case neighbor_Type: {///收到neighbor_Type的packet，将更新邻居�?
						nodeType sType = CheckNodeType(sourceNode,senseNodes,sinkNodes,mobileSinkNode.Get(0));
						if(sType==sense_Type){
						uint32_t curId = thisNode->GetId();
						//发送广�???包的节点
						uint32_t id = sourceNode->GetId();			
						Vector location1;
						Vector location2;
						location1 = thisNode->GetObject<MobilityModel>()->GetPosition();
						location2 = sourceNode->GetObject<MobilityModel>()->GetPosition();
						Ptr<Node>msNode = mobileSinkNode.Get(0);
						Vector location;							
						location = msNode->GetObject<MobilityModel>()->GetPosition();			 
						double distance1 = GetdistanOf2Nodes(location1,location);//接收广播包到ms的距�?
						double distance2 = GetdistanOf2Nodes(location2,location);	
						if(distance1 < distance2)
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
				case data_Type: {///收到data_Type的packet，将执�?�中�???
					// cout<<"�?继节�? "<<thisNode->GetId()<<"获取到的数据包数�???--->"<<packet->GetSize()<<endl;
					// relaySend += packet->GetSize();
					UpdateEnergySources(thisNode, packet, 1,mobileSinkNode);
					ProcessSinkDataPacket(thisNode, packet, h);
				}
					break;
				case neighbor_Type:{
						nodeType sType = CheckNodeType(sourceNode,senseNodes,sinkNodes,mobileSinkNode.Get(0));
						uint32_t curId = thisNode->GetId();						
						uint32_t id = sourceNode->GetId();										
						if(sType==sense_Type){
						graph.addSinkNodeToList(id,0,curId);
						}
						else{									
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
 * 创建系统仿真文件�???
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
	// simFinishRealTime = clock();
	// //生命周期的统�???
	// cout<<"初�?�能�??? = "<<initialJ<<endl;
	// cout<<"网络生命周期 = " << (double)simFinishRealTime/1000000.0 << "(s)" << std::endl;

	// cout<<"�???继发送的数据总量�??? = "<<relaySend<<endl;

	// // cout<<"开始时�???:"<<simStartRealTime<<endl;
	// // cout<<"结束时间:"<<simFinishRealTime<<endl;
	// // cout<<"发送数�???�???:"<<totalBytesGenerated<<endl;
	// // cout<<"接收数据�???:"<<totalBytesGathered<<endl;
	// cout<<"PDR:"<<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
	// cout<<"网络吞吐�???:"<<totalBytesGathered*1000.0/(simFinishRealTime-simStartRealTime)<<"(Byte/s)"<<endl;
	// EnergyFinalRecord(senseNodes, remaingJ);



	std::cout<<"\n\n***** OUTPUT *****\n\n";
	std::stringstream ss;
	if(isLifeCycle){
		if(isEntity){
			ss << "/home/wsn/sim_temp/FFOA/"<<initialJ<<"-entity.record";
		}else{
			if(isRelay){
				ss << "/home/wsn/sim_temp/FFOA/"<<initialJ<<"-J-"<<nSinkNodes<<"-LIFE-RELAYs-"+mobilityModel+".record";
			}else{
				ss << "/home/wsn/sim_temp/FFOA/"<<mobilityModel<<"/"<<initialJ<<"-"+mobilityModel+".record";
			}
		}
	}else{
		if(isEntity){
			ss << "/home/wsn/sim_temp/FFOA/"<<maxPackets<<"-entity.record";
		}else{
			if(isRelay){
				ss << "/home/wsn/sim_temp/FFOA/"<<nSinkNodes<<"-SINK-"+mobilityModel+".record";
			}else{
				ss << "/home/wsn/sim_temp/FFOA/"<<mobilityModel<<"/"<<maxPackets<<"-"+mobilityModel+".record";
			}
		}
	}
	
	std::ofstream of(ss.str().c_str());
	simFinishRealTime = clock();
	if(mobilityModel.length()==0){
		cout<<"使用的移动模�???:RWP"<<endl;
		ss << std::endl << "使用的移动模�???:" << "RWP"<< std::endl;
	}else{
		cout<<"使用的移动模�???:"<<mobilityModel<<endl;
		ss << std::endl << "使用的移动模�???:" << mobilityModel<< std::endl;	
	}

	//如果测量网络生存周期,记录如下�???
	if(isLifeCycle){
		cout<<"初�?�能�???:"<<initialJ<<endl;
		cout<<"网络生存�???:"<<lifeTime<<"(s)"<<endl;
		ss << std::endl << "初�?�能�???:" << initialJ<<"(J)"<< std::endl;
		ss << std::endl << "网络生命周期:" << lifeTime<<"(s)"<< std::endl;
	}else{
		std::cout << "网络时延: " << netDelay<< "\n";
		std::cout<<"PDR指标:"<<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
		cout<<totalBytesGathered<<endl;
		cout<<totalBytesGenerated<<endl;
		std::cout<<"网络吞吐�???:"<<totalBytesGathered*1.0/((simFinishRealTime-simStartRealTime)/1000)<<"(Byte/s)"<<endl;
		ss << std::endl << "网络时延:" << netDelay<<"(ms)"<< std::endl;
		ss << std::endl << "网络吞吐�???: " << totalBytesGathered*1.0/((simFinishRealTime-simStartRealTime)/1000)<<"(Byte/s)"<<endl;
		ss << std::endl << "PDR指标:" <<totalBytesGathered/totalBytesGenerated*100<<"%"<<endl;
	}

	// ss << "Total comsumed energy = " << totalConsumed <<std::endl;

	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();
}







void printNeighbor(){
	graph.print();
	// graph.createRoute();
	// graph.updateArray();//更新邻接矩阵
}

//void createRoute(){
//	graph.createRoute();
	// graph.printRoute();
//}


void testPrintNeighbor(){
	printNeighbor();
	Simulator::Schedule(Seconds(updateInterval), &testPrintNeighbor);
}

void setSimStartTime(){
	simStartRealTime = clock();
}


void setUpdateDoneFlag(bool flag){
	updateDone = flag;
}









void changeColor(uint32_t id,bool flag){
	if(id == 0){
		return;
	}
	if(!flag){//false的时�???,更新为繁忙状�???
		Panim->UpdateNodeColor(GetNodePtrFromGlobalId(id,sinkNodes,mobileSinkNode), 200, 250, 0);
	}else{
		Panim->UpdateNodeColor(GetNodePtrFromGlobalId(id,sinkNodes,mobileSinkNode), 0, 200, 255);
	}
	
}

void createNode(){
	// correctNodes.Create(1);//�???�???id和ip地址对应关系,方便调试
	//Create nodes
	
	senseNodes.Create(nNodes);
	// std::string traceFile = "scratch/NOMADIC/test.ns_movements";
	// Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	// ns2.Install ();
	sinkNodes.Create(nSinkNodes);
	//if(!isEntity){
		std::string traceFile = "scratch/FFOA/RPGM/ffoa_speed5.ns_movements";
		Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
		ns2.Install ();
	//}
	

	mobileSinkNode.Create(1);
	NS_LOG_DEBUG("Create nodes done!");
}


//移动模型
void createMobilityModel(){
	MobilityHelper mobility;
  /*  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                            "X", StringValue ("600"),
                            "Y", StringValue ("600"),
                           "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=100]"));
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
		mobility.Install(senseNodes); 

//		mobility.Install(sinkNodes);
		Ptr<ListPositionAllocator> lpa = CreateObject<ListPositionAllocator>();
	lpa->Add(Vector(600, 600, 0));
	mobility.SetPositionAllocator(lpa);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
		mobility.Install(mobileSinkNode);*/
    mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                             "X", StringValue ("500"),
                             "Y", StringValue ("500"),
                             "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=500]"));
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(senseNodes);     //为sense节点安�?�移动模�???
	Ptr<ListPositionAllocator> lpa = CreateObject<ListPositionAllocator>();
	lpa->Add(Vector(500, 500, 0));
	mobility.SetPositionAllocator(lpa);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(mobileSinkNode);
	//UAV移动模型
/*	ObjectFactory pos1;
		pos1.SetTypeId ("ns3::RandomRectanglePositionAllocator");
		pos1.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=25.0]"));
		pos1.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=25.0]"));
		Ptr<PositionAllocator> taPositionAlloc1 = pos1.Create ()->GetObject<PositionAllocator> ();*/

	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(sinkNodes);
	vector <Ptr<ConstantPositionMobilityModel> > cpmm(5);
		for (uint32_t i = 0; i < 5; i++)
			cpmm[i] =sinkNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
		cpmm[0]->SetPosition(Vector(600, 600, 0));
		cpmm[1]->SetPosition(Vector(300, 700, 0));
		cpmm[2]->SetPosition(Vector(800, 200, 0));
		cpmm[3]->SetPosition(Vector(100, 100, 0));
		cpmm[4]->SetPosition(Vector(1000, 1000, 0));
   NS_LOG_DEBUG("Create Mobility done!");
}


/*
 * 创建wifi通信设�??
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
	/** wifi channel 同时设置最大通信距�?? **/
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
 * 安�?�网络协�???栈，比�?�AODV等协�???，需要安装到该�?��?
 */
void installInternetStack(){
	InternetStackHelper stack2;
	stack2.Install (senseNodes);//注意，AODV协�??安�?�到nodes节点�???
	stack2.Install (sinkNodes);//注意，AODV协�??安�?�到nodes节点�???
   stack2.Install (mobileSinkNode);
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	
	senseIfs = ipv4.Assign(senseDevices);
	sinkIfs = ipv4.Assign(sinkDevices);
    mobileSinkIf = ipv4.Assign(mobileSinkDevice);
	NS_LOG_DEBUG("Install Internet stack done!");
}

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
	cmd.Parse(argc, argv);
	NS_LOG_DEBUG("Configure done!");
	//放在cmd.Parse()后面才会生效的变量赋�???
	simStopTime=totalTime;
	//step 3:创建仿真文件�???
	createSimFolder();
	//step 4:设置不同文件log级别
	setLog();
	//step 5:设置基�?�网�??
	netSet();
	//step 6:创建节点
	createNode();
	//step 7:创建并安装移动模�???
	createMobilityModel();
	//step 8:创建wifi设�??
	createWifiDevice();
	//step 9:安�?�网络协�???�???
	installInternetStack();
	//step 10:设置socket回调
	createSocketCallBack();
	//step 11:生成xml动画文件
	createXml();
	//step 12:节点安�?�能�???
	InstallEnergy(senseNodes,sinkNodes);
	cout<<"install energy done!"<<endl;
	//FlowMonitor set------    
	Ptr<FlowMonitor> flowMonitor;    
	FlowMonitorHelper flowHelper;    
	flowMonitor = flowHelper.InstallAll();
    //build map   节点id和地址的映射关系：
	buildMapIdAdr();
	cout<<"build idAdrMap done!"<<endl;
	initial(); 

	testIP();
	// Simulator::Schedule(Seconds(2.0),&startRoute,senseNodes.Get(0),GetNodeIpv4Address(senseNodes.Get(0)));
	//测试周期性邻居交�???信息
	Simulator::Schedule(Seconds(0.0),&update);   
	Simulator::Schedule(Seconds(2.0),&setSimStartTime);
	Simulator::Schedule(Seconds(2.5),&DataToSink);


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
	cout<<"网络时延:"<<delay/num<<endl;//单跳的平均时�?
	netDelay = delay/num;
	// cout<<"网络生命周期:"<<TIME_STAMP_FUC<<"(s)"<<endl;
	Simulator::Destroy();
	//step 17:数据收集
	finalRecord();
	//graph.print();
	return 0;
}





