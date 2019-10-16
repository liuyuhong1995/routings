#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"
#include "jump-count-table-chain.h"
#include "mobile-sink-record.h"
#include "sink-record.h"
#include "sense-sink-match-table.h"
#include "log-helper.h"

#include "math.h"
#include "time.h"
#include "stdlib.h"    //标准库头文件
#include <sys/stat.h>   //获取文件属性
#include <sys/types.h>   //基本系统数据类型
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
//#include "constant-velocity-loop-mobility-model.h-backup"

//ADD BY ZLB
#include "energy.h"
#include "final_record.h"
#include "sink_check.h"
#include "spt_route.h"
#include "data_gather.h"
#include "tool.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("sptMainTestScript");

/*
 * 定义UdpSocketFactory为socket的类型和socket的广播地址
 */
TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
InetSocketAddress broadcastAdr = InetSocketAddress(
 	Ipv4Address::GetBroadcast(), 80);

//全局网络设置
static std::string phyMode("DsssRate11Mbps");
static double Prss = -80; //dBm
static double offset = 81;  //I don't know
/*
 * Simulator initial global parameters
 */
//Time
static double sinkCheckDoneTime=0;
static double sptStartTime = 0;
static double sptDoneTime=0;
static double dataGenerationStartTime=0;
static double dataGatherStartTime=0;
double totalTime = 9999.0;  //max simulation time
clock_t simStartRealTime;
clock_t simFinishRealTime;

double simStopTime=0.0;
double dataInterval = 5.0; //senseNode感知数据后发送的间隔，也就是每dataInterval的间隔发送dataSize的包
double sinkCheckReqInterval = 1.0;//(5.0)移动汇聚请求汇聚检测的时间间隔
static double dataGatherReqInterval = sinkCheckReqInterval;
stringstream ssi;

//Topology
uint32_t nNodes = 20;
uint32_t nSenses = 0;
uint32_t nSinks = 0;
double maxDis = 52.0;
double maxX;//x length of the scenario
double maxY;
double sinkSpeed;
double mSinkTraceY;
bool gridTest=false;

//Data packet
uint32_t pktSize = 25*dataInterval;//25为单位时间senseNode产生的字节数
static double totalBytesGenerated;
double oldBytesGenerated=0;
double oldBytesGathered=0;
static int32_t maxPktSize=2000;//For test

//Energy
double initialJ = 5.0;
bool eTrace = true; //whether to enable the EnergySource trace
double totalConsumed=0;
uint32_t energyTraceIndex=0;//The global id of node to trace energy, default 0 means no trace
double TxPara=0.000006;//0.000006J/Bytes for Tx
double RxPara=0.000006;//0.000006J/Bytes for Rx
uint32_t nDrain=0;
double Tgather=0.65;

//Behavior
bool doMobileSink = false;

bool doSinkCheck = true;

//Algorithm
algorithmType aType = spt_Algor;
//uint32_t iterationCounts=100000;
//uint32_t initialPopNum = 50;//遗传算法初始种群数量
string algType;
//Others
bool Banim=false;
bool Brotate=true;//是否启动动态网关
bool Bdrain=true;
bool BsinglePkt=false;
AnimationInterface *Panim = 0;
uint32_t firstDrainedNodeId=0;
bool firstDrainFlag=true;
uint32_t firstDrainedSinkId=0;
double firstDrainedNodeTime=0;

string thisSimPath;//simulation path
string exeName="sptMain";
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
static NodeContainer allNodes;
static NodeContainer sinkNodes;
//NodeContainer sinkNodes;
static NodeContainer senseNodes;
static NodeContainer mobileSinkNode;

//Net device container
NetDeviceContainer senseDevices;
NetDeviceContainer mobileSinkDevice;

//Energy source pointer
EnergySourceContainer *senseSources=0;
double remaingJ[MAX_NUM];

//Ipv4 related container
Ipv4InterfaceContainer senseIfs;
Ipv4InterfaceContainer mobileSinkIf;

//Tables to record information
JumpCountTableChain jctChain;
MobileSinkRecord msRecord;
SinkRecord sRecord;
SenseSinkMatchTable *PssmTable=0;

/*
 * 将localAdr往sinkAdr的路由网关指向下一个gateway地址
 */
void GatewayRotate(Ipv4Address localAdr, Ipv4Address sinkAdr){
	jctChain.InquireNode(localAdr)->jcTable->InquireNode(sinkAdr)->gt->RotateTable();
}

/**
 * sink数据传输到接收器
 */
void TransmitDataPacketToMobileSink(Ptr<Socket> sct, Ptr<Packet> pkt) {
	if (CheckRemainingJ(sct->GetNode(), pkt)) {
		sct->Send(pkt);
		Ipv4Header h;
		Ptr<Packet> packet = pkt->Copy();
		packet->PeekHeader(h);
		packet->RemoveHeader(h);
		uint32_t pktSize=packet->GetSize();
		Ipv4Address sinkAdr=GetNodeIpv4Address(sct->GetNode());
		NS_LOG_INFO(
				TIME_STAMP_FUC<<sinkAdr<<" sent "
				<<pktSize<<"Bytes data to the mobile sink");
		UpdateEnergySources(sct->GetNode(), pkt, 0,sinkNodes,mobileSinkNode);
		msRecord.UpdateRecordNode(sinkAdr, 0, pktSize);   //接收比特数增多
		sRecord.UpdateAfterDataGathering(sinkAdr, (-1)*pktSize);  //datatosend减少
	}else
		NS_LOG_LOGIC(
					TIME_STAMP_FUC<<GetNodeIpv4Address(sct->GetNode())
					<<" failed in transmitting "<<pkt->GetSize()<<"Bytes to the mobile sink for energy");
}

///*
// * 设置接收端是否开始采集数据标志
// */
//void SetDataGatheringFlag(bool flag){
//	doMobileSink = flag;
//}

/*
 * 用于sinkNode节点接收mobileSink节点的数据请求
 */
void ReplyDataGatheringPacket(Ptr<Node> localSinkNode){
	Ipv4Address localSinkAdr = GetNodeIpv4Address(localSinkNode);
	int32_t dataToSend = sRecord.GetBytesToSend(localSinkAdr);
	if (dataToSend < 0) {
		NS_LOG_ERROR(TIME_STAMP_FUC<<
				"ERROR: get an invalid dataBytes from Node["
				<<localSinkNode->GetId()<< "] "<<localSinkAdr);
		NS_ASSERT(dataToSend>=0);
		Simulator::Stop();
	} else if (dataToSend == 0) {
		NS_LOG_INFO(
				TIME_STAMP_FUC<< "get 0KB data from Node["
				<<localSinkNode->GetId()<< "] "<<localSinkAdr<<", skip this node");
	} else {
		NS_LOG_INFO(TIME_STAMP_FUC<<localSinkAdr<<"(sink) buffered "
				<<dataToSend<<"Bytes data...");
		Ipv4Header h;
		h.SetIdentification(dataGathering_Type);
		h.SetDestination(mobileSinkIf.GetAddress(0));
		h.SetSource(localSinkAdr);
		double interval=0.0;
		while (dataToSend >= maxPktSize &&interval<dataGatherReqInterval/3) {
			NS_LOG_LOGIC(TIME_STAMP_FUC<<localSinkAdr<<", dataToSend="
					<<dataToSend<<", interval="<<interval);
			Ptr<Packet> gatheringPkt = Create<Packet>(maxPktSize);
			gatheringPkt->AddHeader(h);
			Ptr<Socket> scrSocket = Socket::CreateSocket(localSinkNode, tid);
			InetSocketAddress mobileSinkAdr = InetSocketAddress(
					mobileSinkIf.GetAddress(0), 80);
			scrSocket->Connect(mobileSinkAdr);
			Simulator::Schedule(Seconds(interval),
					&TransmitDataPacketToMobileSink, scrSocket, gatheringPkt);
			dataToSend -= maxPktSize;
			NS_ASSERT(dataToSend>=0);
			interval += 0.1;
		}
		if (dataToSend > 0 && dataToSend < maxPktSize) {
			NS_LOG_LOGIC(TIME_STAMP_FUC<<localSinkAdr<<", dataToSend="
							<<dataToSend<<", interval="<<interval);
			Ptr<Packet> gatheringPkt = Create<Packet>(dataToSend);
			gatheringPkt->AddHeader(h);
			Ptr<Socket> scrSocket = Socket::CreateSocket(localSinkNode, tid);
			InetSocketAddress mobileSinkAdr = InetSocketAddress(
					mobileSinkIf.GetAddress(0), 80);
			scrSocket->Connect(mobileSinkAdr);
			Simulator::Schedule(Seconds(interval),
							&TransmitDataPacketToMobileSink, scrSocket, gatheringPkt);
		}
	}
}
/*
 * 动态统计数据采集相关信息
 */
void DataGatheringStat(){
	ofstream of;
	stringstream ss;
	double timeNow=Simulator::Now().GetSeconds();
	double totalBytesGathered=msRecord.GetTotalRecvBytes();
	double dgp=totalBytesGathered*100.0/totalBytesGenerated;  //data get packet rate=收集的包/所有的包*100%
	ss<<thisSimPath<<exeName<<"-dynamic-DGP.stat";
	of.open(ss.str().c_str(), ios::app);
	ss.str("");
	ss.clear();

	ss<<setw(10)<<left<<timeNow;
	ss<<setw(10)<<left<<dgp<<endl;
	of<<ss.str().c_str();
	ss.str("");
	ss.clear();
	of.close();

	//EUE energy utilizaiton effeciency
	double eue = (totalBytesGathered / 1000) / totalConsumed;    //接收到单位比特所用的能量
	ss << thisSimPath << exeName << "-dynamic-EUE.stat";
	of.open(ss.str().c_str(), ios::app);
	ss.str("");
	ss.clear();

	ss << setw(10) << left << timeNow;
	ss << setw(10) << left << eue << endl;
	of << ss.str().c_str();
	ss.str("");
	ss.clear();
	of.close();
	oldBytesGathered=totalBytesGathered;
	oldBytesGenerated=totalBytesGenerated;
	if (dgp < Tgather * 100.0
			&& timeNow > (dataGatherStartTime + maxX * 10 / sinkSpeed)
			&& !firstDrainFlag) {
		NS_LOG_UNCOND(
				TIME_STAMP_FUC<<"dgp drops below "<<Tgather*100 <<"%, stop the simulation");
		simStopTime = timeNow;
		Simulator::Stop();
	}
	Simulator::Schedule(Seconds(dataInterval), &DataGatheringStat);  //dataInterval=5.0
}

/*
 * 收到dataGathering_Type的packet之后的处理
 */
static inline int32_t ProcessDataGatheringPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h) {
	//Get local node information
	nodeType nType = CheckNodeType(thisNode,sinkNodes,mobileSinkNode.Get(0));
	if (nType == sink_Type) {
		NS_LOG_INFO(
				TIME_STAMP_FUC<< GetNodeIpv4Address(thisNode)
				<<"(sink) received a data gathering Req");
		//产生随机的发包间隔
		double interval = RandomDoubleVauleGenerator(0.0, dataGatherReqInterval/3);  //dataGatherReqInterval=1.0
		Simulator::Schedule(Seconds(interval), &ReplyDataGatheringPacket, thisNode);
		return 0;
	} else {
		return 0;
	}
}


/*
 * 用于mobileSinkNode开始采集sinkNodes上已经缓存的数据，mobileSinkNode向sinkNodes发送数据收集类型的数据包
 */
void MobileSinkRequestDataGathering() {
	Ipv4Header h;
	h.SetIdentification(dataGathering_Type);
	h.SetSource(GetNodeIpv4Address(mobileSinkNode.Get(0)));
	Ptr<Packet> mobileSinkPkt = Create<Packet>(0);
	mobileSinkPkt->AddHeader(h);
	Ptr<Socket> mobileSinkSocket = Socket::CreateSocket(mobileSinkNode.Get(0),
			tid);
	mobileSinkSocket->Connect(broadcastAdr);
	mobileSinkSocket->SetAllowBroadcast(true);	//socket发送广播必须有这么一个设置
	mobileSinkSocket->Send(mobileSinkPkt); // UDP 广播形式的包
	mobileSinkSocket->Close();
	Simulator::Schedule(Seconds(dataGatherReqInterval),&MobileSinkRequestDataGathering);//dataGatherReqInterval=1.0
	NS_LOG_LOGIC(std::endl<<TIME_STAMP_FUC<<"...");
}
/*
 * mobileSinkNode准备发送数据请求
 */
void PrepareRequestDataGathering() {
	if (doMobileSink) {   //一开始为false,直到路由表建立后准备数据传输时变为true
		double timeNow=Simulator::Now().GetSeconds();
		double nextTurnPoint=0.0;
		while(nextTurnPoint<=timeNow)
			nextTurnPoint += maxX / sinkSpeed;
		dataGatherStartTime=nextTurnPoint;        
		double delay=nextTurnPoint-timeNow;
		NS_LOG_DEBUG(std::endl<<TIME_STAMP
				<<"(MobileSinkRequestDataGathering) Will start in "<<delay<<" seconds..."<<std::endl);
		Simulator::Schedule(Seconds(delay), &MobileSinkRequestDataGathering);
		Simulator::Schedule(Seconds(delay+maxX*1.0/sinkSpeed), &DataGatheringStat);
	} else {
		Simulator::Schedule(Seconds(1.0), &PrepareRequestDataGathering);
	}
}
/*
 *用于发送节点之间发送数据local->remote
 *首先查找localNode到remote的gateway，如果找不到，说明到不了，报错
 *如果找到了，就通过socket给gateway发送这个pkt
 *pkt里的Ipv4Header的source是
 */
void TransmitDataPacket(Ptr<Node> localNode, Ipv4Address sourceAdr,
		Ipv4Address sinkAdr) {
	NS_LOG_LOGIC(
			std::endl<<TIME_STAMP_FUC <<sourceAdr <<" to "<<sinkAdr<<" by "
			<<GetNodeIpv4Address(localNode));
	if (CheckRemainingJ(localNode)) {
		Ipv4Address localAdr = GetNodeIpv4Address(localNode);
		NS_LOG_LOGIC(TIME_STAMP_FUC<<"CheckRemainingJ for "<<localAdr<<" passed!");
		pktType pktType = data_Type;
		//find gateway
		Ipv4Address gatewayAdr = jctChain.GetRoutingGatewayAdr(localAdr,
				sinkAdr);
		if (gatewayAdr == Ipv4Address("0.0.0.0")) {
			NS_LOG_DEBUG(
					TIME_STAMP_FUC<<"WARN: No gateway found for localAdr = " <<localAdr<<" to sinkAdr ="<<sinkAdr);
		} else {
			if (localAdr == sourceAdr) {  //当前地址有可能不是源地址，这样就不需要改变生成比特了，因为在源地址发包时改变过了
				NS_LOG_INFO(
						TIME_STAMP_FUC<<sourceAdr<<"(sense) sent a data packet to "<<sinkAdr);
				totalBytesGenerated += pktSize;  //总生成比特是感知节点的比特数
			}
			NS_LOG_LOGIC(
					TIME_STAMP_FUC<<"localAdr = "<<localAdr <<", sinkAdr = "<<sinkAdr<<", gatewayAdr = "<<gatewayAdr);

			Ptr<Packet> dataPkt = Create<Packet>(pktSize);
			//packet header
			Ipv4Header h;
			h.SetDestination(sinkAdr);
			h.SetSource(sourceAdr);
			h.SetIdentification(pktType);
			dataPkt->AddHeader(h);
			//send packet by socket
			InetSocketAddress gateAdr = InetSocketAddress(gatewayAdr, 80);  //发送数据包需要通过网关地址
			Ptr<Socket> srcSoc = Socket::CreateSocket(localNode, tid);
			srcSoc->Connect(gateAdr);
			srcSoc->Send(dataPkt);
			UpdateEnergySources(localNode, dataPkt, 0,sinkNodes,mobileSinkNode);
			NS_LOG_LOGIC(TIME_STAMP_FUC<<"Socket from "<<localAdr<<" launched!");
			if (Brotate == true && aType == my_gene_Algor) {
				GatewayRotate(localAdr, sinkAdr);
			}
		}
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
		nodeType nType = CheckNodeType(thisNode,sinkNodes,mobileSinkNode.Get(0));
		Ipv4Address srcAdr = GetNodeIpv4Address(thisNode);
		switch (nType) {
		case sense_Type: {	//sense_Type就往sinkNode发送数据包
			if (!CheckRemainingJ(thisNode)) {
				continue;
			} else {
				TableNode *thisTn = jctChain.InquireNode(srcAdr);
				if (thisTn != NULL) {//Means this senseNode can reach at least one sinkNode
					Ipv4Address dstAdr = PssmTable->GetSinkAdr(srcAdr);
					Simulator::Schedule(Seconds(interval), &TransmitDataPacket,
							thisNode, srcAdr, dstAdr);
					/*
					 * 每个发送周期dataInterval一开始，
					 * 所有senseNode按照一个根据senseNode的数量的时间间隔上报数据
					 */
					interval+=(dataInterval-1.0)*1.0/(nSenses-nSinks);   //datainteval=5.0
				}
				break;
			}
		}
		case sink_Type: {	//sink_Type就直接在sinkRecord里写入数据
			sRecord.UpdateRecordNode(srcAdr, srcAdr, pktSize);
			totalBytesGenerated+=pktSize;
			break;
		}
		default:
			break;
		}
	}
	if (!BsinglePkt)
		Simulator::Schedule(Seconds(dataInterval), &DataToSink);
}
/*
 * 数据发送的准备工作
 * 判断当前时间和sptDoneTime之间的差，
 * 如果当前时刻比sptDoneTime晚5秒以上就开始执行以下流程：
 * 1、生成SenseSinkMatchTable，即通过aType算法来完成senseNode到sinkNode的匹配
 * 2、设置GeneticMatchAlgorithm的初始种群数量
 * 3、开始发送数据到sinkNodes
 * 4、设置DataGatheringFlag为true，即mobileSink开始发送DataGatherRequest
 * 用以判断spt routing是否已经结束
 */
void PrepareDataToSink() {
	if ((sptDoneTime + 5) < Simulator::Now().GetSeconds()) {
		NS_LOG_DEBUG(TIME_STAMP<<"SPT done, sptDoneTime = "<<sptDoneTime);
		stringstream ss;
		ss<<thisSimPath<<exeName<<"-original.route";
		jctChain.ListJumpCountTableChain(ss.str().c_str());  // 列出所有Sense节点的地址，sink节点地址，跳数和网关地址
		ss.str("");
		ss.clear();
		NS_LOG_DEBUG(std::endl<<TIME_STAMP_FUC<<"...");
		RemoveFutileNodes(senseNodes,senseDevices,sinkNodes,mobileSinkNode);   // 移除无效节点
		NS_LOG_DEBUG(TIME_STAMP_FUC<<"RemoveFutileNodes"<<",nSenses="
				<<nSenses<<" including nSinks="<<nSinks);
		PssmTable = new SenseSinkMatchTable(jctChain, sinkNodes, aType,thisSimPath,exeName);  
//		if (aType == gene_Algor || aType == my_gene_Algor) {
//			NS_LOG_LOGIC("SetPopNum and  SetGeneIteration");
//			PssmTable->SetPopNum(initialPopNum);
//			PssmTable->SetGeneIteration(iterationCounts);
//		}
		NS_LOG_DEBUG(std::endl<<TIME_STAMP_FUC
				<<"Do sense sink match, algorithm="<<aType);
		PssmTable->DoMatch();   
		NS_LOG_DEBUG(TIME_STAMP_FUC<<"Sense sink match, done");
		double timeNow = Simulator::Now().GetSeconds();
		double nextTurnPoint = 0.0;
		while(nextTurnPoint<=timeNow)
			nextTurnPoint+= maxX / sinkSpeed;
		dataGatherStartTime = nextTurnPoint;
		double delay = nextTurnPoint - timeNow;
		dataGenerationStartTime = nextTurnPoint;
		NS_LOG_DEBUG(std::endl<<TIME_STAMP<<"(DataToSink) will start in "<<delay<<" seconds...");
		Simulator::Schedule(Seconds(delay), &DataToSink);  
		Simulator::Schedule(Seconds(0.0), &SetDataGatheringFlag , true);  //domobilesink=true

	} else {
		Simulator::Schedule(Seconds(5.0), &PrepareDataToSink);
	}
}

/*
 *用于senseNodes和mobileSinkNode收到sinkCheck_Type的packet后的处理方法
 */
static inline int32_t ProcessSinkCheckPacket(Ptr<Node> thisNode,
		Ptr<Packet> packet, Ipv4Header h) {
	nodeType nType = CheckNodeType(thisNode,sinkNodes,mobileSinkNode.Get(0));	//获取节点类型
	stringstream recvData;     //用于存放packet中的内容
	packet->CopyData(&recvData, packet->GetSize());	//获取Packet的内容
	Vector sourceLocation = Vector(0, 0, 0);
	switch (nType) {
	case mobileSink_Type: {
		cout<<"4：回复广播包以后，Mobile节点收到广播包然后从sense中分离出sink====="<<endl;
		NS_LOG_LOGIC(FUC<<"mobileSink_Type");
		Ipv4Address sourceAdr = h.GetSource();	//或者Packet的source节点地址
		AnalyzeSinkCheckPacket(recvData, sourceLocation);  //解析汇聚检测包的内容（收到汇聚检测包的感知节点的地址）

		NS_LOG_LOGIC(TIME_STAMP<<"Mobile sink received a sink check reply from "
						<<sourceAdr<<", inquire it in mobile sink record...");
		MobileSinkRecordNode *msrN = msRecord.InquireRecordNode(sourceAdr);	//获取source节点的SinkRecord
		if (msrN != NULL) {	//照理说不可能发生，因为如果已经存在了这个SinkRecordNode，sinkNode是不会再给mobileSinkNode发送这个包的
			return msRecord.UpdateRecordNode(sourceAdr, 0, 0);
		} else {
			cout<<"5:进入sink节点分类处理====="<<endl;
			NS_LOG_INFO(TIME_STAMP<<"Ascertained "
								<<sourceAdr
								<<", insert it into MobileSinkRecord and color to blue");
			if (Banim) {
				Panim->UpdateNodeColor(GetNodePtrFromIpv4Adr(sourceAdr,sinkNodes,senseNodes,mobileSinkNode), 0, 0,
						255);      //把sink节点变成蓝色
			}
			sRecord.InsertRecordNode(sourceAdr);  //在汇聚记录中插入这个源地址表示该节点变成了汇聚节点
			sinkNodes.Add(GetNodePtrFromIpv4Adr(sourceAdr,sinkNodes,senseNodes,mobileSinkNode));
			cout<<"测试用：====="<<sinkNodes.GetN()<<endl;
			nSinks++;
			return msRecord.InsertRecordNode(sourceAdr, sourceLocation, 0);   //源地址、坐标、剩余能量
		}
		break;
	}
	default: {
		NS_LOG_LOGIC(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
				<<" received a sink check packet from mobileSinkNode, inquire it in mobile sink record...");
		NS_LOG_LOGIC(FUC<<"default");
		MobileSinkRecordNode *msrN = msRecord.InquireRecordNode(GetNodeIpv4Address(thisNode));	//获取source节点的SinkRecord
		if (msrN == NULL) {	//SinkRecord里没有这个节点的信息
			//产生随机的发包间隔
				double interval = RandomDoubleVauleGenerator(0.0, 0.5);
				NS_LOG_LOGIC(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
							<<" is for the 1st time to receive a sink check packet, prepare to reply it...");
			Ptr<ConstantPositionMobilityModel> cpmm = thisNode->GetObject<
					ConstantPositionMobilityModel>();
			sourceLocation = cpmm->GetPosition();	//获取当前节点的坐标

			cout<<"2:sense节点收到sink check广播包通过回调将进入ReplySinkCheck处理下一步====="<<endl;
			Simulator::Schedule(Seconds(interval), &ReplySinkCheck, thisNode,
					sourceLocation);
		}else
			NS_LOG_LOGIC(
					TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
					<<" is already in mobile sink record, ignore this sink check packet");
		break;
	}
	}
	return 0;
}
/*
 * 用于sinkNode收到data_Type的packet后的处理方法
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
					<<h.GetSource()<<" to "<<h.GetDestination());
		double interval = RandomDoubleVauleGenerator(0.0, 0.5);
		Simulator::Schedule(Seconds(interval), &TransmitDataPacket, thisNode,
				srcAdr, dstAdr);
		return 0;
	}
	//destination node received the packet, stored the data
	NS_LOG_INFO(TIME_STAMP_FUC<<
						GetNodeIpv4Address(thisNode)<<"(sink) received a data packet from "
						<<h.GetSource());
	sRecord.UpdateRecordNode(localAdr, srcAdr, packet->GetSize());
	return 0;
}


/*
 * 用于senseNode收到SPT packet之后处理的内联函数
 */
static inline int32_t ProcessSptPacket(Ptr<Node> thisNode, Ptr<Packet> packet,
		Ipv4Header h) {
	double interval = RandomDoubleVauleGenerator(0.0, 0.5);	//产生随机的发包间隔
	double interval2 = RandomDoubleVauleGenerator(0.5, 1.0);
	Ipv4Address gateway = h.GetSource();	//get gateway Ipv4Address。这个为什么能获得网关地址？
	Ipv4Address local = GetNodeIpv4Address(thisNode);	//get local Ipv4Address
	stringstream recvData;
	packet->CopyData(&recvData, packet->GetSize());  //把packet中的内容存放到recvData中
	Ipv4Address source;  //此时source是未知的
	uint32_t pktJumps = AnalyzeSptPacket(recvData, source);//Get jumps and source address from SPT packet
	NS_LOG_LOGIC(
			TIME_STAMP_FUC<< local<<" received a SPT packet, gateway= "<<gateway <<", pktJumps = "
			<<pktJumps<<", source = "<<source);
/*	if (source == "10.1.1.25") {
		NS_LOG_DEBUG(
				TIME_STAMP_FUC<< local<<" received a SPT packet, gateway= "
				<<gateway <<", pktJumps = " <<pktJumps<<", source = "<<source);
	}*/
	TableNode *tN = jctChain.InquireNode(local);	//check TableNode
	if (tN == NULL) {	//it could not happen if everything is right（为啥不可能发生？）
		NS_LOG_LOGIC(TIME_STAMP_FUC<<"local = "<<local<<" is not in the chain, insert it");
		jctChain.InsertNode(local);
		tN = jctChain.InquireNode(local);
		tN->jcTable->UpdateTableNode(source, gateway, pktJumps + 1);   //source为汇聚节点的地址
		Simulator::Schedule(Seconds(interval), &ContinueSptRouting, thisNode,
				(pktJumps + 1), source);
		Simulator::Schedule(Seconds(interval+interval2), &ContinueSptRouting, thisNode,
				(pktJumps + 1), source);
		return 0;
	}
	RoutingNode *rN = jctChain.InquireRoutingNode(local, source);
	if (rN == NULL) {  //上面已经检查过感知节点了，所以这里是空的，说明没有这条路由
		NS_LOG_LOGIC(TIME_STAMP_FUC<<
				"sinkSource = "<<source<<" is not in the JumpCountTable of local = " <<local<<", insert it");
	/*	if (source == "10.1.1.25") {
			NS_LOG_DEBUG(TIME_STAMP_FUC<<
							"sinkSource = "<<source<<" is not in the JumpCountTable of local = " <<local<<", insert it");
		}*/
	tN->jcTable->UpdateTableNode(source, gateway, pktJumps + 1);
		Simulator::Schedule(Seconds(interval), &ContinueSptRouting, thisNode,
				(pktJumps + 1), source);
		Simulator::Schedule(Seconds(interval2), &ContinueSptRouting, thisNode,
				(pktJumps + 1), source);
		Simulator::Schedule(Seconds(interval+interval2), &ContinueSptRouting, thisNode,
				(pktJumps + 1), source);
		return 0;
	}
	uint32_t localJumps = rN->jumps;
	if ((pktJumps + 1)
			== localJumps && rN->gt->InquireNode(gateway)==NULL) {
		NS_LOG_LOGIC(TIME_STAMP_FUC<<"localJumps == pktJumps+1, insert the route node");
		rN->gt->InsertNode(gateway);
		return 0;
	}
	else if ((pktJumps + 1) < localJumps) {
		NS_LOG_LOGIC(
				TIME_STAMP_FUC<< "localJumps > pktJumps+1, update the gateway and jumps of local ="
				<<local<<" to sinkAdr = "<<source);
		tN->jcTable->UpdateTableNode(source, gateway, pktJumps + 1);
		Simulator::Schedule(Seconds(interval), &ContinueSptRouting, thisNode,
				(pktJumps + 1), source);
		Simulator::Schedule(Seconds(interval + interval2), &ContinueSptRouting,
				thisNode, (pktJumps + 1), source);
		return 0;
	}
	else {
		NS_LOG_LOGIC(TIME_STAMP_FUC<<"Ignore this packet");
		sptDoneTime = Simulator::Now().GetSeconds();
		return 0;
	}
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
			nType = CheckNodeType(thisNode,sinkNodes,mobileSinkNode.Get(0));

			switch (nType) {
			case mobileSink_Type: {	//mobileSinkNode收到packet
				switch (pType) {
				case sinkCheck_Type:{///收到sinkCheck_Type的packet
					ProcessSinkCheckPacket(thisNode, packet, h);
//					ProcessSinkCheckPacket(thisNode, packet, h,sinkNodes,mobileSinkNode,senseNodes);
					break;
				}
				case data_Type:///收到data_Type的packet
					break;
				case dataGathering_Type:
					ProcessDataGatheringPacket(thisNode, packet, h);
					break;
				default:
					break;
				}
				break;
			}

			case sink_Type: {	//sinkNode收到packet
				switch (pType) {
				case sinkCheck_Type: {///收到sinkCheck_Type的packet
					ProcessSinkCheckPacket(thisNode, packet, h);
//					ProcessSinkCheckPacket(thisNode, packet, h,sinkNodes,mobileSinkNode,senseNodes);
					break;
				}
				case data_Type: {///收到data_Type的packet，汇聚数据
					if (CheckRemainingJ(thisNode)) {
						UpdateEnergySources(thisNode, packet, 1,sinkNodes,mobileSinkNode);
						ProcessDataPacket(thisNode, packet, h);
					}
					break;
				}
				case dataGathering_Type: {
					ProcessDataGatheringPacket(thisNode, packet, h);
					break;
				}
				default:
					break;
				}
				break;
			}

			case sense_Type: {	//senseNode收到packet
				switch (pType) {
				case data_Type: {///收到data_Type的packet，将执行中继
					if (CheckRemainingJ(thisNode, packet)) {
						UpdateEnergySources(thisNode, packet, 1,sinkNodes,mobileSinkNode);
						ProcessDataPacket(thisNode, packet, h);
					}
					break;
				}
				case spt_Type: {///收到spt_Type的packet，将continue spt routing
					ProcessSptPacket(thisNode, packet, h);
					break;
				}
				case sinkCheck_Type: {///收到sinkCheck_Type的packet，回复mobileSinkNode
					cout<<TIME_STAMP<<"一开始sense节点收到sink check广播包进入接收回调"<<endl;
					ProcessSinkCheckPacket(thisNode, packet, h);
//					ProcessSinkCheckPacket(thisNode, packet, h,sinkNodes,mobileSinkNode,senseNodes);
					break;
				}
				case dataGathering_Type: {///收到dataGathering_Type的packet，说明这个节点在sinkcheck的时候没被检测到
					NS_LOG_WARN(
							TIME_STAMP_FUC<<"WARN: "<<GetNodeIpv4Address(thisNode)
							<<" received a data gathering Req from mobileSink");
					break;
				}
				case drainNotice_Type: {///收到drainNotice_Type的packet，说明这个节点要把source节点从gatewayTable上删除
					ProcessDrainNoticePacket(thisNode, packet, h);
					break;
				}
				default: {
					NS_LOG_DEBUG(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
							<<" received "<<packet->GetSize()<<"Bytes, packet type="<<nType<<", from "<<h.GetSource());
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
 * 测试sinkNode作为参数引用传递问题
 */
/*NodeContainer nodeTmp;

void sinkNumTest(){
	cout<<"进入sinkTest====="<<endl;
	cout<<"当前时刻为："<<TIME_STAMP<<endl;
	cout<<"nodeTmp节点数："<<nodeTmp.GetN()<<endl;
	cout<<"sinkNodes的节点数为："<<sinkNodes.GetN()<<endl;
}

void nodeTmpTest(){
	nodeTmp.Create(5);
}*/

void delieveryPrepareSpt(){    //传送准备的spt
//	cout<<"进入sinkTest====="<<endl;
//	cout<<"当前时刻为："<<TIME_STAMP<<endl;
//	cout<<"nodeTmp节点数："<<nodeTmp.GetN()<<endl;
//	cout<<"sinkNodes的节点数为："<<sinkNodes.GetN()<<endl;
	PrepareSptRouting(sinkNodes,mobileSinkNode);
}

/*
 * 创建系统仿真文件夹
 */
void createSimFolder(){
	time_t rawTime;    //定义时间变量值rawTime
	time(&rawTime);    //取当前工作时间值，并赋值给rawTime
	struct tm *timeInfo;   //定义tm的结构指针
	timeInfo = localtime(&rawTime); //localtime()将参数rawtime 所指的time_t 结构中的信息转换成真实世界所使用的时间日期表示方法,然后将结果由结构timeinfo返回
	char pblgtime[20];
	strftime(pblgtime, 20, "%Y-%m-%d %X", timeInfo);  //将时间以自定义的格式显示
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
	//Loglevel index
	uint16_t mainLogIndex = 31;
	uint16_t jctLogIndex = 1;
	uint16_t jctcLogIndex = 1;
	uint16_t rfsrLogIndex = 1;
	uint16_t srLogIndex = 1;
	uint16_t msrLogIndex =1;
	uint16_t wifiLogIndex = 0;
	uint16_t ssmtLogIndex = 1;
	uint16_t gmaLogIndex =1;

	//set log level
	uint16_t logIndexArray[9] = { mainLogIndex, jctLogIndex, jctcLogIndex,
			rfsrLogIndex, srLogIndex, msrLogIndex, wifiLogIndex, ssmtLogIndex,
			gmaLogIndex };

	vector<LogLevel> logLevel(9);
	LogHelper *logHelper = new LogHelper(logIndexArray);
	logHelper->SetLogLevel(logLevel[0], logLevel[1], logLevel[2], logLevel[3],
			logLevel[4], logLevel[5], logLevel[6], logLevel[7], logLevel[8]);

	LogComponentEnable("sptMainTestScript", logLevel[0]);
	LogComponentEnable("JumpCountTable", logLevel[1]);
	LogComponentEnable("JumpCountTableChain", logLevel[2]);
	LogComponentEnable("RecvFromSenseRecord", logLevel[3]);
	LogComponentEnable("SinkRecord", logLevel[4]);
	LogComponentEnable("MobileSinkRecord", logLevel[5]);
	LogComponentEnable("SenseSinkMatchTable", logLevel[7]);
//	LogComponentEnable("GeneticMatchAlgorithm", logLevel[8]);
	LogComponentEnable("AdhocWifiMac", logLevel[6]);
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
	mobileSinkNode.Create(1);
	allNodes.Create(nNodes);
	senseNodes.Add(allNodes);
	NS_LOG_DEBUG("Create nodes done!");
}
/*
 * 创建移动模型并安装到节点
 */
void createMobilityModel(){
	//Install mobility
	MobilityHelper mobility;
	//test grid position allocator
	if (gridTest) {
		mobility.SetPositionAllocator("ns3::GridPositionAllocator", "GridWidth",
				UintegerValue(5), "MinX", DoubleValue(0.0), "MinY",
				DoubleValue(0.0), "DeltaX", DoubleValue(10.0), "DeltaY",
				DoubleValue(10.0));
		mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
		mobility.Install(senseNodes);     //为sense节点安装移动模型

		maxX = 40;
		maxY = 10*(std::ceil(nNodes*1.0/5));
		maxY=40;
		mSinkTraceY = maxY;
		maxDis = 15;
		sinkCheckDoneTime = (maxX * 2) / sinkSpeed;

		vector <Ptr<ConstantPositionMobilityModel> > cpmm(5);
		for (uint32_t i = 0; i < 5; i++)
			cpmm[i] =
					senseNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
		cpmm[0]->SetPosition(Vector(20, 30, 0));
		cpmm[1]->SetPosition(Vector(20, 20, 0));
		cpmm[2]->SetPosition(Vector(10, 10, 0));
		cpmm[3]->SetPosition(Vector(30, 10, 0));
		cpmm[4]->SetPosition(Vector(20, 0, 0));
	}else {
		mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
				"X",
				StringValue("ns3::UniformRandomVariable[Min=0.0|Max=400.0]"),
				"Y",
				StringValue("ns3::UniformRandomVariable[Min=0.0|Max=400.0]"));
		mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
		mobility.Install(senseNodes);
	}

	Ptr<ListPositionAllocator> lpa = CreateObject<ListPositionAllocator>();
	lpa->Add(Vector(0, mSinkTraceY/2, 0));
	mobility.SetPositionAllocator(lpa);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(mobileSinkNode);
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

	/** wifi channel **/
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
	//wifiPhy.EnablePcap(ssi.str().c_str(), mobileSinkNode.Get(0)->GetId(), 0, true);
	ssi.str("");
	ssi.clear();

	NS_LOG_DEBUG("Create devices done!");
	NS_LOG_DEBUG("Set pcap and anmi done!");
}
/*
 * 安装网络协议栈
 */
void installInternetStack(){
	/**Install Internet Stack**/
	InternetStackHelper stack;
	stack.Install(senseNodes);
	stack.Install(mobileSinkNode);

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
	stringstream ssi;
	if (Banim) {
		ssi<<thisSimPath<<exeName<<".xml";
		Panim = new AnimationInterface(ssi.str().c_str());
		ssi.str("");
		ssi.clear();
		Panim->SetMaxPktsPerTraceFile(50000000);
		Panim->UpdateNodeColor(mobileSinkNode.Get(0), 0, 255, 0);  //ms节点是绿色的
		Panim->UpdateNodeSize(mobileSinkNode.Get(0)->GetId(), 10.0, 10.0);  //ms节点较大
		for (NodeContainer::Iterator i = senseNodes.Begin();
				i != senseNodes.End(); i++) {
			Ptr<Node> n = *i;
			Panim->UpdateNodeSize(n->GetId(), 6.0, 6.0);   //感知节点较小
		}
	}
	NS_LOG_DEBUG("Set anmi done!");
}
void finalRecord(){
	/**Record the total Bytes gathered, network lifetime, the percentage of data gathered/data generated**/
	totalBytesGathered = msRecord.GetTotalRecvBytes();
	totalEnergyConsumed =totalConsumed;
	networkLifetime = simStopTime;
	if (totalBytesGathered != 0)
		dataGatherPercentage = totalBytesGathered * 1.0 / totalBytesGathered;
	if (totalBytesGenerated != 0)
		dataGatherPercentage = totalBytesGathered * 1.0 / totalBytesGenerated;
	RngRun =ns3::SeedManager::GetRun();
	simFinishRealTime = clock();
	simRunTime = (simFinishRealTime-simStartRealTime)*1.0/CLOCKS_PER_SEC;
	EnergyConsumeRate=totalEnergyConsumed/networkLifetime;

	totalJumps=PssmTable->GetTotalJumps();
	sinkVariance = PssmTable->GetSinkVariance();   //汇聚变化

	stringstream si;
	si<<"spt";
	algType = si.str();
	si.str("");
	si.clear();

	FindInrangeNodes(allNodes);    //寻找范围内的节点
	recordtable();
	recordfile();
	EnergyFinalRecord(senseNodes, remaingJ);
	gnuplot();
}
/*
 * 工程入口
 */
int main(int argc, char* argv[]) {
	//step0:全局变量初始化
	sinkSpeed = 5.0;  //sink moving speed
	maxX=40.0;
	maxY = 40.0; //the boundary of the scenario(40*40)
	sinkCheckDoneTime = (maxX * 2) / sinkSpeed;//Time to stop sink check
	mSinkTraceY = maxY/2; //the y value of sink path
	simStartRealTime = clock();

	//step 1 :定义main中局部变量
	double simStartTime = 0.0;  //seconds

	//step 2:解析命令行输入参数
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of Nodes", nNodes);
	cmd.AddValue("totalTime", "Simulation time length", totalTime);
	cmd.AddValue("simStartTime", "Simulation start time", simStartTime);
	cmd.AddValue("maxDis", "max distance to transmit", maxDis);
	cmd.AddValue("sinkSpeed", "sinkSpeed", sinkSpeed);
	cmd.AddValue("eTrace", "whether to log energy module", eTrace);
	cmd.AddValue("initialJ","Initial energy of each BaiscEnergySource", initialJ);
	cmd.AddValue("rewrite", "Whether to rewrite the result-sptMain.record",rewrite);
	cmd.AddValue("sinkCheckReqInterval", "The time interval of sink check request", sinkCheckReqInterval);
	cmd.AddValue("dataGatherReqInterval", "The time interval of data gathering request", dataGatherReqInterval);
//	cmd.AddValue("initialPopNum","Number of initial population for genetic algorithm ",initialPopNum);
//	cmd.AddValue("iterationCounts","Iteration times for GeneticMatchAlgorithm",iterationCounts);
	cmd.AddValue("pktSize","Size of data packe",pktSize);
	cmd.AddValue("gridTest","Whether to use grid tes",gridTest);
	cmd.AddValue("Banim","Whether to generate animatit",Banim);
	cmd.AddValue("energyTraceIndex","The global id of node to trace energy",energyTraceIndex);
	cmd.AddValue("dataInterval","The time interval of data generation",dataInterval);
	cmd.AddValue("TxPara","The parameter of Tx",TxPara);
	cmd.AddValue("RxPara","The parameter of Rx",RxPara);
	cmd.AddValue("Brotate","Whether to enable gateway rotation ",Brotate);
	cmd.AddValue("BsinglePkt","Whether to enable single pkt test ",BsinglePkt);
	cmd.AddValue("Bdrain","Whether to enable drain notice broadcast ",Bdrain);
	cmd.AddValue("Tgather","The percentage of threshold of data gathering percentage,(0,1) ",Tgather);
	cmd.Parse(argc, argv);    

	//Log enable
	LogComponentEnable("RectangleAmend", LOG_LEVEL_ERROR);
	LogComponentEnable("GatewayTable", LOG_LEVEL_ERROR);
	NS_LOG_DEBUG("this is input !");
	NS_LOG_DEBUG("Configure done!");
	//放在cmd.Parse()后面才会生效的变量赋值
	nSenses = nNodes;
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
	//  需要改动的地方  --- *** -- 移动模型
	createMobilityModel();
	//step 8:创建wifi设备
	createWifiDevice();
	//step 9:安装网络协议栈
	installInternetStack();  
	//step 10:设置socket回调
	createSocketCallBack();  //节点收到包后回调
	//step 11:生成xml动画文件
	createXml();
	//step 12:节点安装能量
	InstallEnergy(senseNodes);   //开始时只有感知节点和ms节点，所以只在感知节点安装能量即可

	//step 13:sinkCheck检测开始与停止
	Simulator::Schedule(Seconds(simStartTime), &PrepareRequestSinkCheck,mobileSinkNode); //simStartTime为0
	Simulator::Schedule(Seconds(simStartTime), &PrepareRequestDataGathering);
	Simulator::Schedule(Seconds(sinkCheckDoneTime),&SetSinkCheckFlag,false); //sinkCheckDoneTime = (maxX * 2) / sinkSpeed=16s
	Simulator::Schedule(Seconds(sinkCheckDoneTime), &SortSinkNodesByAdr, sinkNodes);
	//step 14：开始spt路由发现
	sptStartTime = sinkCheckDoneTime + 1;
	//解耦代码时候，这里出现参数传递问题，测试用   暂时改为中间函数传递的方法

	//  ********  需要改的地方
	Simulator::Schedule(Seconds(sptStartTime), &delieveryPrepareSpt); //sptStartTime=17s
	//step 15：开始数据传输
	Simulator::Schedule(Seconds(sptStartTime+5),&PrepareDataToSink);//这里设置了Mobile接收端数据采集开始标志
	//for test
	if (gridTest) {
		Simulator::Schedule(Seconds(1000), &ManualDrainTest, energyTraceIndex);
	}
	//step 16:仿真相关
	Simulator::Stop(Seconds(totalTime));
	Simulator::Run();
	Simulator::Destroy();
	//step 17:数据手收集
	finalRecord();   //
	return 0;
}
