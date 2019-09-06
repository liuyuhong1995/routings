/*
*/

#include "ns3/flow-monitor-module.h"
#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h" 
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ns2-mobility-helper.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

#include "energy.h"
#include "tool.h"

using namespace ns3;
using namespace std;

clock_t lifeTime;
string mobilityModel;
bool isLifeCycle;
bool isEntity;

NS_LOG_COMPONENT_DEFINE("aodvScript");

static int packetsSent = 0;
static int packetsReceived = 0;
Time netDelay;
// Time cycle;

static double Prss = -80; //dBm
static double offset = 81;  //I don't know
static std::string phyMode("DsssRate11Mbps");

//统计相关
static double simRunTime=0.0;
clock_t simStartRealTime;
clock_t simFinishRealTime;

//统计时延
static Time receivetime;
static Time sendtime;

NodeContainer c;
NodeContainer nodes;
NodeContainer mobileSinkNode;

NetDeviceContainer cDevices;
Ipv4InterfaceContainer cIfs;


//全局变量声明
uint32_t size=10;
double step=100;

int pktSize = 1500;
int maxPackets = 900;
double interval = 8;
Time interPacketInterval = Seconds (interval);

//最大通信距离
double maxDis = 225;
//能量模型相关
double TxPara=0.000006;//0.000006J/Bytes for Tx
double RxPara=0.000006;//0.000006J/Bytes for Rx
double initialJ = 5.0;
uint32_t nDrain=0;
double Tgather=0.65;
double totalConsumed=0;

AnimationInterface *Panim;
bool Banim=true;
bool isStatic  = false;


double remaingJ[MAX_NUM];

bool firstDrainFlag=true;
uint32_t firstDrainedNodeId=0;
double firstDrainedNodeTime=0;

string thisSimPath = "/home/wsn/sim_temp/aodv";
string exeName = "aodv";
// stringstream ssi;


TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

void RecvPacketCallback (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  simRunTime = Simulator::Now().GetSeconds();
  while ((packet = socket->Recv ()))
    {
		Ptr<Node> thisNode = socket->GetNode();
       // Ipv4Address address = GetNodeIpv4Address(thisNode);
		if (CheckRemainingJ(thisNode, packet)) {
			// UpdateEnergySources(thisNode, packet, 1,mobileSinkNode);
			Ipv4Header h;
			packet->PeekHeader(h);
			//Ipv4Address sourceAdr = h.GetSource();	//Packet的source节点地址
			packetsReceived++;
			//std::cout<<address<<" Received packet - "<<packetsReceived<<" from:"<<sourceAdr<<" and Size is "<<packet->GetSize ()<<" Bytes."<<std::endl;
			//吞吐量测试
			// std::cout<<"simRunTime now - "<<simRunTime<<std::endl;
			receivetime= Simulator::Now();
			//std::cout<<"package relay - "<<(receivetime-sendtime)<<std::endl;
			// std::cout<<"Throughput now - "<<packetsReceived*1024*8/simRunTime<<"bps"<<std::endl;
		}else{
			cout<<"接收器能量用光，停止仿真！："<<endl;
			// Simulator::Stop();
		}
    }
    
}


static void GenerateTraffic (Ipv4Header ipv4Header,Ptr<Packet> pkt,Ptr<Socket> socket,
                             Time pktInterval )
{ 
	if (CheckRemainingJ(socket->GetNode(), pkt)) {
		socket->Send (pkt);
		packetsSent++;//发包数
		if(packetsSent == maxPackets){
			socket->Close ();
			lifeTime = Simulator::Now().GetSeconds();
			Simulator::Stop (Seconds (0.0));
		}
		
		UpdateEnergySources(socket->GetNode(), pkt, 0,mobileSinkNode);//0代表发送，1代表接收
		Simulator::Schedule (pktInterval, &GenerateTraffic, ipv4Header,pkt,socket,  pktInterval);
		std::cout<<TIME_STAMP_FUC<<"Packet sent - "<<"FROM: "<<ipv4Header.GetSource()<<"---num is :"<<packetsSent<<std::endl;
	}else{
		cout<<ipv4Header.GetSource()<<"less of energy!!"<<endl;
		socket->Close ();
	}
}

/*
 * 创建节点
 */
void createNode(){
	c.Create(size);
	if(!isEntity){
		std::string traceFile = "scratch/"+mobilityModel+"/test.ns_movements";
		Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
		ns2.Install ();
	}

	for(uint32_t i=0;i<size-1;i++){
		nodes.Add(c.Get(i));
	}
	mobileSinkNode.Add(c.Get(size-1));
	NS_LOG_DEBUG("Create nodes done!");
}

void createMobilityModel(){
	MobilityHelper mobility;
	mobility.SetPositionAllocator("ns3::GridPositionAllocator",
			"GridWidth",UintegerValue(5),
			"MinX", DoubleValue(0.0),
			"MinY",DoubleValue(0.0),
			"DeltaX", DoubleValue(150.0),
			"DeltaY",DoubleValue(150.0));
	if(isStatic){
		mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	}else{
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
	// mobility.Install(c);

	if(isEntity){
		mobility.Install(nodes);
		mobility.Install(mobileSinkNode);
	}

}
//创建网络设备
void createWifiDevice(){
	//协议栈部分
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
	cDevices = wifi.Install (wifiPhy, wifiMac, c);
/** install PHY + MAC **/
	// senseDevices = wifi.Install(wifiPhy, wifiMac, senseNodes);
	// mobileSinkDevice = wifi.Install(wifiPhy, wifiMac, mobileSinkNode);
	NS_LOG_DEBUG("Create devices done!");
}
//安装路由协议
void installInternetStack(){
	//使用AODV路由协议
	AodvHelper aodv;
	InternetStackHelper stack1;
	stack1.SetRoutingHelper (aodv);
	stack1.Install (c);//注意，AODV协议安装到nodes节点上

	Ipv4AddressHelper address;
	address.SetBase ("10.0.0.0", "255.0.0.0");
	cIfs = address.Assign (cDevices);

	Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
}

/*
 *设置socket广播回调
 */
void createSocketCallBack(){
	Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (size-1), tid);
	InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 8080);
	recvSink->Bind (local);
	recvSink->SetRecvCallback (MakeCallback (&RecvPacketCallback));

	InetSocketAddress remote = InetSocketAddress (cIfs.GetAddress (size-1,0), 8080);
	  

	//改成把IP地址也发送出去
	uint32_t t = 0;
	double interval = 1.0;
	packetsSent+=10;
	for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End();
		i++) {
		// cout<<"--------------------->"<<nodes.GetN()<<endl;
		
		Ptr<Node> thisNode = *i;
		// Ipv4Address sourceAdr = GetNodeIpv4Address(thisNode);
		Ipv4Address source =GetNodeIpv4Address(thisNode);
		stringstream ss;
		ss<<source;//将源地址打包发送出去
		stringstream pktContents(ss.str().c_str());//
		Ptr<Packet> pkt = Create<Packet>((uint8_t *) pktContents.str().c_str(),
				pktSize);
		Ipv4Header ipv4Header;
		ipv4Header.SetSource(source);
		pkt->AddHeader(ipv4Header);
		//所有的源节点都将数据发送到接收器
		Ptr<Socket> socket = Socket::CreateSocket (nodes.Get (t++), tid);
		socket->Connect (remote);
		Simulator::Schedule (Seconds (interval), &GenerateTraffic, ipv4Header,pkt,socket, interPacketInterval);
		interval += 3;
	}
}

//生成XML文件
void createXML(){
	if(isEntity){
		Panim = new AnimationInterface("/home/wsn/sim_temp/AODV/output.xml");
	}else{
		Panim = new AnimationInterface("/home/wsn/sim_temp/GROUPAODV/"+mobilityModel+"/output.xml");
	}
	
	Panim->UpdateNodeColor(mobileSinkNode.Get(0), 0, 255, 0);//接收器颜色设置成绿色。

	Panim->UpdateNodeSize(mobileSinkNode.Get(0)->GetId(), 10.0, 10.0);
	for (NodeContainer::Iterator i = nodes.Begin();
			i != nodes.End(); i++) {
		Ptr<Node> n = *i;
		Panim->UpdateNodeSize(n->GetId(), 2.0, 2.0);
	}
}

//仿真结束以后的统计
void finalRecord(){
	//仿真结束以后，会调用如上方法，所以，如下会最后输出。
	std::cout<<"\n\n***** OUTPUT *****\n\n";
	std::stringstream ss;
	if(isLifeCycle){
		if(isEntity){
			ss << "/home/wsn/sim_temp/AODV/"<<mobilityModel<<"/LIFE/"<<initialJ<<"-entity.record";
		}else{
			ss << "/home/wsn/sim_temp/GROUPAODV/"<<mobilityModel<<"/"<<initialJ<<"-"+mobilityModel+".record";
		}
	}else{
		// if(isEntity){
			ss << "/home/wsn/sim_temp/AODV/"<<mobilityModel<<"/"<<maxPackets<<"-entity.record";
		// }else{
		// 	ss << "/home/wsn/sim_temp/GROUPAODV/"<<mobilityModel<<"/"<<maxPackets<<"-"+mobilityModel+".record";
		// }
	}
	
	std::ofstream of(ss.str().c_str());
	simFinishRealTime = clock();
	// if(mobilityModel.length()==0){
	// 	cout<<"使用的移动模型:RWP"<<endl;
	// 	ss << std::endl << "使用的移动模型:" << "RWP"<< std::endl;
	// }else{
	cout<<"使用的移动模型:"<<mobilityModel<<endl;
	ss << std::endl << "使用的移动模型:" << mobilityModel<< std::endl;	
	// }
	//如果测量网络生存周期,记录如下值
	if(isLifeCycle){
		cout<<"初始能量:"<<initialJ<<endl;
		cout<<"网络生存期:"<<lifeTime<<"(s)"<<endl;
		ss << std::endl << "初始能量:" << initialJ<<"(J)"<< std::endl;
		ss << std::endl << "网络生命周期:" << lifeTime<<"(s)"<< std::endl;
	}else{
		std::cout << "网络时延: " << netDelay<< "\n";
		std::cout<<"PDR指标:"<<(float)(packetsReceived*100/packetsSent)<<" %"<<std::endl;
		std::cout<<"网络吞吐量:"<<(float)(packetsReceived*1500)/((simFinishRealTime-simStartRealTime)/1000)<<"(byte/s)"<<std::endl;
		ss << std::endl << "网络时延:" << netDelay<<"(ms)"<< std::endl;
		ss << std::endl << "网络吞吐量: " << packetsReceived*1500*1.0/((simFinishRealTime-simStartRealTime)/1000)<<"(Byte/s)" << std::endl;
		ss << std::endl << "PDR指标:" <<packetsReceived*100.0/packetsSent<<" %"<< std::endl;
	}

	// ss << "Total comsumed energy = " << totalConsumed <<std::endl;

	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();
}
//主函数入口
int main(int argc, char **argv)
{

	simStartRealTime = clock();
	//解析命令行输入参数
	CommandLine cmd;
	cmd.AddValue("initialJ","Initial energy of each BaiscEnergySource", initialJ);
	cmd.AddValue("size","Initial energy of each BaiscEnergySource", size);
	cmd.AddValue("isStatic","is static", isStatic);
	cmd.AddValue("maxPackets","Initial energy of each BaiscEnergySource", maxPackets);
	cmd.AddValue("maxDis","max distance", maxDis);
	cmd.AddValue("mobilityModel","use mobilityModel", mobilityModel);
	cmd.AddValue("isLifeCycle","is test lifeCycle", isLifeCycle);
	cmd.AddValue("isEntity","is entity", isEntity);
	

	cmd.Parse(argc, argv);


	LogComponentEnable("Energy", LOG_LEVEL_DEBUG);
 	LogComponentEnable("aodvScript", LOG_LEVEL_DEBUG);
    
  //测试实体移动模型
    createNode();
	createMobilityModel();
	InstallEnergy(nodes);

	cout<<"install energy done!"<<endl;
	// createMobilityModel();

	createXML();
	createWifiDevice();
	installInternetStack();

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.InstallAll();

	createSocketCallBack();
	
	//Simulator::Stop (Seconds (totalTime));
	Simulator::Run ();
	Simulator::Destroy ();
	

	// std::cout<<"网络生命周期 = "<<Simulator::Now().GetSeconds()<<std::endl;
	
	// EnergyFinalRecord(nodes, remaingJ);

	
	Time delay;
	uint32_t num;
	flowMonitor->CheckForLostPackets ();
    FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {	
		  num++;
		  delay=delay+i->second.delaySum;
    }
	netDelay = delay/num;
	finalRecord();
}
