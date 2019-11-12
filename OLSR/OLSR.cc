// #include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/olsr-header.h"
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

// #define maxPackets 900
/*
 */
using namespace ns3;
using namespace olsr;
using namespace std;

NS_LOG_COMPONENT_DEFINE("olsrScript");
// #define TIME_STAMP Simulator::Now().GetSeconds()<<": "//当前时间

Time netDelay;
clock_t lifeTime;
string mobilityModel="";
bool isLifeCycle=false;
string moveSpeed="";
bool isSpeed=false;

double dataInterval = 8.0; //发包间隔
int packetsSent = 0;
int packetsReceived = 0;
int maxPackets = 900;

static double Prss = -80; //dBm
static double offset = 81;  //I don't know
static std::string phyMode("DsssRate11Mbps");

//统计相关
static double simRunTime=0.0;
clock_t simStartRealTime;
clock_t simFinishRealTime;
// clock_t simStartRealTime;

//统计时延
static Time receivetime;
static Time sendtime;

NodeContainer c;
NodeContainer nodes; //实体测试
NodeContainer mobileSinkNode;
// NodeContainer nodes1;//群体测试

//Net device container
NetDeviceContainer cDevices;
Ipv4InterfaceContainer cIfs;


//全局变量声明
uint32_t size=21;
double step=100;
double totalTime=9999.0;;

int pktSize = 1500;
// int totalPackets = totalTime-1;
double interval = 5.0;
Time interPacketInterval = Seconds (interval);

//最大通信距离
double maxDis = 225;//15
//能量模型相关
double TxPara=0.000006;//0.000006J/Bytes for Tx
double RxPara=0.000006;//0.000006J/Bytes for Rx
double initialJ = 5.0;
uint32_t nDrain=0;
double Tgather=0.65;
double totalConsumed=0;

AnimationInterface *Panim;
bool Banim=true;

//Energy source pointer
// EnergySourceContainer *senseSources=0;
double remaingJ[MAX_NUM];

bool firstDrainFlag=true;
uint32_t firstDrainedNodeId=0;
double firstDrainedNodeTime=0;
bool isStatic  = false;

//仿真文件
string thisSimPath = "/home/wsn/sim_temp/OLSR/";
string exeName = "OLSR";


TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

void DataToSink();

/*考虑用这个回调机制计算吞吐量，现在每隔1S，进入此函数计算一次,本例子中只有最后一个节点安装了此回调*/
void RecvPacketCallback (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  simRunTime = Simulator::Now().GetSeconds();
  while ((packet = socket->Recv ()))
    {
		//测试接收到的包的大小
		//uint32_t pSize=packet->GetSize();
		//cout<<"接收到的数据包的大小为："<<pSize<<endl;

		Ptr<Node> thisNode = socket->GetNode();
        // Ipv4Address address = GetNodeIpv4Address(thisNode);
		// cout<<"接收数据包的ip地址是:"<<address;
		// MessageHeader headler;
		// int cont = (uint32_t)headler.GetHopCount();
		// cout<<"    跳数是:"<<cont<<endl;

		if (CheckRemainingJ(thisNode, packet)) {
			UpdateEnergySources(thisNode, packet, 1,mobileSinkNode);
			Ipv4Header h;
			packet->PeekHeader(h);
			// Ipv4Address sourceAdr = h.GetSource();	//Packet的source节点地址
			packetsReceived++;
			// std::cout<<address<<" Received packet - "<<packetsReceived<<" from:"<<sourceAdr<<" and Size is "<<packet->GetSize ()<<" Bytes."<<std::endl;
			//吞吐量测试
			// std::cout<<"simRunTime now - "<<simRunTime<<std::endl;
			receivetime= Simulator::Now();
			// std::cout<<"package relay - "<<(receivetime-sendtime)<<std::endl;
			// std::cout<<"Throughput now - "<<packetsReceived*1024*8/simRunTime<<"bps"<<std::endl;
		}else{
			// cout<<"接收器能量用光，停止仿真！："<<endl;
			//cout<<packetsReceived<<endl;
			//Simulator::Stop();
		}
    }
    
}



static void GenerateTraffic (Ptr<Packet> pkt,Ptr<Socket> socket)
{ 
	//这里加入能量消耗，判断能量是否够用，够用以后，发包。
	if (CheckRemainingJ(socket->GetNode(), pkt)) {
		socket->Send (pkt);
		UpdateEnergySources(socket->GetNode(), pkt, 0,mobileSinkNode);//0代表发送，1代表接收
	}else{
		socket->Close ();
	}
}

/*
 * 创建节点
 */
void createNode(){
	c.Create(size);

	std::string traceFile;
	if(moveSpeed.length()>0){
		traceFile = "scratch/"+mobilityModel+"/test"+moveSpeed+".ns_movements";
	}else{
		traceFile = "scratch/"+mobilityModel+"/speed30.ns_movements";
	}
	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install ();

	for(uint32_t i=0;i<size;i++){
		nodes.Add(c.Get(i));
	}
	//mobileSinkNode.Add(c.Get(size-1));
	mobileSinkNode.Create(1);
	NS_LOG_DEBUG("Create nodes done!");
}

void createMobilityModel(){
	MobilityHelper mobility;
	// mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
	// 							"Bounds", RectangleValue (Rectangle (-50, 50, -25, 50)));

	mobility.SetPositionAllocator("ns3::GridPositionAllocator",
			"GridWidth",UintegerValue(5),
			"MinX", DoubleValue(0.0),
			"MinY",DoubleValue(0.0),
			// "DeltaX", DoubleValue(10.0),
			// "DeltaY",DoubleValue(10.0));
			"DeltaX", DoubleValue(150.0),
			"DeltaY",DoubleValue(150.0));

	if(isStatic){
		mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	}else{
		ObjectFactory pos1;
		pos1.SetTypeId ("ns3::RandomRectanglePositionAllocator");
		pos1.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
		pos1.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
		Ptr<PositionAllocator> taPositionAlloc1 = pos1.Create()->GetObject<PositionAllocator> ();

		mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
										//"Speed", StringValue ("ns3::UniformRandomVariable[Min=0|Max=25"),
										"Speed", StringValue ("ns3::UniformRandomVariable[Min=1|Max=10]"),//2 6 10 14 18 22 26 30 34 38
										"Pause", StringValue ("ns3::ConstantRandomVariable[Constant=20.0]"),
										"PositionAllocator", PointerValue (taPositionAlloc1)
										);
	}
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(mobileSinkNode);
	mobileSinkNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));

}
//创建网络设备
void createWifiDevice(){
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
	cDevices = wifi.Install (wifiPhy, wifiMac, c);
/** install PHY + MAC **/
	// senseDevices = wifi.Install(wifiPhy, wifiMac, senseNodes);
	// mobileSinkDevice = wifi.Install(wifiPhy, wifiMac, mobileSinkNode);
	NS_LOG_DEBUG("Create devices done!");
}
//安装路由协议
void installInternetStack(){

	// OLSR  路由协议安装
	InternetStackHelper internet = InternetStackHelper();
	Ipv4ListRoutingHelper list_routing = Ipv4ListRoutingHelper();
	OlsrHelper olsr_routing = OlsrHelper();
	Ipv4StaticRoutingHelper static_routing = Ipv4StaticRoutingHelper();
	list_routing.Add(static_routing, 0);
	list_routing.Add(olsr_routing, 10);
	internet.SetRoutingHelper(list_routing);
	internet.Install(c);

	Ipv4AddressHelper address;
	address.SetBase ("10.0.0.0", "255.0.0.0");
	cIfs = address.Assign (cDevices);


	Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
}




void createSocketCallBack(){
	Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (size-1), tid);
	InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 8080);
	recvSink->Bind (local);
	recvSink->SetRecvCallback (MakeCallback (&RecvPacketCallback));
}

void DataToSink(){
	InetSocketAddress remote = InetSocketAddress (cIfs.GetAddress (size-1,0), 8080);
	//改成把IP地址也发送出去
	uint32_t t = 0;
	double interval = 0.0;
	for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End();i++) {
		Ptr<Node> thisNode = *i;
		Ipv4Address source =GetNodeIpv4Address(thisNode);
		Ptr<Packet> pkt = Create<Packet>(pktSize);//修改为创建这么大字节的包。
		Ipv4Header ipv4Header;
		ipv4Header.SetSource(source);
		pkt->AddHeader(ipv4Header);
		//所有的源节点都将数据发送到接收器
		Ptr<Socket> socket = Socket::CreateSocket (nodes.Get (t++), tid);
		socket->Connect (remote);
		std::cout<<TIME_STAMP_FUC<<"Packet sent - "<<"FROM: "<<ipv4Header.GetSource()<<"---num is :"<<packetsSent<<std::endl;
		Simulator::Schedule (Seconds (interval), &GenerateTraffic, pkt,socket);
		packetsSent++;
		interval += 3;
	}
	if(packetsSent == maxPackets){
		lifeTime = Simulator::Now().GetSeconds();
		Simulator::Stop(Seconds(0.0));
	}
	Simulator::Schedule(Seconds(dataInterval), &DataToSink);
}

//生成XML文件
void createXML(){
	if(isSpeed){
		if(isLifeCycle){
			Panim = new AnimationInterface("/home/wsn/sim_temp/OLSR/"+mobilityModel+"/SPEED/LIFE/output.xml");
		}else{
			Panim = new AnimationInterface("/home/wsn/sim_temp/OLSR/"+mobilityModel+"/SPEED/output.xml");
		}
	}else{
		if(isLifeCycle){
			Panim = new AnimationInterface("/home/wsn/sim_temp/OLSR/"+mobilityModel+"/PACKAGE/LIFE/output.xml");
		}else{
			Panim = new AnimationInterface("/home/wsn/sim_temp/OLSR/"+mobilityModel+"/PACKAGE/output.xml");
		}
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


	std::cout<<"\n\n***** OUTPUT *****\n\n";
	std::stringstream ss;
	if(isLifeCycle){
		if(isSpeed){
			ss << "/home/wsn/sim_temp/OLSR/"<<mobilityModel<<"/SPEED/LIFE/"<<moveSpeed<<".record";
		}else{
			ss << "/home/wsn/sim_temp/OLSR/"<<mobilityModel<<"/PACKAGE/"<<"LIFE"<<"/"<<initialJ<<"-"+mobilityModel+".record";
		}
	}else{
		if(isSpeed){
			ss << "/home/wsn/sim_temp/OLSR/"<<mobilityModel<<"/SPEED/"<<moveSpeed<<"-"+mobilityModel+".record";
		}else{
			ss << "/home/wsn/sim_temp/OLSR/"<<mobilityModel<<"/PACKAGE/"<<maxPackets<<"-"+mobilityModel+".record";
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
		std::cout<<"PDR指标:"<<(float)(packetsReceived*100/packetsSent)<<"%"<<endl;
		std::cout<<"网络吞吐量:"<<packetsReceived*1500*1.0/((simFinishRealTime-simStartRealTime)/1000)<<"(Byte/s)"<<endl;
		ss << std::endl << "网络时延:" << netDelay<<"(ms)"<< std::endl;
		ss << std::endl << "网络吞吐量: " << packetsReceived*1500*1.0/((simFinishRealTime-simStartRealTime)/1000)<<"(Byte/s)"<<endl;
		ss << std::endl << "PDR指标:" <<(float)(packetsReceived*100/packetsSent)<<"%"<<endl;
	}

	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();

}
//主函数入口
int main(int argc, char **argv)
{
	//解析命令行输入参数
	CommandLine cmd;
	cmd.AddValue("isSpeed","is Speed", isSpeed);
	cmd.AddValue("moveSpeed","move Speed", moveSpeed);
	cmd.AddValue("mobilityModel","use mobilityModel", mobilityModel);
	cmd.AddValue("isLifeCycle","is test lifeCycle", isLifeCycle);
	
	cmd.AddValue("pktSize","Size of data packe",pktSize);
	cmd.AddValue("initialJ","Initial energy of each BaiscEnergySource", initialJ);
	cmd.AddValue("size","Initial energy of each BaiscEnergySource", size);
	cmd.AddValue("maxPackets","maxPackets to send", maxPackets);
	cmd.AddValue("isStatic","is static", isStatic);
	cmd.AddValue("maxDis","max distance", maxDis);
	cmd.Parse(argc, argv);

	LogComponentEnable("Energy", LOG_LEVEL_DEBUG);
 	LogComponentEnable("olsrScript", LOG_LEVEL_DEBUG);
    simStartRealTime = clock();

  //测试实体移动模型
    createNode();

	//已经在createNode中设置接收器移动
    createMobilityModel();
	createWifiDevice();
	installInternetStack();
	createSocketCallBack();
	createXML();
	InstallEnergy(nodes);

	DataToSink();
	// cout<<"install energy done!"<<endl;
	Ptr<FlowMonitor> flowMonitor;    
	FlowMonitorHelper flowHelper;    
	flowMonitor = flowHelper.InstallAll();

	



	Simulator::Stop (Seconds (totalTime));
	Simulator::Run ();

	// double delay = 0; 
	Time delay;    
	uint32_t num;    
	flowMonitor->CheckForLostPackets ();   
	FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();    
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {    num++;                        
		delay=delay+i->second.delaySum;    //.GetSeconds()
		num++;
	}


	// double delay = 0;    
	// uint32_t num;    
	// flowMonitor->CheckForLostPackets ();   
	// Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper.GetClassifier ());
	// FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();    
	// for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {    num++;                        
	// 	Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
	// 	if (t.destinationAddress == "10.0.0.21"){
	// 		if (i->second.rxPackets != 0){
	// 			delay=delay+i->second.delaySum.GetSeconds();    //
	// 			num++;
	// 		}
	// 	}
	// }
	cout<<"网络时延:"<<delay/num<<endl;//单跳的平均时延  4.19ms

	netDelay = delay/num;

	Simulator::Destroy ();
	finalRecord();

	// simFinishRealTime = clock();
	// EnergyFinalRecord(nodes, remaingJ);

}
