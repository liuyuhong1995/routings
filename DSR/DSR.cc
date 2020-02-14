

#include "ns3/aodv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h" 
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

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
using namespace dsr;
using namespace std;

NS_LOG_COMPONENT_DEFINE("dsrScript");

// #define TIME_STAMP Simulator::Now().GetSeconds()<<": "//当前时间

Time netDelay;
clock_t lifeTime;
string mobilityModel="";
bool isLifeCycle=false;
string moveSpeed="";
bool isSpeed=false;

double dataInterval = 10.0; //发包间隔
int packetsSent = 0;
int packetsReceived = 0;
int maxPackets = 900;

static double Prss = -80; //dBm
static double offset = 81;  //I don't know
static std::string phyMode("DsssRate11Mbps");

//统计相关
static double simRunTime=0.0;
double simStartRealTime;
double simFinishRealTime;
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
int inputSpeed = 0;

//全局变量声明
uint32_t size=21;
double step=100;
double totalTime=9999.0;;

int pktSize = 1500;
// int totalPackets = totalTime-1;
double interval = 8.0;
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
string thisSimPath = "/home/wsn/sim_temp/DSR/";
string exeName = "DSR";


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
      //  Ipv4Address address = GetNodeIpv4Address(thisNode);
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
			 cout<<"接收器能量用光，停止仿真！："<<endl;
			cout<<packetsReceived<<endl;
			Simulator::Stop();
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
	nodes.Create(size);
	mobileSinkNode.Create(1);
	NS_LOG_DEBUG("Create nodes done!");
	c.Add(nodes);
	c.Add(mobileSinkNode);
}

/*
 * 创建移动模型并安装到节点
 */
void createMobilityModel(){
	stringstream input;
	input<<"ns3::UniformRandomVariable[Min="<<inputSpeed-2<<"|Max="<<inputSpeed+2<<"]";
	MobilityHelper mobility;  
	Ptr<RandomBoxPositionAllocator>RBOXPA=CreateObject<RandomBoxPositionAllocator>();
	RBOXPA->SetAttribute("X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"));
	RBOXPA->SetAttribute("Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"));
	RBOXPA->SetAttribute("Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=300]"));
	mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
		"Speed",  StringValue (input.str()),
		//"Speed",  StringValue ("ns3::UniformRandomVariable[Min=33|Max=37]"),//速度设置
		"Pause", StringValue ("ns3::UniformRandomVariable[Min=0.3|Max=2]"),
		"PositionAllocator", PointerValue (RBOXPA));
    
	mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator", "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"),
																"Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"),
																"Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=300]"));

	//mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes); 
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(mobileSinkNode);	
	Ptr<MobilityModel> mm= mobileSinkNode.Get(0)->GetObject<MobilityModel>();
	Vector BSposition{500,500,150};
	mm->SetPosition(BSposition);
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
	// DSR  路由协议安装	
	/*
	InternetStackHelper internet = InternetStackHelper();
	DsrMainHelper dsrMain;
	DsrHelper dsr;
	internet.Install (c);
	dsrMain.Install (dsr, c);   */ 
	AodvHelper aodv;
	InternetStackHelper stack1;
	stack1.SetRoutingHelper (aodv);
	stack1.Install (c);
	// internet.Install(c);
	Ipv4AddressHelper address;
	address.SetBase ("10.0.0.0", "255.0.0.0");
	//address.SetBase ("10.1.1.0", "255.255.255.0");
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
	packetsSent+=size;
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
	
		Panim = new AnimationInterface("/home/wsn/sim_temp/DSR/output.xml");
	
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
	ss << "/home/wsn/sim_temp/DSR/"<<mobilityModel<<"/"<<moveSpeed<<"-"<<maxDis<<"-"<<maxPackets<<".record";	
	std::ofstream of(ss.str().c_str());
	simFinishRealTime = Simulator::Now().GetSeconds();
	cout<<"使用的移动模型:"<<mobilityModel<<endl;
	ss << std::endl << "使用的移动模型:" << mobilityModel<< std::endl;	
	std::cout << "网络时延: " << netDelay<< "\n";
	std::cout<<"PDR指标:"<<(float)(packetsReceived*100/packetsSent)<<" %"<<std::endl;
	std::cout<<"网络吞吐量:"<<(float)(packetsReceived*1500*8)/((simFinishRealTime - simStartRealTime)*1000)<<"(kbps)"<<std::endl;
	ss << std::endl << "网络时延:" << netDelay<< std::endl;
	ss << std::endl << "网络吞吐量: " << (float)(packetsReceived*1500*8)/((simFinishRealTime - simStartRealTime)*1000)<<"(kbps)" << std::endl;
	cout<<Simulator::Now().GetSeconds()<<endl;
	ss << std::endl << "PDR指标:" <<packetsReceived*100.0/packetsSent<<" %"<< std::endl;
	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();
}


//主函数入口
int main(int argc, char **argv)
{

	simStartRealTime = Simulator::Now().GetSeconds();
	//解析命令行输入参数
	CommandLine cmd;
	cmd.AddValue("initialJ","Initial energy of each BaiscEnergySource", initialJ);
	cmd.AddValue("size","Initial energy of each BaiscEnergySource", size);
	cmd.AddValue("isStatic","is static", isStatic);
	cmd.AddValue("maxPackets","Initial energy of each BaiscEnergySource", maxPackets);
	cmd.AddValue("maxDis","max distance", maxDis);
	cmd.AddValue("mobilityModel","use mobilityModel", mobilityModel);
	cmd.AddValue("isLifeCycle","is test lifeCycle", isLifeCycle);
	cmd.AddValue("moveSpeed","move Speed", moveSpeed);
	cmd.AddValue("inputSpeed","input Speed", inputSpeed);
	cmd.Parse(argc, argv);
	

	LogComponentEnable("Energy", LOG_LEVEL_DEBUG);
 	LogComponentEnable("dsrScript", LOG_LEVEL_DEBUG);
    
  //测试实体移动模型
    createNode();
	createMobilityModel();
	InstallEnergy(nodes);
	cout<<"install energy done!"<<endl;
	
	createXML();
	createWifiDevice();
	installInternetStack();

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.InstallAll();

	createSocketCallBack();
	Simulator::Run ();
	
	
	Time delay;
	long num = 0L;
	flowMonitor->CheckForLostPackets ();
    FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {	
		  num++;
		  delay=delay+i->second.delaySum;
    }
	netDelay = delay/num;
	finalRecord();
	Simulator::Destroy ();
}