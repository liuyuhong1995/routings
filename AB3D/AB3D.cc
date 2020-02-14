#include "ns3/aodv-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "tool.h"
#include "global.h"
#include "energy.h"
#include "packet_manage.h"

using namespace ns3;
using namespace std;
static TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
static InetSocketAddress broadcastAdr = InetSocketAddress(Ipv4Address::GetBroadcast(), 80);
string moveSpeed="";
string mobilityModel="";
int inputSpeed = 0;
int maxPackets = 1500;

nodestate* NS; //节点状态
NodeContainer BS;
NodeContainer unodes;
/*路由表*/
map<uint32_t,vector<neighbor>> loop_map; 
//
int RPN; //总接收packet数量
int TPN; //总发送packet数量
int* flag; //标志位
double initialJ;
bool isTest=false;

//==================================================================
//创建节点并初始化节点状态
//==================================================================
void createnode(uint32_t UAVnum){
	BS.Create(1);
	unodes.Create(UAVnum);
	NS=new nodestate[UAVnum];
	flag=new int[UAVnum];
	for(uint32_t i=0;i<UAVnum;i++){
		NS[i].isdead=false;
		NS[i].remainingJ=initialJ;
		flag[i]=0;
	}
}
//==================================================================
//创建移动模型安装到节点上
//==================================================================
void createmobilitymodel(){
//3D-Randomwaypoint mobilitymodel
	stringstream input;
	input<<"ns3::UniformRandomVariable[Min="<<inputSpeed-2<<"|Max="<<inputSpeed+2<<"]";
	MobilityHelper mobility;  
	Ptr<RandomBoxPositionAllocator>RBOXPA=CreateObject<RandomBoxPositionAllocator>();
	RBOXPA->SetAttribute("X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"));
	RBOXPA->SetAttribute("Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"));
	RBOXPA->SetAttribute("Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=300]"));
	mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
		"Speed",  StringValue (input.str()),//速度设置
		"Pause", StringValue ("ns3::UniformRandomVariable[Min=0.3|Max=2]"),
		"PositionAllocator", PointerValue (RBOXPA));

	mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator", "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"),
																	 "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"),
																	 "Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=300]"));
    mobility.Install(unodes);
//基站移动模型
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(BS);	
	Ptr<MobilityModel> mm= BS.Get(0)->GetObject<MobilityModel>();
	Vector BSposition{500,500,150};
	mm->SetPosition(BSposition);
}


int main(int argc, char **argv)
{
  
  	uint32_t UAVnum=10;
  	double totalTime=9999.9; 
  	double circletime=12.0;
	double DTtime=8.0; 
	double maxDis=30;
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of Nodes", UAVnum);
	// cmd.AddValue("totalTime", "Simulation time length", totalTime);
	cmd.AddValue("maxDis", "max distance to transmit", maxDis);
	cmd.AddValue("initialJ","Initial energy of each BaiscEnergySource", initialJ);
	cmd.AddValue("isTest", "open Test Point or not", isTest);
	cmd.AddValue("maxPackets", "max distance to transmit", maxPackets);
	cmd.AddValue("mobilityModel","use mobilityModel", mobilityModel);
	cmd.AddValue("moveSpeed","move Speed", moveSpeed);
	cmd.AddValue("inputSpeed","input speed",inputSpeed);
	cmd.Parse(argc, argv);
	RPN=0;
	TPN=0;
	
	createnode(UAVnum);
	if(isTest){
		cout<<"node created!"<<endl;
		for(uint32_t i=0;i<UAVnum;i++){
			cout<<"node"<<i<<"  isdead:"<<NS[i].isdead<<"   energy:"<<NS[i].remainingJ<<endl;
		}
	}
	NetDeviceContainer BSdevice;  
	NetDeviceContainer Udevices;

	Ipv4InterfaceContainer BSIf;	
	Ipv4InterfaceContainer UIfs; 
	
/// disable fragmentation for frames below 2200 bytes
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
			StringValue("2200"));
/// turn off RTS/CTS for frames below 2200 bytes
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
			StringValue("2200"));
/// Fix non-unicast data rate to be the same as that of unicast
	std::string phyMode("DsssRate11Mbps");
	Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
			StringValue(phyMode));
	// Config::SetDefault("ns3::WifiRemoteStationManager::DefaultTxPowerLevel",
	//         StringValue("1"));
	// Config::SetDefault ("ns3::ArpCache::PendingQueueSize", UintegerValue (300));

	createmobilitymodel();
	if(isTest){
		cout<<"node position:  "<<endl;
		for(NodeContainer::Iterator i=unodes.Begin();i!=unodes.End();i++){
			Ptr<MobilityModel> mm = (*i)->GetObject<MobilityModel>();
			Vector posit = mm->GetPosition();
			cout<<"node"<<(*i)->GetId()<<" ("<<posit.x<<","<<posit.y<<","<<posit.z<<")"<<endl;
		}
	}
//Create Udevices
	WifiHelper wifi;
	wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
	/** Wifi PHY **/
	/***************************************************************************/
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	wifiPhy.Set("RxGain", DoubleValue(0.0));
	double Prss = -80; //dBm
	double offset = 80;  //I don't know
	wifiPhy.Set("TxGain", DoubleValue(offset + Prss));
	// wifiPhy.Set("CcaMode1Threshold", DoubleValue(0.0));
    wifiPhy.Set("TxPowerStart",DoubleValue(-49));
    wifiPhy.Set("TxPowerEnd",DoubleValue(-49));
	// wifiPhy.Set("TxPowerLevels",UintegerValue(2));
	wifiPhy.Set("Frequency",UintegerValue(2400));
	wifiPhy.Set("EnergyDetectionThreshold", DoubleValue(-90));

	/***************************************************************************/
	/** wifi channel **/
	YansWifiChannelHelper wifiChannel;
	wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	//固定范围
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",DoubleValue(maxDis));
	//空间衰减
	// wifiChannel.AddPropagationLoss("ns3::RandomPropagationLossModel",
	//                                "Variable",StringValue("ns3::UniformRandomVariable[Min=0.0|Max=5.0]"));
	// wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent",
	// 		DoubleValue(2.5),"ReferenceDistance",DoubleValue(1),"ReferenceLoss",DoubleValue(4.0));
// create wifi channel
	Ptr<YansWifiChannel> wifiChannelPtr = wifiChannel.Create();
	wifiPhy.SetChannel(wifiChannelPtr);
	/** MAC layer **/
// Add a non-QoS upper MAC, and disable rate control
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
	//WifiMacHelper wifiMac;	
	//wifiMac.SetType("ns3::AdhocWifiMac");
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
			StringValue(phyMode), "ControlMode", StringValue(phyMode));
/// Set it to ad-hoc mode
	wifiMac.SetType("ns3::AdhocWifiMac");
/** install PHY + MAC **/
	Udevices = wifi.Install(wifiPhy, wifiMac, unodes);
	BSdevice = wifi.Install(wifiPhy, wifiMac, BS);

/**Install Internet Stack**/
    NodeContainer ALLnode;
	ALLnode.Add(BS);
	ALLnode.Add(unodes);
	InternetStackHelper stack;
	stack.Install(ALLnode);
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	UIfs = ipv4.Assign(Udevices);
	BSIf= ipv4.Assign(BSdevice);
	if(isTest){
		for(NodeContainer::Iterator i=unodes.Begin();i!=unodes.End();i++){
			cout<<"node"<<(*i)->GetId()<<": "<<GetNodeIpv4Address((*i))<<endl;
		}
	}
//Set recvSocket and callback function
	std::vector<Ptr<Socket> > recvSocket(UAVnum+1);
	recvSocket[0] = Socket::CreateSocket(BS.Get(0),tid);
	recvSocket[0]->Bind(InetSocketAddress(BSIf.GetAddress(0), 80));
	recvSocket[0]->SetRecvCallback(MakeCallback(&BS_RecvPacketCallback));
	
	for (uint32_t i = 1; i < UAVnum+1; i++) {
		recvSocket[i] = Socket::CreateSocket(unodes.Get(i-1), tid);
		recvSocket[i]->Bind(InetSocketAddress(UIfs.GetAddress(i-1), 80));
		recvSocket[i]->SetRecvCallback(MakeCallback(&RecvPacketCallback));
	}


	Ptr<FlowMonitor> flowMonitor;    
	FlowMonitorHelper flowHelper;    
	flowMonitor = flowHelper.InstallAll();


    
	for(double timenow=0;timenow<totalTime;timenow=timenow+circletime)
	{		   
			Simulator::Schedule(Seconds(timenow), &neighbordiscover,unodes,circletime);		   		   
	}

	for(double timenow=1;timenow<totalTime;timenow=timenow+DTtime)
	{		   
			Simulator::Schedule(Seconds(timenow), &data_trans,unodes,DTtime);		    		   
	}
  
  
  AnimationInterface anim ("/home/wsn/sim_temp/AB3D/output.xml");
 // Simulator::Schedule(Seconds(0), &printposition,unodes.Get(0));
	      
  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();


	Time delay;    
	uint32_t num;    
	flowMonitor->CheckForLostPackets ();   
	FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();    
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {                         
		delay=delay+i->second.delaySum;    
		num++;
	}
	std::stringstream ss;
	ss << "/home/wsn/sim_temp/AB3D/"<<mobilityModel<<"/"<<moveSpeed<<"-"<<maxDis<<"-"<<maxPackets<<".record";		
	std::ofstream of(ss.str().c_str());
	cout<<std::endl << "使用的移动模型:" << mobilityModel<< std::endl;
	ss << std::endl << "使用的移动模型:" << mobilityModel<< std::endl;	
	cout<<"网络时延:"<<delay/num<<endl;//单跳的平均时延
	cout<<"发送数据包总数： "<<TPN<<endl;
	cout<<"接收数据包总数： "<<RPN<<endl;
	cout<<"PDR:            "<<((RPN*1.0)/(TPN * 1.0))*100.0<<"%"<<endl;
	cout<<"网络吞吐量：     "<<RPN*1500.0*8.0/((Simulator::Now().GetSeconds()-0)*1000.0)<<"(kbps)"<<endl;	
	ss<<"网络时延:"<<delay/num<<endl;//单跳的平均时延
	ss<<"发送数据包总数： "<<TPN<<endl;
	ss<<"接收数据包总数： "<<RPN<<endl;
	ss<<"PDR:            "<<((RPN*1.0)/(TPN * 1.0))*100.0<<"%"<<endl;
	ss<<"网络吞吐量：     "<<RPN*8.0*1500.0/((Simulator::Now().GetSeconds()-0)*1000.0)<<"(kbps)"<<endl;
	of << ss.str().c_str();
	of.close();

  Simulator::Destroy ();
  
  std::cout<<"\n\n***** OUTPUT *****\n\n";
  std::cout<<"Finished !"<<std::endl;

  
}