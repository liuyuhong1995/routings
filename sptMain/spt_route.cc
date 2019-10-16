#include "spt_route.h"
#include <string>
#include <iomanip>
#include "ns3/mobility-module.h"
#include "tool.h"
#include "jump-count-table-chain.h"

using namespace std;
using namespace ns3;

//extern string thisSimPath;//simulation path
//extern string exeName;
////声明外部变量
//extern double totalConsumed;
//extern uint32_t nDrain;
//extern bool firstDrainFlag;
extern InetSocketAddress broadcastAdr;
extern TypeId tid;
extern bool Banim;
extern uint32_t nSenses;
extern AnimationInterface *Panim;
extern JumpCountTableChain jctChain;

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("SPT_ROUTE");

void RemoveFutileNodes(NodeContainer senseNodes,NetDeviceContainer senseDevices,NodeContainer sinkNodes,NodeContainer mobileSinkNode) {
	NS_LOG_LOGIC(std::endl<<TIME_STAMP_FUC<<"...");
	NodeContainer tempC;
	NetDeviceContainer tempDC;
	uint32_t tempNum = 0;
	Ptr<Node> n;
	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
			i++) {
		n = *i;
		if (jctChain.InquireNode(GetNodeIpv4Address(n))
				|| CheckNodeType(n,sinkNodes,mobileSinkNode.Get(0)) == sink_Type) {
			NS_LOG_LOGIC(
					FUC<<"NodeId = "<<n->GetId()<<", node type = "
					<<CheckNodeType(n,sinkNodes,mobileSinkNode.Get(0))<<" is in the JumpCountTableChain");
			tempC.Add(n);
			tempDC.Add(n->GetDevice(0));
			tempNum++;
		} else {
			NS_LOG_LOGIC(FUC<<"NodeId = "<<n->GetId()<<", node type = "
					<<CheckNodeType(n,sinkNodes,mobileSinkNode.Get(0))<<" is a futile node");
			if (Banim) {
				Panim->UpdateNodeColor(n, 255, 255, 255);
			}
		}
	}
	senseNodes = tempC;
	senseDevices = tempDC;
	nSenses=tempNum;
	NS_LOG_LOGIC(TIME_STAMP_FUC<<"Done");
}


void ContinueSptRouting(Ptr<Node> n, uint32_t jumps, Ipv4Address sinkAdr) {
	NS_LOG_LOGIC(
					endl<<TIME_STAMP_FUC
					<<"From sinkNode "
					<<sinkAdr<<" by "<<GetNodeIpv4Address(n));
	stringstream ss;
	pktType pktType=spt_Type;
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
	source = Ipv4Address(si.str().c_str());   //source不是输入参数吗？
	NS_LOG_LOGIC(FUC<<"source = "<<source<<", jumps = "<<jumps);
	return jumps;
}
/*
 * 开始从一个sinkNode n的SPT计算
 */
void StartSptRoutingFrom(Ptr<Node> n,NodeContainer sinkNodes,NodeContainer mobileSinkNode) {
	if (CheckNodeType(n,sinkNodes,mobileSinkNode.Get(0)) != sink_Type) {
		NS_LOG_ERROR(TIME_STAMP_FUC<<"ERROR: Wrong SPT starting node!");
		Simulator::Stop();
	}
	else {
		NS_LOG_INFO(TIME_STAMP_FUC<<"sinkNode "<<GetNodeIpv4Address(n));
		Ipv4Address sinkSource =GetNodeIpv4Address(n);
		stringstream ss;
		ss<<"0/"<<sinkSource;
		stringstream pktContents(ss.str().c_str());//路由广播从SinkNode发出来的时候，数据里面的跳数是0，/后面是sinkSource的Ipv4Address
		Ptr<Packet> pkt = Create<Packet>((uint8_t *) pktContents.str().c_str(),  //啥意思？（包的内容和大小）
				pktContents.str().length());
		Ipv4Header ipv4Header;
		pktType pktType=spt_Type;
		//	ipv4Header.SetDestination(senseIfs.GetAddress(0));
		ipv4Header.SetSource(sinkSource);
		ipv4Header.SetIdentification(pktType);
		pkt->AddHeader(ipv4Header);

		Ptr<Socket> source = Socket::CreateSocket(n, tid);  //参数为发送节点的指针和tid
		source->Connect(broadcastAdr);
		source->SetAllowBroadcast(true);	//socket发送广播必须有这么一个设置
		source->Send(pkt);
	}
}
/*
 * Prepare to start SPT routing
 */
void PrepareSptRouting(NodeContainer sinkNodes,NodeContainer mobileSinkNode) {

//	NodeContainer S;
//	NodeContainer M;
//	std::cout<<"join in PrepareSptRouting ============"<<endl;
	NS_LOG_LOGIC((Simulator::Now().GetSeconds()-1.0)<<": Sink check done");
	NS_LOG_LOGIC(endl<<TIME_STAMP_FUC<<"...");
	std::cout<<"进入spt_route.cc--PrepareSptRouting()->sink nums are:"<<sinkNodes.GetN()<<endl;

	double interval=0;
	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();
			i++) {
		interval=RandomDoubleVauleGenerator(0.1, 5.0);
		Ptr<Node> n = *i;
		NS_LOG_LOGIC(TIME_STAMP<<"Node["<<n->GetId()<<"], interval = "<<interval);
		cout<<"befor StartSptRoutingFrom======"<<endl;
		Simulator::Schedule(Seconds(interval), StartSptRoutingFrom, n, sinkNodes,mobileSinkNode);
	}
}



}
