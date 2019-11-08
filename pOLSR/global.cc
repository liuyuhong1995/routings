#include "global.h"
#include "ns3/node-container.h"
#include <iomanip>
#include "ns3/mobility-module.h"

using namespace std;

extern double maxDis;
extern string thisSimPath;//simulation path
extern string exeName;
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Global");

/*
 * Find the numbers of nodes within the range of some node's communication
 */
void FindInrangeNodes(NodeContainer allNodes) {
	uint32_t num = allNodes.GetN();
	NS_LOG_LOGIC(FUC<<"num="<<num);
	uint32_t inrangeStat[num] = { 0 };
	uint32_t index=0;
	Ptr<Node> srcN, remN;
	for (NodeContainer::Iterator i = allNodes.Begin(); i != allNodes.End();
			i++) {
		srcN = *i;
		for (NodeContainer::Iterator j = allNodes.Begin(); j != allNodes.End();
				j++) {
			remN = *j;
			if (GetDistanceOf2Nodes(srcN, remN) < maxDis) {
				inrangeStat[index]++;
			}
		}
		index++;
	}
	stringstream ss;
	ss << thisSimPath<<exeName<<"-inrange-nodes.stat";
	ofstream of(ss.str().c_str());
	ss << std::endl;
	ss << std::setw(20) << std::left << "Node ID(Global)";
	ss << std::setw(20) << std::left << "Num of Inrange Nodes" << std::endl;
	stringstream si;
	for (uint32_t i = 0; i < num; i++) {
		si << allNodes.Get(i)->GetId();
//		si << senseNodes.Get(i)->GetId();
		ss << std::setw(20) << std::left << si.str();
		si.str("");
		si.clear();

		si << --inrangeStat[i];
		ss << std::setw(20) << std::left << si.str() << std::endl;
		si.str("");
		si.clear();
	}
	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();
}
/**
 * 注意：这里传递的泛型参数
 */
inline double GetDistanceOf2Nodes(Ptr<Node> srcN, Ptr<Node> remN) {
	Ptr<ConstantPositionMobilityModel> srcCpmm = srcN->GetObject<
			ConstantPositionMobilityModel>();
	Ptr<ConstantPositionMobilityModel> remCpmm = remN->GetObject<
			ConstantPositionMobilityModel>();
	Vector srcP = srcCpmm->GetPosition();
	Vector remP = remCpmm->GetPosition();
	return std::sqrt((srcP.x - remP.x)*(srcP.x - remP.x) + (srcP.y - remP.y)*(srcP.y - remP.y));
}
/*
 * 解析SinkCheck包的内容
 */
int32_t AnalyzeSinkCheckPacket(std::stringstream &ss, Vector &location) {
	std::stringstream si;
	char c[ss.str().size()];
	ss >> c;
	char *p = c;

	//x
	while (*p != '/') {
		si << *p;
		p++;
	}
	si >> location.x;

	//y
	p++;
	si.str("");
	si.clear();
	while (*p != '/') {
		si << *p;
		p++;
	}
	si >> location.y;

	//z
	p++;
	si.str("");
	si.clear();
	while (*p != '/') {
		si << *p;
		p++;
	}
	si >> location.z;

	NS_LOG_LOGIC("AnalyzeSinkCheckPacket: location = "<<location);
	return 0;
}

enum nodeType CheckNodeType(Ptr<Node> node,NodeContainer senseNodes,Ptr<Node> mobilenode) {
	if(node->GetId()==mobilenode->GetId())
		return mobileSink_Type;

	// for (vector<uint32_t>::const_iterator iter = sinkVector.cbegin(); iter != sinkVector.cend(); iter++)
    // {
	// 	// cout<<"sink type!!!"<<endl;
	// 	if((*iter)==node->GetId())
    //      return sink_Type;
    // }
	
	// for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
	// 		i++) {
	// 	Ptr<Node> n = *i;
	// 	if (n->GetId() == node->GetId())
	// 		return sense_Type;
	// }

	// for (NodeContainer::Iterator i = relayNodes.Begin(); i != relayNodes.End();
	// 		i++) {
	// 	Ptr<Node> n = *i;
	// 	if (n->GetId() == node->GetId())
	// 		return relay_Type;
	// }

	return sense_Type;
}

/*
 * Get node ptr by global id
 */
Ptr<Node> GetNodePtrFromGlobalId(uint32_t id, NodeContainer n,NodeContainer mobileSinkNode) {
	for (uint32_t i = 0; i < n.GetN(); i++) {
		if (n.Get(i)->GetId() == id) {
			return n.Get(i);
		}
	}
	return mobileSinkNode.Get(0);
}

/*
 * Generate a random double value
 */
double RandomDoubleVauleGenerator(double min, double max) {

	Ptr<UniformRandomVariable> r = CreateObject<UniformRandomVariable>();
	r->SetAttribute("Min", DoubleValue(min));
	r->SetAttribute("Max", DoubleValue(max));
	return r->GetValue();
}

/*
 * 返回某个节点的Ipv4Address
 */
Ipv4Address GetNodeIpv4Address(Ptr<Node> n){
	return n->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();//1表示interface 的index，0表示address index
}

/*
 * 通Ipv4Address查找Node，返回Ptr
 */
// Ptr<Node> GetNodePtrFromIpv4Adr(Ipv4Address adr,NodeContainer sinkNodes,NodeContainer senseNodes,NodeContainer mobileSinkNode) {
// 	for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();
// 			i++) {
// 		Ptr<Node> n = *i;
// 		if (GetNodeIpv4Address(n) == adr) {
// 			NS_LOG_LOGIC(TIME_STAMP
// 					<<"GetNodePtrFromIpv4Adr: SinkNodes matched!");
// 			return n;
// 		}
// 	}

// 	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
// 			i++) {
// 		Ptr<Node> n = *i;
// 		if (GetNodeIpv4Address(n) == adr) {
// 			NS_LOG_LOGIC("GetNodePtrFromIpv4Adr: senseNodes matched!");
// 			return n;
// 		}
// 	}

// 	NS_LOG_LOGIC("GetNodePtrFromIpv4Adr: mobileSinkNode matched!");
// 	return mobileSinkNode.Get(0);
// }

Ptr<Node> GetNodePtrFromIpv4Adr(Ipv4Address adr,NodeContainer senseNodes,NodeContainer mobileSinkNode) {
	// for (NodeContainer::Iterator i = sinkNodes.Begin(); i != sinkNodes.End();
	// 		i++) {
	// 	Ptr<Node> n = *i;
	// 	if (GetNodeIpv4Address(n) == adr) {
	// 		NS_LOG_LOGIC(TIME_STAMP
	// 				<<"GetNodePtrFromIpv4Adr: SinkNodes matched!");
	// 		return n;
	// 	}
	// }

	for (NodeContainer::Iterator i = senseNodes.Begin(); i != senseNodes.End();
			i++) {
		Ptr<Node> n = *i;
		if (GetNodeIpv4Address(n) == adr) {
			NS_LOG_LOGIC("GetNodePtrFromIpv4Adr: senseNodes matched!");
			// cout<<"通过ip地址找到了,几点id是"<<n->GetId()<<endl;
			return n;
		}
	}

	NS_LOG_LOGIC("GetNodePtrFromIpv4Adr: mobileSinkNode matched!");
	return mobileSinkNode.Get(0);
}

}//namespace ns3
