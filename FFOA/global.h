 #ifndef GLOBAL_H
#define GLOBAL_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include <fstream>

namespace ns3 {
	#define TIME_STAMP Simulator::Now().GetSeconds()<<": "
	#define TIME_STAMP_FUC Simulator::Now().GetSeconds()<<": ("<<__FUNCTION__<<") "
	#define FUC __FUNCTION__<<": "
	#define MAX_NUM 255

//	TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
//	InetSocketAddress broadcastAdr = InetSocketAddress(
//		Ipv4Address::GetBroadcast(), 80);
	/*
	 * 定义Node类型的枚�?
	 */
	enum nodeType {
		sense_Type = 0,
		sink_Type = 1,
		mobileSink_Type = 2,
		relay_Type = 3,
	} ;

	/*
	 * 定义Packet类型（ipv4Header的Identification）的枚举
	 */
	enum pktType {
	//	data_Type = 0, spt_Type = 1, sinkCheck_Type = 2, dataGathering_Type = 3,//Ipv4Header的identification默�?�情�?=0
		data_Type = 1, spt_Type = 2, sinkCheck_Type = 3, 
		dataGathering_Type = 4,drainNotice_Type=5,relayCheck_Type = 6,neighbor_Type = 7,neighborRelay_Type = 8,
	};


	enum nodeType isSinkType(Ptr<Node> node, Ptr<Node> mobilenode);
	int32_t AnalyzeSinkCheckPacket(std::stringstream &ss, Vector &location);
	// enum nodeType CheckNodeType(Ptr<Node> node,NodeContainer senseNodes,Ptr<Node> mobilenode);
	enum nodeType CheckNodeType(Ptr<Node> node,NodeContainer senseNodes,NodeContainer relayNodes,Ptr<Node> mobilenode);
	// Ptr<Node> GetNodePtrFromGlobalId(uint32_t id, NodeContainer n);
	Ptr<Node> GetNodePtrFromGlobalId(uint32_t id, NodeContainer n,NodeContainer mobileSinkNode);
	
	double RandomDoubleVauleGenerator(double min, double max);
	Ipv4Address GetNodeIpv4Address(Ptr<Node> n);
	Ptr<Node> GetNodePtrFromIpv4Adr(Ipv4Address adr,NodeContainer senseNodes,NodeContainer relayNodes,NodeContainer mobileSinkNode);
	void FindInrangeNodes(NodeContainer allNodes);
	inline double GetDistanceOf2Nodes(Ptr<Node> srcN, Ptr<Node> remN);

}

#endif
