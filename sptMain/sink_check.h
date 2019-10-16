#ifndef SINK_CHECK_H
#define SINK_CHECK_H

#include "ns3/core-module.h"
#include <iomanip>
#include <string>
#include "ns3/internet-module.h"
#include "ns3/animation-interface.h"
#include "energy.h"
#include "tool.h"

#include <fstream>


using namespace std;
using namespace ns3;
namespace ns3 {
//	int32_t ProcessSinkCheckPacket(Ptr<Node> thisNode,
//		Ptr<Packet> packet, Ipv4Header h,NodeContainer sinkNodes,
//		NodeContainer mobileSinkNode,NodeContainer senseNode);
	void SetSinkCheckFlag(bool flag);
	void SortSinkNodesByAdr(NodeContainer sinkNodes);
	void MobileSinkRequestSinkCheck(NodeContainer mobileSinkNode);
	void PrepareRequestSinkCheck(NodeContainer mobileSinkNode);
	void ReplySinkCheck(Ptr<Node> sinkNode, Vector location);
//	int32_t ProcessSinkCheckPacket(Ptr<Node> thisNode,Ptr<Packet> packet,Ipv4Header h,NodeContainer senseNodes,NodeContainer sinkNodes,NodeContainer mobileSinkNode);
}
#endif
