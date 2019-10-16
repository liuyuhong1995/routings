#ifndef SPT_ROUTE_H
#define SPT_ROUTE_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/animation-interface.h"
#include "tool.h"
#include <string>
#include <fstream>

using namespace std;

namespace ns3 {
	void RemoveFutileNodes(NodeContainer senseNodes,NetDeviceContainer senseDevices,NodeContainer sinkNodes,NodeContainer mobileSinkNode);
   void ContinueSptRouting(Ptr<Node> n, uint32_t jumps, Ipv4Address sinkAdr);
   uint32_t AnalyzeSptPacket(stringstream &ss, Ipv4Address &source);
   void StartSptRoutingFrom(Ptr<Node> n,NodeContainer sinkNodes,NodeContainer mobileSinkNode);
   void PrepareSptRouting(NodeContainer sinkNodes,NodeContainer mobileSinkNode);
}

#endif
