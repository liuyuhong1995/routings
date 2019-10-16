#ifndef ENERGY_H
#define ENERGY_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/animation-interface.h"
#include "energy.h"
#include "tool.h"

#include <fstream>

namespace ns3 {
	void ManualDrainTest(uint32_t index);
	int32_t ProcessDrainNoticePacket(Ptr<Node> thisNode,
		Ptr<Packet> packet, Ipv4Header h);
	void InstallEnergy(NodeContainer senseNodes);
	void EnergyFinalRecord(NodeContainer senseNodes,double remaingJ[]);
	bool CheckRemainingJ(Ptr<Node> n, Ptr<Packet> pkt);
	bool CheckRemainingJ(Ptr<Node> n);
	void UpdateNodesColorByEnergy(Ptr<Node> n, double per, nodeType nT);
	void UpdateEnergySources(Ptr<Node> thisNode,Ptr<Node> anotherNode, Ptr<Packet> p, uint16_t flag,NodeContainer sinkNodes,NodeContainer mobileSinkNode);
	void DoDrainRecord(Ptr<Node> thisNode);

//	void UpdateEnergySources(Ptr<Node> n, Ptr<Packet> p,uint32_t flag,NodeContainer sinkNodes,
//			NodeContainer mobileSinkNode,double remaingJ[],AnimationInterface *Panim);
	//pktSize   initialJ   original  static
}

#endif
