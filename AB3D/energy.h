#ifndef ENERGY_H
#define ENERGY_H

#include "ns3/animation-interface.h"
#include "energy.h"
#include "tool.h"
#include "global.h"
#include <fstream>

namespace ns3 {
	// void BroadcastDrainNotice(Ptr<Node> node);
//	void EnergyFinalRecord(NodeContainer senseNodes,nodestate NS[]);
	void UpdateEnergyTX(Ptr<Node> fromnode, Ptr<Node> tonode,Ptr<Packet> p);
    void UpdateEnergyRX(Ptr<Node> node, Ptr<Packet> p);
}

#endif
