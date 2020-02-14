#ifndef PACKET_MANAGE_H
#define PACKET_MANAGE_H
#include "ns3/animation-interface.h"
#include "tool.h"
#include <string>
#include <map>
#include <vector>
#include "energy.h"
using namespace std;

namespace ns3 {
	void BroadcastPacket(Ptr<Node> node,pktType ptype,int size);
	void SendPacket(Ptr<Node>thisNode,Ptr<Node>tonode,pktType ptype,int size);
	void RecvPacketCallback(Ptr<Socket> socket);
	void BS_RecvPacketCallback(Ptr<Socket> socket);
	void inerstmap(Ptr<Node>thisNode,Ptr<Node>nenode);
	void neighbordiscover(NodeContainer nodes,double circletime);
	void data_trans(NodeContainer nodes,double circletime);
	void F_DataPacket(Ptr<Node>thisnode,Ptr<Packet>pkt);
}
#endif
