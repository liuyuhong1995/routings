#ifndef GATEWAY_TABLE_H
#define GATEWAY_TABLE_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include <fstream>

namespace ns3 {

struct GatewayNode {
	Ipv4Address gtwAdr;
	GatewayNode *next;
};

class GatewayTable {
public:
	GatewayTable(Ipv4Address remote);
	GatewayTable();
	~GatewayTable();
	int32_t InsertNode(Ipv4Address gtw);
	int32_t DeleteNode(Ipv4Address gtw);
	GatewayNode *InquireNode(Ipv4Address gtw);
	GatewayNode *GetHead();
	uint32_t GetGatewayNum();
	Ipv4Address GetRemoteAdr();
	Ipv4Address GetCurrentGatewayAdr();
	int32_t RotateTable();
	std::string ListGatewayTable();
	int32_t SetRemote(Ipv4Address remote);

private:
	GatewayNode *head;
	Ipv4Address remoteAdr;
	uint32_t gtwNum;
};
} //ns3 namespace

#endif /*GATEWAY_TABLE_H*/
