#ifndef JUMP_COUNT_TABLE_H
#define JUMP_COUNT_TABLE_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "gateway-table.h"

namespace ns3 {

struct RoutingNode {
	Ipv4Address remoteAdr;
	GatewayTable *gt;
	Ipv4Address gatewayAdr;
	uint32_t jumps;
	RoutingNode *next;
};

class JumpCountTable {
public:
	JumpCountTable();
	~JumpCountTable();
	void CreateTable(Ipv4Address local);
	int32_t InsertNode(Ipv4Address remote, Ipv4Address gateway);
	int32_t DeleteNode(Ipv4Address remote);
	int32_t UpdateTableNode(Ipv4Address remote, Ipv4Address gateway, uint32_t newJump);
//	int32_t UpdateTableNode(Ipv4Address remote, Ipv4Address gateway);
	void ListTable(const std::string name);
	void SortByRemote();
	void ExchangeRoutingNode(RoutingNode *thisNode);
	RoutingNode *InquireNode(Ipv4Address remote);
	RoutingNode *GetHead();
	uint32_t GetSinkNum();
	Ipv4Address GetLocalAdr();
	Ipv4Address GetGatewayAdr(Ipv4Address remote);
	uint32_t GetJumps(Ipv4Address remote);
	static uint32_t AdrToInt(Ipv4Address adr);

private:
	RoutingNode *head;
	Ipv4Address localAdr;
	uint32_t sinkNum;
};
} //ns3 namespace

#endif /*JUMP_COUNT_TABLE_H*/
