#ifndef JUMP_TCOUNT_TABLE_CHAIN_H
#define JUMP_TCOUNT_TABLE_CHAIN_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "jump-count-table.h"

namespace ns3 {

struct TableNode {
	JumpCountTable *jcTable;
	TableNode *next;
};

class JumpCountTableChain {
public:
//	for JumpCountTableChain
	JumpCountTableChain();
	~JumpCountTableChain();
	void CreateChain();
	int32_t InsertNode(Ipv4Address local);
	int32_t DeleteNode(Ipv4Address local);
	TableNode *InquireNode(Ipv4Address local);
	TableNode *GetHead();
	uint32_t GetSenseNum();
	void ListJumpCountTableChain(const std::string name);
	void SortTableChainByLocal();

//	for JumpCountTable
	int32_t InsertRoutingNode(Ipv4Address local, Ipv4Address remote,
			Ipv4Address gateway);
	int32_t DeleteRoutingNode(Ipv4Address local, Ipv4Address remote);
	int32_t UpdateJumpCountTable(Ipv4Address local, Ipv4Address remote,
			Ipv4Address gateway, uint32_t newJumps);
	RoutingNode *InquireRoutingNode(Ipv4Address local, Ipv4Address remote);
	Ipv4Address GetRoutingGatewayAdr(Ipv4Address local, Ipv4Address remote);
	void ListJumpCountTable(Ipv4Address local, const std::string name);
	void SortTableNodeByRemote();
	void ExchangeTableNode(TableNode * thisNode);

private:
	TableNode *head;
	uint32_t senseNum;
};
} //NS3 namespace

#endif /*JUMP_COUNT_TABLE_CHAIN_H*/
