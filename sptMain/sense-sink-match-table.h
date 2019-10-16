#ifndef SENSE_SINK_MATCH_TABLE_H
#define SENSE_SINK_MATCH_TABLE_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "jump-count-table-chain.h"
//#include "traffic-table.h"

namespace ns3 {

struct MatchNode {
	Ipv4Address senseAdr	;
	Ipv4Address sinkAdr;
	uint32_t jumps;
	MatchNode *next;
};

enum algorithmType {
	spt_Algor = 0x00000000,
	gene_Algor = 0x00000001,
	random_Algor = 0x00000002,
	my_gene_Algor = 0x00000003,
};

class SenseSinkMatchTable {
public:
	SenseSinkMatchTable(JumpCountTableChain jctChain, NodeContainer skNodes,
			algorithmType aT,std::string s,std::string n);
	SenseSinkMatchTable();
	~SenseSinkMatchTable();
	void CreateTable();
	int32_t DeleteNode(Ipv4Address senseAdr);
	void SetSinkNodes(NodeContainer skNodes);
	void ListTable(const std::string name);
	//	int32_t InsertNode(Ipv4Address senseAdr, Ipv4Address sinkAdr, uint32_t jumps);
	MatchNode *InquireNode(Ipv4Address senseAdr);
	MatchNode *GetHead();

	void DoMatch();
	uint32_t GetSenseNum();
	Ipv4Address GetSinkAdr(Ipv4Address senseAdr);
	uint32_t GetTotalJumps();
	double GetSinkVariance();
//	TrafficTable *GetTrafficTable();
	int32_t GetGlobalIdFromIpv4Adr(Ipv4Address adr, NodeContainer nc);
//	void CalculateTraffic();
	void CalculateTotalJumps();
	void CalculateSinkVariance();

	void SptMatchAlgorithm();
	void RandomMatchAlgorithm();
//	void GeneMatchAlgorithm();
//	void SetGeneIteration(uint32_t iteration);
//	void SetPopNum(uint32_t nPop);

private:
	MatchNode *head;
//	TrafficTable *tt;
	JumpCountTableChain jctChain;
	uint32_t senseNum;
	uint32_t sinkNum;
	NodeContainer sinkNodes;
	uint32_t iterationCounts;
	uint32_t nPopForGene;
	algorithmType aType;
	uint32_t totalJumps;
	double sinkVariance;
	std::string path;
	std::string name;
};
} //ns3 namespace

#endif /*SENSE_SINK_MATCH_TABLE_H*/
