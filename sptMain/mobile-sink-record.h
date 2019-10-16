#ifndef MOBILE_SINK_RECORD_H
#define MOBILE_SINK_RECORD_H

#include "jump-count-table.h"

namespace ns3 {

struct MobileSinkRecordNode {
	Ipv4Address sinkAdr;
	Vector location;
	int32_t recvBytes;
	double remainingE;
	MobileSinkRecordNode *next;
};

class MobileSinkRecord {
public:
	MobileSinkRecord();
	~MobileSinkRecord();
	void CreateRecord();
	int32_t InsertRecordNode(Ipv4Address sinkAdr, Vector location, double remainingE);
	int32_t UpdateRecordNode(Ipv4Address sinkAdr, double remainingE,int32_t recvBytes);
	void ListMobileSinkRecord(const std::string name);
	void SortBySink();
	void ExchangeRecordNode(MobileSinkRecordNode *thisRecordNode);
	MobileSinkRecordNode *InquireRecordNode(Ipv4Address sinkAdr);
	MobileSinkRecordNode *GetHead();
	uint32_t GetSinkNum();
	uint32_t GetTotalRecvBytes();

private:
	MobileSinkRecordNode *head;
	uint32_t sinkNum;
	uint32_t totalRecvBytes;
};
} //ns3 namespace

#endif /*MOBILE_SINK_RECORD_H*/
