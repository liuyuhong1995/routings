#ifndef SINK_RECORD_H
#define SINK_RECORD_H

#include "jump-count-table.h"//for a method
#include "recv-from-sense-record.h"

namespace ns3 {

struct SinkRecordNode{
	Ipv4Address sinkAdr;
	RecvFromSenseRecord *rfsR;
	int32_t totalRecvBytes;
	int32_t BytesToSend;
	uint32_t senseNum;
	SinkRecordNode *next;
};

class SinkRecord {
public:
	SinkRecord();
	~SinkRecord();
	void CreateRecord();
	int32_t InsertRecordNode(Ipv4Address sinkAdr);
	int32_t UpdateRecordNode(Ipv4Address sinkAdr, Ipv4Address senseAdr,
			int32_t recvBytes);
	int32_t UpdateAfterDataGathering(Ipv4Address sinkAdr, int32_t updateBytes);
	void ListSinkRecord(const std::string name);
	void SortBySink();
	void ExchangeRecordNode(SinkRecordNode *thisRecordNode);
	SinkRecordNode *InquireRecordNode(Ipv4Address sinkAdr);
	SinkRecordNode *GetHead();
	uint32_t GetSinkNum();
	int32_t GetBytesToSend(Ipv4Address sinkAdr);

private:
	SinkRecordNode *head;
	uint32_t sinkNum;
};
} //ns3 namespace

#endif /*SINK_RECORD_H*/
