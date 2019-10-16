#ifndef RECV_FROM_SENSE_RECORD_H
#define RECV_FROM_SENSE_RECORD_H

#include "jump-count-table.h"

namespace ns3 {

struct SenseRecordNode {
	Ipv4Address senseAdr;
	int32_t recvBytes;
	SenseRecordNode *next;
};

class RecvFromSenseRecord {
public:
	RecvFromSenseRecord();
	RecvFromSenseRecord(Ipv4Address localAdr);
	~RecvFromSenseRecord();
	void CreateRecord(Ipv4Address localAdr);
	int32_t InsertRecordNode(Ipv4Address senseAdr);
	int32_t UpdateRecordNode(Ipv4Address senseAdr, int32_t recvBytes);
//	void ListSinkRecvRecord();
	void SortBySense();
	void ExchangeRecordNode(SenseRecordNode *thisRecordNode);
	SenseRecordNode *InquireRecordNode(Ipv4Address senseAdr);
	SenseRecordNode *GetHead();
	uint32_t GetSenseNum();
	int32_t GetRecvBytes(Ipv4Address senseAdr);

private:
	SenseRecordNode *head;
	Ipv4Address sinkAdr;
	uint32_t senseNum;
};
} //ns3 namespace

#endif /*RECV_FROM_SENSE_RECORD_H*/
