#include "sink-record.h"
#include "ns3/log.h"
#include <iomanip>
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SinkRecord");

SinkRecord::SinkRecord() :
		head(NULL), sinkNum(0) {
	NS_LOG_FUNCTION(this);
}

SinkRecord::~SinkRecord() {
	NS_LOG_FUNCTION(this);
}

void SinkRecord::CreateRecord() {
	sinkNum = 0;
	head = NULL;
}

int32_t SinkRecord::InsertRecordNode(Ipv4Address sinkAdr){
	NS_LOG_DEBUG("SinkRecord::InsertRecordNode: "<<"sinkAdr = "
				<<sinkAdr<<"...");
	SinkRecordNode *sinkRN = InquireRecordNode(sinkAdr);
	if(sinkRN!=NULL){
		NS_LOG_ERROR(
						"SinkRecord::InsertRecord ERROR: "<<"sinkAdr = "<<sinkAdr
						<<" already exists, InsertRecordNode is terminated, return -1!");
				return -1;
	}
	else{
		NS_LOG_DEBUG("SinkRecord::InsertRecordNode: Creating a new SinkRecordNode...");
		SinkRecordNode *newRecordNode = new SinkRecordNode;
		newRecordNode->sinkAdr = sinkAdr;
		newRecordNode->totalRecvBytes = 0;
		newRecordNode->BytesToSend = 0;
		newRecordNode->senseNum = 0;
		newRecordNode->rfsR = new RecvFromSenseRecord(sinkAdr);
//		newRecordNode->rfsR->CreateRecord(sinkAdr);
		newRecordNode->next = NULL;
		NS_LOG_DEBUG(
						"A newRecordNode is created: "
						<<"sinkAdr = "<<newRecordNode->sinkAdr);

		SinkRecordNode *currentRecordNode = GetHead();
				if (currentRecordNode == NULL) {
					NS_LOG_DEBUG("SinkRecord::InsertRecordNode: This is an empty record, set newRecordNode as head");
					head = newRecordNode;
					sinkNum++;
					NS_LOG_DEBUG("SinkRecord::InsertRecordNode: Done");
					SortBySink();
					return 0;
				} else {
					NS_LOG_DEBUG("SinkRecord::InsertRecordNode: Looking for the tail of the record...");
					while (currentRecordNode->next != NULL) {
						NS_LOG_DEBUG(
								"SinkRecord::InsertRecordNode: currentRecordNode->next is not NULL: "
								<<currentRecordNode->next->sinkAdr<<", continue...");
						currentRecordNode = currentRecordNode->next;
					}
					NS_LOG_DEBUG(
							"SinkRecord::InsertRecordNode: Now, currentRecordNode->next is NULL, set newRecordNode as currentRecordNode->next");
					currentRecordNode->next = newRecordNode;
					sinkNum++;
					NS_LOG_DEBUG("SinkRecord::InsertRecordNode: Done");
					SortBySink();
					return 0;
				}
	}
}

int32_t SinkRecord::UpdateRecordNode(Ipv4Address sinkAdr, Ipv4Address senseAdr,
		int32_t recvBytes) {
	NS_LOG_DEBUG(
			"SinkRecord::UpdateRecordNode: Updating the record of sinkAdr = "
			<<sinkAdr <<", senseAdr = "<<senseAdr<<" , recvBytes = "<<recvBytes<<"...");
	SinkRecordNode *thisRecordNode = InquireRecordNode(sinkAdr);
	if (thisRecordNode == NULL) {
		NS_LOG_WARN(
				"SinkRecord::UpdateRecord WARN: "<<"sinkAdr = " <<sinkAdr
				<<" does not exist in the SinkRecord, insert it!");
		InsertRecordNode(sinkAdr);
		thisRecordNode = InquireRecordNode(sinkAdr);
	}
		thisRecordNode->totalRecvBytes += recvBytes;
		thisRecordNode->BytesToSend += recvBytes;
		thisRecordNode->rfsR->UpdateRecordNode(senseAdr, recvBytes);
		thisRecordNode->senseNum = thisRecordNode->rfsR->GetSenseNum();
		NS_LOG_DEBUG(
				"SinkRecord::UpdateRecord: sinkAdr = "<<sinkAdr<<" is updated");
		return 0;
	}

int32_t SinkRecord::UpdateAfterDataGathering(Ipv4Address sinkAdr, int32_t updateBytes) {
	NS_LOG_DEBUG(
			"SinkRecord::UpdateAfterDataGathering: Updating the BytesToSend of sinkAdr = "
			<<sinkAdr<<" , updateBytes = "<<updateBytes<<"...");
	SinkRecordNode *thisRecordNode = InquireRecordNode(sinkAdr);
		if (thisRecordNode == NULL) {
			NS_LOG_WARN(
					"SinkRecord::UpdateAfterDataGathering WARN: "<<"sinkAdr = "
					<<sinkAdr <<" does not exist in the SinkRecord, insert it!");
			InsertRecordNode(sinkAdr);
			thisRecordNode = InquireRecordNode(sinkAdr);}
		thisRecordNode->BytesToSend += updateBytes;
		NS_ASSERT(thisRecordNode->BytesToSend>=0);
		NS_LOG_DEBUG("SinkRecord::UpdateAfterDataGathering: sinkAdr = "<<sinkAdr
				<<" is updated, BytesToSend = "<<thisRecordNode->BytesToSend);
		return 0;
}

void SinkRecord::ListSinkRecord(const std::string name){
	NS_LOG_DEBUG("SinkRecord::ListSinkRecord: Listing the SinkRecord...");
	if (GetHead() == NULL) {
		NS_LOG_WARN("SinkRecord::ListSinkRecord WARN: Empty record, nothing is to be listed!");
	} else {
		std::stringstream ss;
		ss << name;
		std::ofstream of(ss.str().c_str());
		ss<<std::endl;
		NS_LOG_INFO("The SinkRecord is as follows:");
		SinkRecordNode *currentRecordNode = GetHead();
		while (currentRecordNode != NULL) {
			std::stringstream si;
			ss << std::setw(15) << std::left << "Sink Address: ";
			ss << currentRecordNode->sinkAdr;
			ss<<"     ";
			ss << std::setw(15) << std::left << "Num of Sense:";
			ss << currentRecordNode->senseNum<<std::endl;

			ss << std::setw(15) << std::left << "Total Recv Bytes: ";
			ss << currentRecordNode->totalRecvBytes;
			ss<<"     ";
			ss << std::setw(15) << std::left << "Bytes To Send:";
			ss << currentRecordNode->BytesToSend<<std::endl;

			ss << std::setw(15) << std::left << "Sense Address";
			ss << std::setw(10) << std::left << "Recv Bytes"<<std::endl;

			SenseRecordNode *srN = currentRecordNode->rfsR->GetHead();
			while(srN != NULL){
				si<<srN->senseAdr;
				ss<< std::setw(15) << std::left << si.str();
				si.str("");
				si<<srN->recvBytes;
				ss<< std::setw(10) << std::left << si.str();
				si.str("");
				ss << std::endl;
				srN=srN->next;
			}
			currentRecordNode = currentRecordNode->next;
			ss<< std::endl;
		}
		NS_LOG_INFO(ss.str().c_str());
		of<<ss.str();
		of.close();
	}
}

void SinkRecord::SortBySink(){
	NS_LOG_DEBUG("SinkRecord::SortBySink: Begins...");
		SinkRecordNode *thisRecordNode = GetHead();
		if (thisRecordNode == NULL)
			NS_LOG_WARN(
					"SinkRecord::SortBySink WARN: The  SinkRecord is empty!");
		else {
			for (uint32_t i = GetSinkNum() - 1; i > 0; i--) {
				NS_LOG_LOGIC("i = "<<i);
				thisRecordNode = GetHead();
				for (uint32_t j = i; j > 0; j--) {
					NS_LOG_LOGIC("j = "<<j);
					uint32_t thisValue = JumpCountTable::AdrToInt(thisRecordNode->sinkAdr);
					uint32_t nextValue = JumpCountTable::AdrToInt(thisRecordNode->next->sinkAdr);
					NS_LOG_LOGIC(
							"thisValue = "<<thisValue<<", nextValue = "<<nextValue);
					if (thisValue > nextValue) {
						NS_LOG_LOGIC("Greater than, do exchange");
						ExchangeRecordNode(thisRecordNode);
					} else {
						NS_LOG_LOGIC("Smaller than or equal to, skip");
						thisRecordNode = thisRecordNode->next;
					}
				}
			}
		}
}

inline void SinkRecord::ExchangeRecordNode(SinkRecordNode *thisRecordNode){
	SinkRecordNode *nextRecordNode = thisRecordNode->next;
	if (thisRecordNode == GetHead()) {
		thisRecordNode->next = nextRecordNode->next;
		nextRecordNode->next = thisRecordNode;
		head = nextRecordNode;
	} else {
		NS_LOG_LOGIC("thisRecordNode  sinkAdr = "<<thisRecordNode->sinkAdr);
		SinkRecordNode *precedingRecordNode = GetHead();
		while (precedingRecordNode->next != thisRecordNode) {
			NS_LOG_LOGIC(
					"precedingRecordNode  sinkAdr = "<<precedingRecordNode->sinkAdr);
			precedingRecordNode = precedingRecordNode->next;
		}
		NS_LOG_LOGIC("precedingRecordNode  sinkAdr = "<<precedingRecordNode->sinkAdr);
		thisRecordNode->next = nextRecordNode->next;
		nextRecordNode->next = thisRecordNode;
		precedingRecordNode->next = nextRecordNode;
	}
}

SinkRecordNode *SinkRecord::InquireRecordNode(Ipv4Address sinkAdr){
	NS_LOG_DEBUG("SinkRecord::InquireRecordNode: Looking for "<<"sinkAdr = "<<sinkAdr
				<<" in the SinkRecord...");
		if (GetHead() == NULL) {
			NS_LOG_WARN("SinkRecord::InquireRecordNode WARN: "
					<<"This is an empty record, return NULL!");
			return NULL;
		} else {
			SinkRecordNode *currentRecordNode = GetHead();

			while (currentRecordNode->sinkAdr != sinkAdr) {
				if (currentRecordNode->next != NULL) {
					currentRecordNode = currentRecordNode->next;
					continue;
				} else
					NS_LOG_DEBUG("SinkRecord::InquireRecordNode: sinkAdr = "<<sinkAdr
							<<" does not exists in the SinkRecord, return NULL");
					return NULL;
			}
			NS_LOG_DEBUG("sinkAdr = "<<sinkAdr<<" is found in the SinkRecord");
			return currentRecordNode;
		}
}

SinkRecordNode *SinkRecord::GetHead() {
	return head;
}

uint32_t SinkRecord::GetSinkNum() {
	return sinkNum;
}

int32_t SinkRecord::GetBytesToSend(Ipv4Address sinkAdr){
	NS_LOG_DEBUG(
				"SinkRecord::GetRecvBytes: sinkAdr = "<<sinkAdr<<"...");
	SinkRecordNode *thisRecordNode = InquireRecordNode(sinkAdr);
//	if(thisRecordNode == NULL){
//		NS_LOG_ERROR("SinkRecord::GetBytesToSend ERROR: "
//				<<"sinkAdr = "<<sinkAdr<<" is not in the SinkRecord, return -1!");
//		return -1;
//	}
	NS_ASSERT(thisRecordNode!=NULL);
	return thisRecordNode->BytesToSend;
}
}//namespace ns3
