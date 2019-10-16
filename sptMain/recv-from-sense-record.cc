#include "recv-from-sense-record.h"
#include "ns3/log.h"
#include <iomanip>
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("RecvFromSenseRecord");

RecvFromSenseRecord::RecvFromSenseRecord(){
	NS_LOG_FUNCTION(this);
}

RecvFromSenseRecord::RecvFromSenseRecord(Ipv4Address localAdr){
	NS_LOG_FUNCTION(this);
	sinkAdr=localAdr;
	head = NULL;
	senseNum = 0;
}

RecvFromSenseRecord::~RecvFromSenseRecord(){
	NS_LOG_FUNCTION(this);
}

void RecvFromSenseRecord::CreateRecord(Ipv4Address localAdr){
	sinkAdr=localAdr;
	head = NULL;
	senseNum = 0;
}

int32_t RecvFromSenseRecord::InsertRecordNode(Ipv4Address senseAdr){
	NS_LOG_DEBUG(
			"RecvFromSenseRecord::InsertRecordNode: "<<"senseAdr = " <<senseAdr<<"...");
	SenseRecordNode *senseRN = InquireRecordNode(senseAdr);
	if (senseRN != NULL) {
		NS_LOG_ERROR(
				"RecvFromSenseRecord::InsertRecord ERROR: "<<"senseAdr = "<<senseAdr
				<<" already exists, InsertRecordNode is terminated, return -1!");
		return -1;
	} else {
		NS_LOG_DEBUG(
				"RecvFromSenseRecord::InsertRecordNode: Creating a new SenseRecordNode...");
		SenseRecordNode *newRecordNode = new SenseRecordNode;
		newRecordNode->senseAdr = senseAdr;
		newRecordNode->recvBytes = 0;
		newRecordNode->next = NULL;
		NS_LOG_DEBUG(
				"A newRecordNode is created: " <<"senseAdr = "<<newRecordNode->senseAdr);

		SenseRecordNode *currentRecordNode = GetHead();
		if (currentRecordNode == NULL) {
			NS_LOG_DEBUG(
					"RecvFromSenseRecord::InsertRecordNode: This is an empty record, set newRecordNode as head");
			head = newRecordNode;
			senseNum++;
			NS_LOG_DEBUG("RecvFromSenseRecord::InsertRecordNode: Done");
			SortBySense();
			return 0;
		} else {
			NS_LOG_DEBUG(
					"RecvFromSenseRecord::InsertRecordNode: Looking for the tail of the record...");
			while (currentRecordNode->next != NULL) {
				NS_LOG_DEBUG(
						"RecvFromSenseRecord::InsertRecordNode: currentRecordNode->next is not NULL: "
						<<currentRecordNode->next->senseAdr<<", continue...");
				currentRecordNode = currentRecordNode->next;
			}
			NS_LOG_DEBUG(
					"RecvFromSenseRecord::InsertRecordNode: Now, currentRecordNode->next is NULL, set newRecordNode as currentRecordNode->next");
			currentRecordNode->next = newRecordNode;
			senseNum++;
			NS_LOG_DEBUG("RecvFromSenseRecord::InsertRecordNode: Done");
			SortBySense();
			return 0;
		}
	}
}

int32_t RecvFromSenseRecord::UpdateRecordNode(Ipv4Address senseAdr,
		int32_t recvBytes) {
	NS_LOG_DEBUG(
			"RecvFromSenseRecord::UpdateRecordNode: Updating the record of senseAdr = "
			<<senseAdr <<" , recvBytes = "<<recvBytes<<"...");
	SenseRecordNode *thisRecordNode = InquireRecordNode(senseAdr);
	if (thisRecordNode == NULL) {
		NS_LOG_WARN(
				"RecvFromSenseRecord::UpdateRecord WARN: "<<"senseAdr = "
				<<senseAdr <<" does not exist in the RecvFromSenseRecord, insert it!");
		InsertRecordNode(senseAdr);
		thisRecordNode = InquireRecordNode(senseAdr);
	}
	thisRecordNode->recvBytes += recvBytes;
	NS_LOG_DEBUG(
			"RecvFromSenseRecord::UpdateRecord: senseAdr = "<<senseAdr<<" is updated");
	return 0;
}

//void RecvFromSenseRecord::ListSinkRecvRecord(){
//	NS_LOG_DEBUG("RecvFromSenseRecord::ListSinkRecord: Listing the RecvFromSenseRecord...");
//		if (GetHead() == NULL) {
//			NS_LOG_WARN("RecvFromSenseRecord::ListSinkRecord WARN: Empty record, nothing is to be listed!");
//		} else {
//			std::stringstream ss;
//			ss<<"/home/wuchao/workspace/NS3/pp2-recv-from-sense.record";
//			std::ofstream of(ss.str().c_str());
//			ss.str("");
//			NS_LOG_INFO("The RecvFromSenseRecord is as follows:");
//			ss << std::setw(20) << std::left << "Sense Address";
//			ss << std::setw(20) << std::left << "Received Bytes" << std::endl;
//			SenseRecordNode *currentRecordNode = GetHead();
//			while (currentRecordNode != NULL) {
//				std::stringstream si;
//				si << currentRecordNode->senseAdr;
//				ss << std::setw(20) << std::left << si.str();
//				si.str("");
//				si << currentRecordNode->recvBytes;
//				ss << std::setw(20)<< std::left << si.str();
//				si.str("");
//				ss << std::endl;
//				currentRecordNode = currentRecordNode->next;
//			}
//			NS_LOG_INFO(ss.str().c_str());
//			of<<ss.str();
//			of.close();
//		}
//}

void RecvFromSenseRecord::SortBySense(){
	NS_LOG_DEBUG("RecvFromSenseRecord::SortBySense: Begins...");
	SenseRecordNode *thisRecordNode = GetHead();
	if (thisRecordNode == NULL)
		NS_LOG_WARN("RecvFromSenseRecord::SortBySense WARN: The  SinkRecord is empty!");
	else {
		for (uint32_t i = GetSenseNum() - 1; i > 0; i--) {
			NS_LOG_LOGIC("i = "<<i);
			thisRecordNode = GetHead();
			for (uint32_t j = i; j > 0; j--) {
				NS_LOG_LOGIC("j = "<<j);
				uint32_t thisValue = JumpCountTable::AdrToInt(
						thisRecordNode->senseAdr);
				uint32_t nextValue = JumpCountTable::AdrToInt(
						thisRecordNode->next->senseAdr);
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

void RecvFromSenseRecord::ExchangeRecordNode(SenseRecordNode *thisRecordNode){
	SenseRecordNode *nextRecordNode = thisRecordNode->next;
		if (thisRecordNode == GetHead()) {
			thisRecordNode->next = nextRecordNode->next;
			nextRecordNode->next = thisRecordNode;
			head = nextRecordNode;
		} else {
			NS_LOG_LOGIC("thisRecordNode  senseAdr = "<<thisRecordNode->senseAdr);
			SenseRecordNode *precedingRecordNode = GetHead();
			while (precedingRecordNode->next != thisRecordNode) {
				NS_LOG_LOGIC(
						"precedingRecordNode  senseAdr = "<<precedingRecordNode->senseAdr);
				precedingRecordNode = precedingRecordNode->next;
			}
			NS_LOG_LOGIC("precedingRecordNode  senseAdr = "<<precedingRecordNode->senseAdr);
			thisRecordNode->next = nextRecordNode->next;
			nextRecordNode->next = thisRecordNode;
			precedingRecordNode->next = nextRecordNode;
		}
}

SenseRecordNode *RecvFromSenseRecord::InquireRecordNode(Ipv4Address senseAdr){
	NS_LOG_DEBUG(
			"RecvFromSenseRecord::InquireRecordNode: Looking for " <<"senseAdr = "
			<<senseAdr<<" in the RecvFromSenseRecord...");
			if (GetHead() == NULL) {
				NS_LOG_WARN("RecvFromSenseRecord::InquireRecordNode WARN: "
						<<"This is an empty record, return NULL!");
				return NULL;
			} else {
				SenseRecordNode *currentRecordNode = GetHead();

				while (currentRecordNode->senseAdr != senseAdr) {
					if (currentRecordNode->next != NULL) {
						currentRecordNode = currentRecordNode->next;
						continue;
					} else
						NS_LOG_DEBUG("RecvFromSenseRecord::InquireRecordNode: senseAdr = "<<senseAdr
								<<" does not exists in the RecvFromSenseRecord, return NULL");
						return NULL;
				}
				NS_LOG_DEBUG("senseAdr = "<<senseAdr<<" is found in the RecvFromSenseRecord");
				return currentRecordNode;
			}
}

SenseRecordNode *RecvFromSenseRecord::GetHead(){
	return head;
}

uint32_t RecvFromSenseRecord::GetSenseNum(){
	return senseNum;
}

	int32_t
RecvFromSenseRecord::GetRecvBytes(Ipv4Address senseAdr) {
	NS_LOG_DEBUG(
			"RecvFromSenseRecord::GetRecvBytes: senseAdr = "<<senseAdr<<"...");
	SenseRecordNode *thisRecordNode = InquireRecordNode(senseAdr);
	if(thisRecordNode == NULL) {
		NS_LOG_ERROR("RecvFromSenseRecord::GetRecvBytes ERROR: "
				<<"senseAdr = "<<senseAdr<<" is not in the RecvFromSenseRecord, return -1!");
		return -1;
	}
	return thisRecordNode->recvBytes;
}

} //namespace ns3
