#include "mobile-sink-record.h"
#include "ns3/log.h"
#include <iomanip>
#include <fstream>
//该记录表存放汇聚节点地址、坐标、剩余能量、接收比特数
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MobileSinkRecord");

MobileSinkRecord::MobileSinkRecord() :
		head(NULL),
		sinkNum(0),
		totalRecvBytes(0)
{
	NS_LOG_FUNCTION(this);
}

MobileSinkRecord::~MobileSinkRecord() {
	NS_LOG_FUNCTION(this);
}

void MobileSinkRecord::CreateRecord() {
	sinkNum = 0;
	totalRecvBytes = 0;
	head = NULL;
}

int32_t MobileSinkRecord::InsertRecordNode(Ipv4Address sinkAdr, Vector location, double remainingE){
	NS_LOG_DEBUG("MobileSinkRecord::InsertRecordNode: "<<"sinkAdr = "
			<<sinkAdr<<", location = "<<location<<", remainingE = "<<remainingE<<"...");
	MobileSinkRecordNode *sRN = InquireRecordNode(sinkAdr);
	if(sRN!=NULL){
		NS_LOG_ERROR(
						"MobileSinkRecord::InsertRecordNode ERROR: "<<"sinkAdr = "
						<<sinkAdr<<" already exists, InsertNode is terminated, return -1!");
				return -1;
	}
	else{
		NS_LOG_DEBUG("MobileSinkRecord::InsertRecordNode: Creating a new MobileSinkRecordNode...");
		MobileSinkRecordNode *newRecordNode = new MobileSinkRecordNode;
		newRecordNode->sinkAdr = sinkAdr;
		newRecordNode->location = location;
		newRecordNode->recvBytes =0.0;
		newRecordNode->remainingE = remainingE;
		newRecordNode->next = NULL;
		NS_LOG_DEBUG(
				"MobileSinkRecord::InsertRecordNode: A newRecordNode is created: "
				<<"sinkAdr = "<<newRecordNode->sinkAdr <<", location = "
				<<newRecordNode->location <<", remainingE = "<<newRecordNode->remainingE);

				MobileSinkRecordNode *currentRecordNode = GetHead();
				if (currentRecordNode == NULL) {
					NS_LOG_DEBUG("MobileSinkRecord::InsertRecordNode: This is an empty record, set newRecordNode as head");
					head = newRecordNode;
					sinkNum++;
					NS_LOG_DEBUG("MobileSinkRecord::InsertRecordNode: Done");
					SortBySink();
					return 0;
				} else {
					NS_LOG_DEBUG("MobileSinkRecord::InsertRecordNode: Looking for the tail of the record...");
					while (currentRecordNode->next != NULL) {
						NS_LOG_DEBUG(
								"MobileSinkRecord::InsertRecordNode: currentRecord->next is not NULL: "
								<<currentRecordNode->next->sinkAdr<<", continue...");
						currentRecordNode = currentRecordNode->next;
					}
					NS_LOG_DEBUG(
							"MobileSinkRecord::InsertRecordNode: Now, currentRecord->next is NULL, set newRecord as currentRecord->next");
					currentRecordNode->next = newRecordNode;
					sinkNum++;
					NS_LOG_DEBUG("MobileSinkRecord::InsertRecordNode: Done");
					SortBySink();
					return 0;
				}
	}
}

int32_t MobileSinkRecord::UpdateRecordNode(Ipv4Address sinkAdr, double remainingE,
		int32_t recvBytes) {
	NS_LOG_DEBUG(
			"MobileSinkRecord::UpdateRecordNode: sinkAdr = "<<sinkAdr <<", remainingE = "
			<<remainingE <<", recvBytes= "<<recvBytes<<"...");
	MobileSinkRecordNode *thisRecordNode = InquireRecordNode(sinkAdr);
	if (thisRecordNode == NULL) {
		NS_LOG_ERROR(
				"MobileSinkRecord::UpdateRecordNode ERROR: "<<"sinkAdr = "<<sinkAdr
				<<" does not exist in the MobileSinkRecord, return -1!");
		return -1;
	} else {
		thisRecordNode->remainingE = remainingE;
		thisRecordNode->recvBytes += recvBytes;
		totalRecvBytes+=recvBytes;
		NS_LOG_DEBUG("MobileSinkRecord::UpdateRecordNode: sinkAdr = "<<sinkAdr<<" is updated");
		return 0;
	}
}

void MobileSinkRecord::ListMobileSinkRecord(const std::string name){
	NS_LOG_INFO("MobileSinkRecord::ListMobileSinkRecord: Listing the MobileSinkRecord...");
	if (GetHead() == NULL) {
		NS_LOG_WARN("MobileSinkRecord::ListMobileSinkRecord WARN: Empty record, nothing is to be listed!");
	} else {
		std::stringstream ss;
		ss<<name;
		std::ofstream of(ss.str().c_str());
		ss<<std::endl;
		NS_LOG_INFO("MobileSinkRecord::ListMobileSinkRecord: The MobileSinkRecord  is as follows:");
		ss<<"Generated at :"<<Simulator::Now().GetSeconds()<<std::endl;
		ss << std::setw(20) << std::left << "Sink Address";
		ss<<std::setw(30)<<std::left<<"Sink Location";
//		ss << std::setw(20) << std::left << "Remaining Energy";
		ss << std::setw(20) << std::left << "Received Bytes" << std::endl;
		MobileSinkRecordNode *currentRecordNode = GetHead();
		while (currentRecordNode != NULL) {
			std::stringstream si;
			si << currentRecordNode->sinkAdr;
			ss << std::setw(20) << std::left << si.str();
			si.str("");
			si << currentRecordNode->location;
			ss << std::setw(30) << std::left << si.str();
			si.str("");
//			si << currentRecordNode->remainingE;
//			ss << std::setw(20) << std::left << si.str();
//			si.str("");
			si << currentRecordNode->recvBytes;
			ss << std::setw(20)<< std::left << si.str();
			si.str("");
			ss << std::endl;
			currentRecordNode = currentRecordNode->next;
		}
		ss << std::setw(15) << std::left << "Total Bytes: "<<totalRecvBytes<<std::endl;
		NS_LOG_INFO(ss.str().c_str());
		of<<ss.str();
		of.close();
	}
}

void MobileSinkRecord::SortBySink(){
	NS_LOG_DEBUG("MobileSinkRecord::SortBySink: Begins...");
		MobileSinkRecordNode *thisRecordNode = GetHead();
		if (thisRecordNode == NULL)
			NS_LOG_WARN(
					"MobileSinkRecord::SortBySink WARN: The MobileSinkRecord is empty!");
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

inline void MobileSinkRecord::ExchangeRecordNode(MobileSinkRecordNode *thisRecordNode){
	MobileSinkRecordNode *nextRecordNode = thisRecordNode->next;
	if (thisRecordNode == GetHead()) {
		thisRecordNode->next = nextRecordNode->next;
		nextRecordNode->next = thisRecordNode;
		head = nextRecordNode;
	} else {
		NS_LOG_LOGIC("thisRecord  sinkAdr = "<<thisRecordNode->sinkAdr);
		MobileSinkRecordNode *precedingRecordNode = GetHead();
		while (precedingRecordNode->next != thisRecordNode) {   //找到this节点前面那个节点
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

MobileSinkRecordNode *MobileSinkRecord::InquireRecordNode(Ipv4Address sinkAdr){
	NS_LOG_DEBUG("MobileSinkRecord::InquireRecordNode: Looking for "<<"sinkAdr = "<<sinkAdr
				<<" in the MobileSinkRecord...");
		if (GetHead() == NULL) {
			NS_LOG_WARN("MobileSinkRecord::InquireRecordNode WARN: "
					<<"The mobile sink reorcd is empty, return NULL!");
			return NULL;
		} else {
			MobileSinkRecordNode *currentRecordNode = GetHead();

			while (currentRecordNode->sinkAdr != sinkAdr) {
				if (currentRecordNode->next != NULL) {
					currentRecordNode = currentRecordNode->next;
					continue;
				} else
					NS_LOG_DEBUG("MobileSinkRecord::InquireRecordNode: sinkAdr = "<<sinkAdr
							<<" does not exist in the MobileSinkRecord, return NULL");
					return NULL;
			}
			NS_LOG_DEBUG("MobileSinkRecord::InquireRecordNode: "<<"sinkAdr = "<<sinkAdr
					<<" is found in the MobileSinkNode");
			return currentRecordNode;
		}
}

MobileSinkRecordNode *MobileSinkRecord::GetHead() {
	return head;
}

uint32_t MobileSinkRecord::GetSinkNum() {
	return sinkNum;
}

uint32_t MobileSinkRecord::GetTotalRecvBytes(){
	return totalRecvBytes;
}

}//namespace ns3
