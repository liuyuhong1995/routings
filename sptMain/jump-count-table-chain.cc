#include "jump-count-table-chain.h"
#include "ns3/log.h"
#include <fstream>
#include <iostream>
#include <iomanip>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("JumpCountTableChain");

JumpCountTableChain::JumpCountTableChain() :            //构造函数
		head(NULL), senseNum(0) {
	NS_LOG_FUNCTION(this);
}

JumpCountTableChain::~JumpCountTableChain(){           //析构函数
	NS_LOG_FUNCTION (this);
}

void JumpCountTableChain::CreateChain() {
	senseNum = 0;
	head = NULL;
}

int32_t JumpCountTableChain::InsertNode(Ipv4Address local) {
	NS_LOG_DEBUG("Inserting "<<local<<" into the chain...");
	if (InquireNode(local) != NULL) {
		NS_LOG_ERROR(
				"InsertNode error: "<<local<<" already exists in the chain, InsertNode terminated");
		return -1;
	} else {
		NS_LOG_INFO("Creating a new TableNode...");
		TableNode *newNode = new TableNode;         //创建一个名为newNode的表节点
		newNode->jcTable = new JumpCountTable;
		newNode->jcTable->CreateTable(local);       //以一个local地址创建一个新的table
		newNode->next = NULL;
		NS_LOG_INFO(
				"A newNode is created: "<<"localAdr = "<<newNode->jcTable->GetLocalAdr());

		TableNode *currentNode = GetHead();
		if (currentNode == NULL) {
			NS_LOG_INFO("This is an empty chain, set newNode as head");
			head = newNode;
			senseNum++;
			SortTableChainByLocal();      //按照Local地址进行排序
			return 0;
		} else {
			NS_LOG_INFO("Looking for the tail of the chain...");
			while (currentNode->next != NULL) {
				NS_LOG_INFO(
						"currentNode->next is not NULL: "<<currentNode->next->jcTable->GetLocalAdr()<<", continue...");
				currentNode = currentNode->next;
			}
			NS_LOG_INFO(
					"Now, currentNode->next is NULL, set newNode as currentNode->next");
			currentNode->next = newNode;
			senseNum++;
			SortTableChainByLocal();
			return 0;
		}
	}
}

int32_t JumpCountTableChain::DeleteNode(Ipv4Address local) {
	NS_LOG_DEBUG("Deleting "<<local<<" from the chain...");
	TableNode *thisNode = InquireNode(local);
	if (thisNode == NULL) {
		NS_LOG_ERROR(
						"DeleteNode error: "<<local<<" does not exists in the chain");
		return -1;
	} else {
		if (thisNode == GetHead()) {
			NS_LOG_INFO(local<<" is the head of the chain, delete it...");
			head = thisNode->next;
			delete thisNode;
			senseNum--;
			NS_LOG_INFO("local = "<<local<<" is deleted");
			if (GetHead() == NULL)
				NS_LOG_INFO("and the chain becomes empty");
			return 0;
		} else {
			NS_LOG_INFO("Looking for "<<local);
			TableNode *precedingNode = GetHead();
			while (precedingNode->next != thisNode)
				precedingNode = precedingNode->next;
			precedingNode->next = thisNode->next;
			delete thisNode;
			senseNum--;
			NS_LOG_INFO(local<<" is deleted");
			return 0;
		}
	}
}

TableNode *JumpCountTableChain::InquireNode(Ipv4Address local) {
	NS_LOG_DEBUG("Looking for "<<local<<" in the chain...");
	if (GetHead() == NULL) {
		NS_LOG_DEBUG("InquireNode error: "<<"This is an empty chain!");
		return NULL;
	} else {
		TableNode *currentNode = GetHead();

		while (currentNode->jcTable->GetLocalAdr() != local) {   //为什么要去jcTable中查找呢
			if (currentNode->next != NULL) {
				currentNode = currentNode->next;
				continue;
			} else
				NS_LOG_INFO(local<<" does not exists in the chain");
				return NULL;
		}
		NS_LOG_INFO(local<<" is found in the chain");
		return currentNode;
	}
}

TableNode *JumpCountTableChain::GetHead() {
	return head;
}

uint32_t JumpCountTableChain::GetSenseNum(){
	return senseNum;
}

void JumpCountTableChain::SortTableChainByLocal(){
	NS_LOG_INFO(
			"JumpCountTable::SortTableChainByLocal: begins...");
	TableNode *thisNode = GetHead();
	if (thisNode == NULL)
		NS_LOG_ERROR(
				"JumpCountTableChain::SortTableChainByLocal error: emtpy chain");
	else {
		for (uint32_t i = GetSenseNum() - 1; i > 0; i--) {
			NS_LOG_INFO("i = "<<i);
			thisNode = GetHead();
			for (uint32_t j = i; j > 0; j--) {
				NS_LOG_INFO("j = "<<j);
				uint32_t thisValue = JumpCountTable::AdrToInt(thisNode->jcTable->GetLocalAdr());
				uint32_t nextValue = JumpCountTable::AdrToInt(thisNode->next->jcTable->GetLocalAdr());
				NS_LOG_INFO(
						"thisValue = "<<thisValue<<", nextValue = "<<nextValue);
				if (thisValue > nextValue) {
					NS_LOG_INFO("Greater than, do exchange");
					ExchangeTableNode(thisNode);
				} else {
					NS_LOG_INFO("Smaller than or equal to, skip");
					thisNode = thisNode->next;
				}
			}
		}
	}
}

void JumpCountTableChain::ExchangeTableNode(TableNode *thisNode){
	TableNode *nextNode = thisNode->next;
		if (thisNode == GetHead()) {
			thisNode->next = nextNode->next;
			nextNode->next = thisNode;
			head = nextNode;
		} else {
			NS_LOG_INFO("thisNode  local address: "<<thisNode->jcTable->GetLocalAdr());
			TableNode *precedingNode = GetHead();
			while (precedingNode->next != thisNode) {
				NS_LOG_INFO("precedingNode  local address: "<<precedingNode->jcTable->GetLocalAdr());
				precedingNode = precedingNode->next;
			}
			NS_LOG_INFO("precedingNode  local address: "<<precedingNode->jcTable->GetLocalAdr());
			thisNode->next = nextNode->next;
			nextNode->next = thisNode;
			precedingNode->next = nextNode;
		}
}

int32_t JumpCountTableChain::InsertRoutingNode(Ipv4Address local, Ipv4Address remote,
		Ipv4Address gateway) {
	NS_LOG_DEBUG(
			"Inserting "<<"remoteAdr = "<<remote
			<<"with gatewayAdr = "<<gateway
			<<" into the JumpCountTable of "
			<<"local = "<<local<<" of the chain...");
	TableNode *tN = InquireNode(local);     //local就是感知节点的地址
	if (tN == NULL) {
		NS_LOG_ERROR(
				"InsertRoutingNode error: "<<"local = " <<local<<" is not in the chain, InsertRoutingNode is terminated");
		return -1;
	} else {
		return tN->jcTable->InsertNode(remote, gateway);
	}
}

int32_t JumpCountTableChain::DeleteRoutingNode(Ipv4Address local,
		Ipv4Address remote) {
	NS_LOG_DEBUG(
			"Deleting "<<"remote = "<<remote<<" into the JumpCountTable of "<<"local = "<<local<<" of the chain...");
	TableNode *tN = InquireNode(local);
	if (tN == NULL) {
		NS_LOG_ERROR(
				"DeleteRoutingNode error: "<<"local = " <<local<<" is not in the chain, InsertRoutingNode is terminated");
		return -1;
	} else {
		return tN->jcTable->DeleteNode(remote);
	}
}

int32_t JumpCountTableChain::UpdateJumpCountTable(Ipv4Address local,
		Ipv4Address remote, Ipv4Address gateway, uint32_t newJumps) {
	NS_LOG_DEBUG(
			"Updating the jumps of the JumpCountTable of local = "<<local<<" for remote = "
			<<remote<<" with newJumps = "<<newJumps<<" and gatewayAdr = "<<gateway<<"...");
	TableNode *tN = InquireNode(local);
	if (tN == NULL) {
		NS_LOG_ERROR(
				"UpdateJumpCountTable error: local = "
				<<local <<" is not in the chain, UpdateJumpCountTable is terminated");
		return -1;
	} else {
		return tN->jcTable->UpdateTableNode(remote, gateway, newJumps);
	}
}

RoutingNode *JumpCountTableChain::InquireRoutingNode(Ipv4Address local,
		Ipv4Address remote) {
	NS_LOG_DEBUG(
			"Looking for remote = "<<remote<<" in the JumpCountTable of local = "<<local<<" of the chain...");
	TableNode *tN = InquireNode(local);
	if (tN == NULL) {
		NS_LOG_ERROR("InquireRoutingNode error: local = "<<local<<" is not in the chain");
		return NULL;
	} else {
		NS_LOG_INFO(
				"local = "<<local<<" is found in the chain, now looking for remote = "
				<<remote<<" in the TableNode of it...");
		RoutingNode *rN = tN->jcTable->InquireNode(remote);
		return rN;
	}
}

Ipv4Address JumpCountTableChain::GetRoutingGatewayAdr(Ipv4Address local,
		Ipv4Address remote) {
	NS_LOG_DEBUG(
			"Get gateway to remote = "<<remote<<" in the JumpCountTable of local = "<<local<<" of the chain...");
	TableNode *tN = InquireNode(local);
	if (tN == NULL) {
		NS_LOG_ERROR(
				"GetRoutingGatewayAdr: local = "<<local<<" is not in the chain");
		return Ipv4Address("0.0.0.0");
	} else
		return tN->jcTable->GetGatewayAdr(remote);
}

void JumpCountTableChain::ListJumpCountTable(Ipv4Address local, const std::string name) {
	NS_LOG_DEBUG("Listing the table of local = "<<local<<"...");
	TableNode *tN = InquireNode(local);
	if (tN == NULL) {
		NS_LOG_ERROR(
				"ListJumpCountTable error: local = "<<local<<" is not in the chain");
	} else {
		tN->jcTable->ListTable(name);
	}
}

void JumpCountTableChain::ListJumpCountTableChain(const std::string name) {
	std::stringstream ss;
	std::stringstream si;
	ss << name;
	ss.clear();
	std::ofstream of(ss.str().c_str());
	TableNode *tN = GetHead();
	RoutingNode *rN;
//	std::string s;
	while (tN != NULL) {
		ss << std::endl<<"Local = " << tN->jcTable->GetLocalAdr() << std::endl;
		ss << std::setw(20) << std::left << "Remote";
		ss << std::setw(10) << std::left << "Jumps";
		ss << std::setw(20) << std::left << "Gateway"<<std::endl;
		rN = tN->jcTable->GetHead();
		while (rN != NULL) {
//			si<<rN->remoteAdr;
//			si>>s;
//			ss << std::setw(20) << std::left << s;
//			si.clear();
//			s.clear();
//			ss << std::setw(10) << std::left << rN->jumps;
//			si<<rN->gatewayAdr;
//			si>>s;
//			ss << std::setw(20) << std::left << s << std::endl;
//			si.clear();
//			s.clear();
			si<<rN->remoteAdr;
			ss<< std::setw(20) << std::left << si.str();
			si.str("");
			si.clear();
			ss << std::setw(10) << std::left << rN->jumps;
			si<<rN->gt->ListGatewayTable();
			ss << std::left << si.str() << std::endl;
			si.str("");
			si.clear();
			rN = rN->next;
		}
		tN = tN->next;
	}
	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();
}

void JumpCountTableChain::SortTableNodeByRemote() {
	NS_LOG_INFO("JumpCountTableChain::SortTableNodeByRemote: begins ");
	TableNode *tN = GetHead();
	if (tN == NULL)
		NS_LOG_ERROR(
				"JumpCountTableChain::SortTableNodeByRemote error: empty chain!");
	else {
		while (tN != NULL) {
			NS_LOG_INFO(
					"JumpCountTableChain::SortTableNodeByRemote: local = "<<tN->jcTable->GetLocalAdr()<<"...");
			tN->jcTable->SortByRemote();
			tN = tN->next;
		}
	}
}

} //NS3 namespace
