#include "jump-count-table.h"
#include "ns3/log.h"
#include <iomanip>
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("JumpCountTable");

JumpCountTable::JumpCountTable() {
	NS_LOG_FUNCTION(this);      //输出JumpCountTable函数的调用信息
}

JumpCountTable::~JumpCountTable() {
	NS_LOG_FUNCTION(this);      //输出析构函数的调用信息
}

/*
 * 以一个local地址创建一个新的table，head设置为NULL，表中记录感知节点到达汇聚节点的跳数
 */
void JumpCountTable::CreateTable(Ipv4Address local) {
	NS_LOG_DEBUG("Creating a new JumpCountTable for local = "<<local<<"...");
	localAdr = local;
	head = NULL;
	sinkNum = 0;
}

/*
 * 插入一个remote地址标示的Node
 * 如果待插入Node的address已存在，则返回-1
 * 如果待插入Node的address不存在，则创建newNode，如果是空表，则将newNode设置为head，返回0
 * 如果不是空表，则把newNode加入到链表的最后，返回0
 */
int32_t JumpCountTable::InsertNode(Ipv4Address remote, Ipv4Address gateway) {
	NS_LOG_DEBUG("Inserting remote = "<<remote<<" into the table of local = "<<GetLocalAdr()<<"...");
	if (InquireNode(remote) != NULL) {
		NS_LOG_ERROR(
				"InsertNode error: "<<"remote = "<<remote<<" already exists, InsertNode is terminated");
		return -1;
	} else {
		NS_LOG_INFO("Creating a new RoutingNode...");
		RoutingNode *newNode = new RoutingNode;
		newNode->jumps = 0;
		newNode->remoteAdr = remote;
		newNode->gt = new GatewayTable;
		newNode->gt->SetRemote(remote);    //新节点的网关远端
		newNode->gt->InsertNode(gateway);  //插入网关节点
		newNode->gatewayAdr=newNode->gt->GetCurrentGatewayAdr();
		newNode->next = NULL;
		NS_LOG_INFO(
				"A newNode is created: "
				<<"jumps = "<<newNode->jumps
				<<", remoteAdr = "<<newNode->remoteAdr
				<<", gatewayAdr = "<<newNode->gatewayAdr);

		RoutingNode *currentNode = GetHead();
		if (currentNode == NULL) {
			NS_LOG_INFO("This is an empty table, set newNode as head");
			head = newNode;
			sinkNum++;
			SortByRemote();      //按照远端地址进行排序
			return 0;
		} else {
			NS_LOG_INFO("Looking for the tail of the table...");
			while (currentNode->next != NULL) {
				NS_LOG_INFO(
						"currentNode->next is not NULL: "<<currentNode->next->remoteAdr<<", continue...");
				currentNode = currentNode->next;
			}
			NS_LOG_INFO(
					"Now, currentNode->next is NULL, set newNode as currentNode->next");
			currentNode->next = newNode;
			sinkNum++;
			SortByRemote();
			return 0;
		}
	}
}

/*
 * 删除一个Node
 * 首先查询是否存在该Node，如果不存在，则返回-1
 * 如果存在该Node，则将该Node删除
 */
int32_t JumpCountTable::DeleteNode(Ipv4Address remote) {
	NS_LOG_DEBUG("Deleting "<<"remote = "<<remote
			<<" from the JumpCountTable of local = "<<GetLocalAdr()<<"...");
	RoutingNode *thisNode = InquireNode(remote);
	if (thisNode == NULL) {
		NS_LOG_ERROR("DeleteNode error: "<<"remote = "
				<<remote<<" does not exists in the table!");
		return -1;
	} else {
		if (thisNode == GetHead()) {
			NS_LOG_INFO(remote<<" is the head of the table, delete it...");
			head = thisNode->next;
			delete thisNode;
			NS_LOG_INFO("remote = "<<remote<<" is deleted");
			if (GetHead() == NULL)
				NS_LOG_INFO("and the table becomes empty");
			sinkNum--;
			return 0;
		} else {
			NS_LOG_INFO("Looking for "<<"remote = "<<remote);
			RoutingNode *precedingNode = GetHead();
			while (precedingNode->next != thisNode)
				precedingNode = precedingNode->next;
			precedingNode->next = thisNode->next;
			delete thisNode;
			NS_LOG_INFO("remote = "<<remote<<" is deleted");
			sinkNum--;
			return 0;
		}
	}
}

/*
 * 查询某个remote地址是否已经存在于链表中
 * 如果发现是空表，返回NULL
 * 如果发现不是空表，则从头到尾遍历整个链表，
 * remote地址存在， * 则返回该remote地址的Node，不存在，则返回NULL
 */
RoutingNode *JumpCountTable::InquireNode(Ipv4Address remote) {
	NS_LOG_DEBUG("Looking for "<<"remote = "<<remote
			<<" in the table of local = "<<GetLocalAdr()<<"...");
	if (GetHead() == NULL) {
		NS_LOG_DEBUG("InquireNode error: "<<"This is an empty table!");
		return NULL;
	} else {
		RoutingNode *currentNode = GetHead();

		while (currentNode->remoteAdr != remote) {
			if (currentNode->next != NULL) {
				currentNode = currentNode->next;
				continue;
			} else
				NS_LOG_INFO("remote = "<<remote<<" does not exists in the table");
				return NULL;
		}
		NS_LOG_INFO("remote = "<<remote<<" is found in the table");
		return currentNode;
	}
}

/*
 * 更新某个节点的jump值和gatewayAdr
 */
int32_t JumpCountTable::UpdateTableNode(Ipv4Address remote,      //remote为sink的地址
		Ipv4Address gateway, uint32_t newJumps) {
	NS_LOG_DEBUG("Updating the jumps of JumpCountTable of local = "<<GetLocalAdr()
			<<" for remote = "<<remote<<" with newJumps = "<<newJumps
			<<" and gatewayAdr = "<<gateway<<"...");
	RoutingNode *thisNode = InquireNode(remote);
	if (thisNode == NULL) {
		NS_LOG_WARN(
				"UpdateTableNode WARN: "<<"remote = "<<remote <<" does not exists in the table, insert one!");
		InsertNode(remote, gateway);
		thisNode = InquireNode(remote);
	}
		thisNode->jumps = newJumps;
		thisNode->gt = new GatewayTable;
		thisNode->gt->InsertNode(gateway);
		thisNode->gt->SetRemote(remote);
		thisNode->gatewayAdr =thisNode->gt->GetCurrentGatewayAdr();
		NS_LOG_INFO("remote = "<<remote<<" is updated");
		return 0;
}

/*
 * 打印出所有的Node与对应的jump值
 */
void JumpCountTable::ListTable(const std::string name) {
	NS_LOG_DEBUG("Listing the JumpCountTable of local = "<<GetLocalAdr()<<"...");
	if (GetHead() == NULL) {
		NS_LOG_ERROR("JumpCountTable::ListTable error: Empty table for local = "<<GetLocalAdr()<<"!");
	} else {
		std::stringstream ss;
		ss<<"/home/wsn/sim_temp/record_files/"<<name<<"-"<<GetLocalAdr();
		std::ofstream of(ss.str().c_str());
		ss.str("");
		NS_LOG_INFO("The routing table of "<<GetLocalAdr()<<" is as follows:");
		ss << std::setw(15) << std::left << "Remote";
		ss << std::setw(15) << std::left << "Gateway";
		ss << std::setw(10) << std::left << "Jumps" << std::endl;
		RoutingNode *currentNode = GetHead();
		while (currentNode != NULL) {
			std::stringstream si;
			si << currentNode->remoteAdr;
			ss << std::setw(15) << std::left << si.str();
			si.str("");
			si << currentNode->gatewayAdr;
			ss << std::setw(15) << std::left << si.str();
			si.str("");
			si << currentNode->jumps;
			ss << std::setw(10)<< std::left << si.str();
			si.str("");
			ss << std::endl;
			currentNode = currentNode->next;
		}
		NS_LOG_UNCOND(ss.str().c_str());
		of<<ss.str();
		of.close();
	}
}

/*
 *将TableNode的节点按照remote的排序
 */
void JumpCountTable::SortByRemote(){
	NS_LOG_LOGIC(
			"JumpCountTable::SortByRemote: local = "<<GetLocalAdr()<<"...");
	RoutingNode *thisNode = GetHead();
	if (thisNode == NULL)
		NS_LOG_ERROR(
				"JumpCountTable::SortByRemote error: local = "<<GetLocalAdr()<<" has an empty table");
	else {
		for (uint32_t i = GetSinkNum() - 1; i > 0; i--) {
			NS_LOG_INFO("i = "<<i);
			thisNode = GetHead();
			for (uint32_t j = i; j > 0; j--) {
				NS_LOG_INFO("j = "<<j);
				uint32_t thisValue = AdrToInt(thisNode->remoteAdr);
				uint32_t nextValue = AdrToInt(thisNode->next->remoteAdr);
				NS_LOG_INFO(
						"thisValue = "<<thisValue<<", nextValue = "<<nextValue);
				if (thisValue > nextValue) {
					NS_LOG_INFO("Greater than, do exchange");
					ExchangeRoutingNode(thisNode);    //先把最大的排在最后
				} else {
					NS_LOG_INFO("Smaller than or equal to, skip");
					thisNode = thisNode->next;
				}
			}
		}
	}
}

/*
 *交换两个RoutingNode的位置
 */
void JumpCountTable::ExchangeRoutingNode(RoutingNode *thisNode){
	RoutingNode *nextNode = thisNode->next;
	if (thisNode == GetHead()) {
		thisNode->next = nextNode->next;    //把thisNode放在第二位
		nextNode->next = thisNode;          //把nextNode放在第一位
		head = nextNode;                    //头为nextNode
	} else {
		NS_LOG_INFO("thisNode  remote address: "<<thisNode->remoteAdr);
		RoutingNode *precedingNode = GetHead();
		while (precedingNode->next != thisNode) {
			NS_LOG_INFO("precedingNode  remote address: "<<precedingNode->remoteAdr);
			precedingNode = precedingNode->next;
		}
		NS_LOG_INFO("precedingNode  remote address: "<<precedingNode->remoteAdr);
		thisNode->next = nextNode->next;
		nextNode->next = thisNode;
		precedingNode->next = nextNode;
	}
}

/*
 * 返回head节点的指针
 */
RoutingNode *JumpCountTable::GetHead() {
	return head;
}

/*
 * 返回localAdr
 */
Ipv4Address JumpCountTable::GetLocalAdr() {
	return localAdr;
}

/*
 * 返回sinkNum
 */
uint32_t JumpCountTable::GetSinkNum() {
	return sinkNum;
}

/*
 * 返回remote的gateway
 */
Ipv4Address JumpCountTable::GetGatewayAdr(Ipv4Address remote) {
	RoutingNode *rN = InquireNode(remote);
	if (rN == NULL) {
		NS_LOG_ERROR(
				"JumpCountTable::GetGatewayAdr error: local ="<<GetLocalAdr()
				<<" can not reach remoteAdr = "<<remote<<" by any gateway!");
		return Ipv4Address("0.0.0.0");
	} else
		rN->gatewayAdr=rN->gt->GetCurrentGatewayAdr();
		return rN->gatewayAdr;
}

/*
 * Return jumps of remote
 */
uint32_t JumpCountTable::GetJumps(Ipv4Address remote) {
	NS_LOG_DEBUG(__FUNCTION__<<": remote = "<<remote);
	RoutingNode *rN = InquireNode(remote);
	if (rN) {
		NS_LOG_DEBUG(__FUNCTION__<<": remote = "
				<<remote<<"exits in the JumpCountTable, return its jumps");
		return rN->jumps;
	} else{
		NS_LOG_WARN(__FUNCTION__<<" WARN: remote = "
				<<remote<<"dose not exit in the JumpCountTable, return 9999");
		return 9999;}
}

/*
 * 将para adr 的最后一位数字返回
 */
uint32_t JumpCountTable::AdrToInt(Ipv4Address adr){
	std::stringstream ss;
	ss<<adr;
	uint8_t dotFlag=0;
	char c[ss.str().size()];
	uint32_t i[4];
	ss>>c;
	ss.str("");
	ss.clear();
	char *p = c;
	while (dotFlag < 4) {
		while (*p != '.') {
			ss << *p;
			p++;
		}
		ss >> i[dotFlag];
		ss.str("");
		ss.clear();
		p++;
		dotFlag++;
	}
	ss<<i[0]<<" "<<i[1]<<" "<<i[2]<<" "<<i[3];
	NS_LOG_LOGIC("JumpCountTable::AdrToInt result : "<<ss.str());
	return i[3];
}

}//namespace ns3
