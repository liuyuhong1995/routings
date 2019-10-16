#include "gateway-table.h"
#include "ns3/log.h"
#include <iomanip>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE("GatewayTable");

GatewayTable::GatewayTable(Ipv4Address remote) :         //有参数的构造函数
		head(NULL), gtwNum(0) {
	NS_LOG_FUNCTION(this);
	remoteAdr = remote;
}
GatewayTable::GatewayTable() :          //无参数的构造函数
		head(NULL), gtwNum(0) {
	NS_LOG_FUNCTION(this);
}

GatewayTable::~GatewayTable(){          //析构函数
	NS_LOG_FUNCTION (this);
}

int32_t GatewayTable::InsertNode(Ipv4Address gtw){
	NS_LOG_DEBUG("Inserting "<<gtw<<" into the table...");
	if (InquireNode(gtw) != NULL) {
		NS_LOG_ERROR(
				"InsertNode error: "<<gtw<<" already exists in the table, InsertNode terminated");
		return -1;
	} else {
		NS_LOG_INFO("Creating a new GatewayTable...");
		GatewayNode *newNode = new GatewayNode;
		newNode->gtwAdr = gtw;
		NS_LOG_INFO(
				"A newNode is created: "<<"gtwAdr = "<<newNode->gtwAdr);

		GatewayNode *currentNode = GetHead();
		if (currentNode == NULL) {
			NS_LOG_INFO("This is an empty table, set newNode as head");
			head = newNode;       //这个head是头
			newNode->next=head;      //这个head是尾
			gtwNum++;
			return 0;
		} else {
			NS_LOG_INFO("Looking for the tail of the table...");
			while (currentNode->next != head) {    //这个head是尾
				NS_LOG_INFO(
						"currentNode->next is not the head: "
						<<", continue...");
				currentNode = currentNode->next;
			}
			NS_LOG_INFO(
					"Now, currentNode->next is the head, set newNode as currentNode->next");
			currentNode->next = newNode;
			newNode->next=head;      //这个head是尾
			gtwNum++;
			return 0;
		}
	}
}

int32_t GatewayTable::DeleteNode(Ipv4Address gtw){
	NS_LOG_DEBUG("Deleting "<<gtw<<" from the GatewayTable of "<<remoteAdr<<"...");
	GatewayNode *targetNode=InquireNode(gtw);
	GatewayNode *previousNode=targetNode;       //??
	if (targetNode == NULL) {         //没有这个目标节点
		NS_LOG_WARN(
				"DeleteNode WARN: "<<gtw<<" does not exist in the table, DeleteNode terminated");
		return -1;
	}
	if (GetGatewayNum() == 1) {      //网关数量为1
		gtwNum = 0;
		head = NULL;
		delete targetNode;
		NS_LOG_DEBUG("Deleting Done!");
		return 0;
	} else {
		while (previousNode->next != targetNode) {
			previousNode = previousNode->next;
		}
		NS_LOG_DEBUG("previousNode="<<previousNode->gtwAdr);
		if (targetNode == GetHead()) {     //目标节点在表的头部
			head = targetNode->next;
		}
		previousNode->next = targetNode->next;   //目标节点不在表的头部
		gtwNum--;
		delete targetNode;
		NS_LOG_DEBUG("Deleting Done!");
		return 0;
	}
}

GatewayNode *GatewayTable::InquireNode(Ipv4Address gtw){    //返回值类型为网关节点
	NS_LOG_DEBUG("Looking for "<<"gtwAdr = "<<gtw
				<<" in the table of remoteAdr = "<<GetRemoteAdr()<<"...");
		if (GetHead() == NULL) {
			NS_LOG_DEBUG("InquireNode error: "<<"This is an empty table!");
			return NULL;
		} else {
			GatewayNode *currentNode = GetHead();

			while (currentNode->gtwAdr != gtw) {
				if (currentNode->next != head) {
					currentNode = currentNode->next;
					continue;
				} else
					NS_LOG_INFO("gtwAdr = "<<gtw<<" does not exists in the table");
					return NULL;
			}
			NS_LOG_INFO("gtwAdr = "<<gtw<<" is found in the table");
			return currentNode;
		}
}

GatewayNode *GatewayTable::GetHead(){     //返回值类型为网关节点
	return head;
}

uint32_t GatewayTable::GetGatewayNum(){
	return gtwNum;
}

Ipv4Address GatewayTable::GetRemoteAdr(){
	return remoteAdr;
}

Ipv4Address GatewayTable::GetCurrentGatewayAdr(){
	if(GetHead()==NULL){
		return Ipv4Address("0.0.0.0");
	}
	return GetHead()->gtwAdr;
}

int32_t GatewayTable::RotateTable() {   //旋转表
	if (GetGatewayNum() == 1) {
		return 0;
	} else {
		Ipv4Address oldGtw = GetCurrentGatewayAdr();
		Ipv4Address newGtw;
		head = head->next;
		newGtw = GetCurrentGatewayAdr();
		NS_LOG_LOGIC(
				"GatewayTable::RotateTable: For "
				<<GetRemoteAdr()<<" from "<<oldGtw<<" to "<<newGtw);
		return 0;
	}
}

std::string GatewayTable::ListGatewayTable() {
	std::stringstream ss, si;
	GatewayNode *currentNode = GetHead();
	if (currentNode == NULL) {
		ss<<" ";
		return ss.str();
	} else {
		si << currentNode->gtwAdr;        //当前节点的网关地址
		ss << std::setw(10) << std::left << si.str() << " ";
		si.str("");
		si.clear();

		while (currentNode->next != GetHead()) {
			currentNode = currentNode->next;
			si << currentNode->gtwAdr;
			ss << std::setw(10) << std::left << si.str() << " ";
			si.str("");
			si.clear();
		}
		return ss.str();
	}
}

int32_t GatewayTable::SetRemote(Ipv4Address remote){
	remoteAdr=remote;
	return 0;
}

}//namespace ns3
