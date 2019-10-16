#include "sense-sink-match-table.h"
#include "ns3/log.h"
#include <iomanip>
#include <fstream>
#include "jump-count-table.h"
//#include "genetic-match-algorithm.h"

#define MAX_NUM 255
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SenseSinkMatchTable");

SenseSinkMatchTable::SenseSinkMatchTable(JumpCountTableChain jctc,
//		NodeContainer skNodes, algorithmType aT, std::string s, std::string n) :
//		head(NULL), tt(NULL), jctChain(jctc), senseNum(jctc.GetSenseNum()), sinkNum(
//				skNodes.GetN()), sinkNodes(skNodes), iterationCounts(100), nPopForGene(
//				10), aType(aT), totalJumps(0), sinkVariance(0.0), path(s), name(
//				n) {
		NodeContainer skNodes, algorithmType aT, std::string s, std::string n) :
		head(NULL), jctChain(jctc), senseNum(jctc.GetSenseNum()), sinkNum(
				skNodes.GetN()), sinkNodes(skNodes), iterationCounts(100), nPopForGene(
				10), aType(aT), totalJumps(0), sinkVariance(0.0), path(s), name(
				n) {         //有参数的构造函数
	NS_LOG_FUNCTION(this);
//	jctChain = jctc;
//	senseNum = jctc.GetSenseNum();
//	sinkNum = skNodes.GetN();
//	nPopForGene = 10; //Default
//	iterationCounts = 100; //Default
//	sinkNodes = skNodes;
//	head =NULL;
//	aType=aT;
}

SenseSinkMatchTable::SenseSinkMatchTable() {     //无参数的构造函数
	NS_LOG_FUNCTION(this);
}

SenseSinkMatchTable::~SenseSinkMatchTable() {
	NS_LOG_FUNCTION(this);
}

/*
 * 创建一个新的table，head设置为NULL
 */
void SenseSinkMatchTable::CreateTable() {
	NS_LOG_DEBUG("SenseSinkMatchTable::CreateTable ...");
	head = NULL;
//	tt=NULL;
	senseNum = 0;
}

/*
 * 删除一个Node
 * 首先查询是否存在该Node，如果不存在，则返回-1
 * 如果存在该Node，则将该Node删除
 */
int32_t SenseSinkMatchTable::DeleteNode(Ipv4Address senseAdr) {
	NS_LOG_DEBUG("Deleting "<<"senseAdr = "<<senseAdr
			<<" from the SenseSinkMatchTable...");
	MatchNode *thisNode = InquireNode(senseAdr);
	if (thisNode == NULL) {
		NS_LOG_ERROR("DeleteNode error: "<<"senseAdr = "
				<<senseAdr<<" does not exists in the table!");
		return -1;
	} else {
		if (thisNode == GetHead()) {
			NS_LOG_INFO(senseAdr<<" is the head of the table, delete it...");
			head = thisNode->next;
			delete thisNode;
			NS_LOG_INFO("senseAdr = "<<senseAdr<<" is deleted");
			if (GetHead() == NULL)
				NS_LOG_INFO("and the table becomes empty");
			senseNum--;
			return 0;
		} else {
			NS_LOG_INFO("Looking for "<<"senseAdr = "<<senseAdr);
			MatchNode *precedingNode = GetHead();
			while (precedingNode->next != thisNode)
				precedingNode = precedingNode->next;
			precedingNode->next = thisNode->next;
			delete thisNode;
			NS_LOG_INFO("senseAdr = "<<senseAdr<<" is deleted");
			senseNum--;
			return 0;
		}
	}
}

void SenseSinkMatchTable::SetSinkNodes(NodeContainer skNodes){
	sinkNodes = skNodes;
}

/*
 * 查询某个senseAdr地址是否已经存在于链表中
 * 如果发现是空表，返回NULL
 * 如果发现不是空表，则从头到尾遍历整个链表，
 * remote地址存在， * 则返回该senseAdr地址的MatchNode，不存在，则返回NULL
 */
MatchNode *SenseSinkMatchTable::InquireNode(Ipv4Address senseAdr) {
	NS_LOG_DEBUG("SenseSinkMatchTable::InquireNode: Looking for "<<"senseAdr = "<<senseAdr
			<<" in the SenseSinkMatchTable...");
	if (GetHead() == NULL) {
		NS_LOG_WARN("SenseSinkMatchTable::InquireNode WARN: "<<"This is an empty table, return NULL!");
		return NULL;
	} else {
		MatchNode *currentNode = GetHead();

		while (currentNode->senseAdr != senseAdr) {
			if (currentNode->next != NULL) {
				currentNode = currentNode->next;
				continue;
			} else
				NS_LOG_WARN("SenseSinkMatchTable::InquireNode WARN: "<<"senseAdr = "
						<<senseAdr<<" does not exists in the table, return NULL");
				return NULL;
		}
		NS_LOG_INFO("SenseSinkMatchTable::InquireNode: senseAdr = "
				<<senseAdr<<" is found in the SenseSinkMatchTable");
		return currentNode;
	}
}

/*
 * 打印出所有的senseNodes匹配的sinkNode以及对应的jumps值
 */
void SenseSinkMatchTable::ListTable(const std::string name) {
	NS_LOG_DEBUG("SenseSinkMatchTable::ListTable: Listing the SenseSinkMatchTable...");
	std::stringstream ss;
	std::stringstream si;
	ss <<name;
	std::ofstream of(ss.str().c_str());
	MatchNode *mN = GetHead();
	std::string s;
	ss<<std::endl;
	ss << std::setw(15) << std::left << "Sense Adr";
	ss << std::setw(15) << std::left << "Sink Adr";
	ss << std::setw(5) << std::left << "Jumps" << std::endl;
	while (mN != NULL) {
		si << mN->senseAdr;
		si >> s;
		ss << std::setw(15) << std::left << s;
		si.clear();
		si.str("");
		s.clear();
		si << mN->sinkAdr;
		si >> s;
		ss << std::setw(15) << std::left << s;
		si.clear();
		si.str("");
		s.clear();
		ss << std::setw(5) << std::left << mN->jumps << std::endl;
		mN = mN->next;
	}
	NS_LOG_INFO(ss.str().c_str());
	of << ss.str().c_str();
	of.close();
}

/*
 * 返回head节点的指针
 */
MatchNode *SenseSinkMatchTable::GetHead() {
	NS_LOG_LOGIC("SenseSinkMatchTable::GetHead");
	return head;
}

void SenseSinkMatchTable::DoMatch(){
	switch (aType) {
	case spt_Algor: {
		SptMatchAlgorithm();
		break;
	}
//	case gene_Algor: {
//		GeneMatchAlgorithm();
//		break;
//	}
	case random_Algor: {
		RandomMatchAlgorithm();
		break;
	}
//	case my_gene_Algor:{
//		GeneMatchAlgorithm();
//		break;
//	}
	default: {
		SptMatchAlgorithm();
		break;
	}
	}
}

/*
 * 返回senseNum
 */
uint32_t SenseSinkMatchTable::GetSenseNum() {
	NS_LOG_LOGIC("SenseSinkMatchTable::GetSenseNum");
	return senseNum;
}

/*
 * 返回remote的gateway
 */
Ipv4Address SenseSinkMatchTable::GetSinkAdr(Ipv4Address senseAdr) {
	NS_LOG_DEBUG("SenseSinkMatchTable::GetSinkAdr: senseAdr = "<<senseAdr<<"...");
	MatchNode *mN = InquireNode(senseAdr);
	if (mN == NULL) {
		NS_LOG_ERROR(
				"SenseSinkMatchTable::GetGatewayAdr ERROR: senseAdr ="<<senseAdr
				<<" matches no sinkNode");
		return Ipv4Address("0.0.0.0");
	} else
		NS_LOG_DEBUG("SenseSinkMatchTable::GetSinkAdr: Done, return "<<mN->sinkAdr);
		return mN->sinkAdr;
}

uint32_t SenseSinkMatchTable::GetTotalJumps() {
	return totalJumps;
}

double SenseSinkMatchTable::GetSinkVariance(){
	return sinkVariance;      //汇聚方差
}

//TrafficTable *SenseSinkMatchTable::GetTrafficTable(){
//	return tt;
//}

int32_t SenseSinkMatchTable::GetGlobalIdFromIpv4Adr(Ipv4Address adr,
		NodeContainer nc) {
	for (NodeContainer::Iterator i = nc.Begin(); i != nc.End(); i++) {
		Ptr<Node> n = *i;
		if (n->GetObject<Ipv4>()->GetAddress(1,0).GetLocal() == adr) {    //??
			return n->GetId();
		}
	}
	return -1;
}

void SenseSinkMatchTable::CalculateTotalJumps() {
	MatchNode *thisMatchNode = GetHead();
	if (thisMatchNode == NULL) {
		NS_LOG_ERROR("SenseSinkMatchTable::CalculateTotalJumps ERROR: "
				"Empty SenseSinkMatchTable");
		Simulator::Stop();
	}
	while (thisMatchNode != NULL) {
		NS_LOG_DEBUG(
				"SenseSinkMatchTable::CalculateTotalJumps: thisMatchNode jumps="
				<<thisMatchNode->jumps);
		totalJumps += thisMatchNode->jumps;
		NS_LOG_DEBUG(
				"SenseSinkMatchTable::CalculateTotalJumps:totalJumps="
				<<totalJumps);
		thisMatchNode = thisMatchNode->next;
	}
}

void SenseSinkMatchTable::CalculateSinkVariance() {   //计算汇聚方差
	MatchNode *curMn = GetHead();
	if (curMn == NULL) {
		NS_LOG_ERROR("SenseSinkMatchTable::CalculateSinkVariance ERROR: "
				"Empty SenseSinkMatchTable");
		Simulator::Stop();
	}

	uint32_t tempArray[MAX_NUM] = { 0 };
	uint32_t sinkArray[sinkNum] = { 0 };
	double avg = 0;
	uint32_t totalNum = 0;
	uint32_t quadraticSum = 0;     //二次和
	int32_t sinkId=0;

	while (curMn->next != NULL) {
		sinkId = GetGlobalIdFromIpv4Adr(curMn->sinkAdr, sinkNodes);
		NS_ASSERT(sinkId > 0&&sinkId<MAX_NUM);
		tempArray[sinkId]++;       //统计Id在（0,MAX_NUM)之间的
		curMn = curMn->next;
	}

	uint32_t sinkIndex=0;
	for (uint32_t i = 0; i < MAX_NUM; i++) {
		if (tempArray[i] != 0) {
			NS_ASSERT(sinkIndex < sinkNum);
			sinkArray[sinkIndex] = tempArray[i];    //把存在Id的sink传给sinkArray
			sinkIndex++;
		}
	}

	for (uint32_t k = 0; k < sinkNum; k++) {
		totalNum += sinkArray[k];
	}

	avg = totalNum * 1.0 / sinkNum;
	for (uint32_t m = 0; m < sinkNum; m++) {
		quadraticSum += (sinkArray[m] - avg) * (sinkArray[m] - avg);
	}

	sinkVariance=quadraticSum*1.0/sinkNum;      //方差
}

/*
 * 最小跳数算法
 * 每个senseNode在其对应的JumpCountTable里面
 * 查找jumps最小的sinkNode
 * 作为data gathering的destination
 * 如果跳数一样的sinkNode，随机选择
 */
void SenseSinkMatchTable::SptMatchAlgorithm() {
	NS_LOG_DEBUG("SenseSinkMatchTable::SptMatchAlgorithm: Begins...");
	TableNode *tN = jctChain.GetHead();
	if (tN == NULL) {
		NS_LOG_ERROR(
				"SenseSinkMatchTable::SptMatchAlgorithm ERROR: "
				<<"Empty JumpCountTableChain, stop simulation!");
		Simulator::Stop();
	} else {
		MatchNode *currentNode = GetHead();
		RoutingNode *currentRoutingNode;
		RoutingNode *rN;
		while (tN != NULL) {    // 遍历操作
			uint32_t lowestJumpsCount=0;
			uint32_t lowestJumps=0;
			rN = tN->jcTable->GetHead();
			if (rN == NULL) { //Technically impossible
				NS_LOG_ERROR(
						"SenseSinkMatchTable::SptMatchAlgorithm ERROR: "
						<<"Empty JumpCountTable, stop simulation!");
				Simulator::Stop();
			} else {
				NS_LOG_INFO(std::endl<<
						"SenseSinkMatchTable::SptMatchAlgorithm: "
						"Looking for the RoutingNode with the smallest jumps for ");
//				tN->jcTable->ListTable();
				currentRoutingNode = rN;
				while (currentRoutingNode != NULL) {
//					if (currentRoutingNode->jumps < rN->jumps) { //The greater global ID is, the more chance to choose it
//						rN = currentRoutingNode;
//					}else if(currentRoutingNode->jumps == rN->jumps){
//						uint16_t possibility=rand()%2;
//						if(possibility==0){
//
//						}
//					}
//					currentRoutingNode = currentRoutingNode->next;
					if (currentRoutingNode->jumps < rN->jumps) {   //找到最短跳数的路由
						lowestJumpsCount = 1;     //统计有几个sink节点同时跳数最小
						rN = currentRoutingNode;
						lowestJumps=currentRoutingNode->jumps;
					} else if (currentRoutingNode->jumps == rN->jumps) {
						lowestJumpsCount++;
						lowestJumps=currentRoutingNode->jumps;
					}
					currentRoutingNode = currentRoutingNode->next; // 遍历链表
				}//while done

				NS_LOG_LOGIC("lowestJumpsCount="<<lowestJumpsCount<<", lowestJumps="<<lowestJumps);
				RoutingNode *lowestJumpsRoutingNodes[lowestJumpsCount];
				uint32_t index = 0;
				currentRoutingNode = tN->jcTable->GetHead();
				while (currentRoutingNode != NULL) {
					if (currentRoutingNode->jumps == lowestJumps) {
						NS_ASSERT(index<lowestJumpsCount);
						lowestJumpsRoutingNodes[index] = currentRoutingNode;  //存放最小跳数的路由节点
						index++;     //统计有几个路由节点同时跳数最小
					}
					currentRoutingNode = currentRoutingNode->next;
				}
				uint32_t possibility = rand() % lowestJumpsCount;
				NS_LOG_INFO("possibility="<<possibility);
				rN = lowestJumpsRoutingNodes[possibility];       //随机选择跳数最小的路由节点
				NS_LOG_INFO("SenseSinkMatchTable::SptMatchAlgorithm: choose the smallest jumps RoutingNode done");

				NS_LOG_INFO(
						"SenseSinkMatchTable::SptMatchAlgorithm: Creating a new MatchNode...");
				MatchNode *newNode = new MatchNode;
				newNode->senseAdr = tN->jcTable->GetLocalAdr();
				newNode->sinkAdr = rN->remoteAdr;
				newNode->jumps = tN->jcTable->GetJumps(newNode->sinkAdr);
				newNode->next = NULL;
				NS_LOG_INFO(
						"SenseSinkMatchTable::SptMatchAlgorithm: New MatchNode is created, senseAdr ="
						<<newNode->senseAdr<<", sinkAdr = "<<newNode->sinkAdr<<", jumps = "<<newNode->jumps);

				if (currentNode == NULL) {
					NS_LOG_INFO(
							"SenseSinkMatchTable::SptMatchAlgorithm: " <<"This is an empty SenseSinkMatchTable, set the newNode as head...");
					head = newNode;
					currentNode = newNode;

				} else {
					NS_LOG_INFO(
							"SenseSinkMatchTable::SptMatchAlgorithm: " <<currentNode->senseAdr <<" is the tail of the SenseSinkMatchTable, set the newNode as its next...");
					currentNode->next = newNode;
					currentNode = newNode;
				}
			}
			tN = tN->next;
		}
	}
//	CalculateTraffic();
	CalculateTotalJumps();
	CalculateSinkVariance();
//	ListTable();
	NS_LOG_DEBUG("SenseSinkMatchTable::SptMatchAlgorithm: Done");
}

/*
 * 随机选择目标sink算法
 */
void SenseSinkMatchTable::RandomMatchAlgorithm() {
	NS_LOG_DEBUG("SenseSinkMatchTable::RandomMatchAlgorithm: Begins...");
	TableNode *tN = jctChain.GetHead();
	if (tN == NULL) {
		NS_LOG_ERROR(
				"SenseSinkMatchTable::RandomMatchAlgorithm ERROR: "
				<<"Empty JumpCountTableChain, stop simulation!");
		Simulator::Stop();
		std::exit(-1);
	} else {
		MatchNode *currentNode = GetHead();
		RoutingNode *DstRn;
		while (tN != NULL) {
			uint32_t nCandidates = tN->jcTable->GetSinkNum(); //number of sinks this senseNode can reach
			uint32_t chooseIndex = rand() % nCandidates; //0到nCandidates-1的范围的整数
			DstRn = tN->jcTable->GetHead();
			while (chooseIndex > 0) {
				DstRn = DstRn->next;
				chooseIndex--;
			}
			NS_LOG_INFO(
					"SenseSinkMatchTable::RandomMatchAlgorithm: Random choose done");
			NS_LOG_INFO(
					"SenseSinkMatchTable::RandomMatchAlgorithm: Creating a new MatchNode...");
			MatchNode *newNode = new MatchNode;
			newNode->senseAdr = tN->jcTable->GetLocalAdr();
			newNode->sinkAdr = DstRn->remoteAdr;
			newNode->jumps = tN->jcTable->GetJumps(newNode->sinkAdr);
			newNode->next = NULL;
			NS_LOG_INFO(
					"SenseSinkMatchTable::RandomMatchAlgorithm: New MatchNode is created, senseAdr ="
					<<newNode->senseAdr<<", sinkAdr = "<<newNode->sinkAdr<<", jumps = "<<newNode->jumps);

			if (currentNode == NULL) {
				NS_LOG_INFO(
						"SenseSinkMatchTable::RandomMatchAlgorithm: "
						<<"This is an empty SenseSinkMatchTable, set the newNode as head...");
				head = newNode;
				currentNode = newNode;

			} else {
				NS_LOG_INFO(
						"SenseSinkMatchTable::RandomMatchAlgorithm: "
						<<currentNode->senseAdr <<" is the tail of the SenseSinkMatchTable, set the newNode as its next...");
				currentNode->next = newNode;
				currentNode = newNode;
			}
			tN = tN->next;
		}
	}
//	CalculateTraffic();
	CalculateTotalJumps();
	CalculateSinkVariance();
//	ListTable();
	NS_LOG_DEBUG("SenseSinkMatchTable::RandomMatchAlgorithm: Done");
}

/*
 * MASP的遗传选择算法
 */
/*void SenseSinkMatchTable::GeneMatchAlgorithm() {
	NS_LOG_DEBUG("SenseSinkMatchTable::GeneMatchAlgorithm: Begins...");
	chrom *optChrom;

	geneType gT;
	switch(aType){
	case gene_Algor:{
		gT=masp_Algor;
		break;
	}
	case my_gene_Algor:{
		gT=my_Algor;
		break;
	}
	default: {
		NS_LOG_ERROR(
				"SenseSinkMatchTable::GeneMatchAlgorithm ERROR: Wrong algorithmType");
		break;
	}
	}

//	GeneticMatchAlgorithm *geneOperator=
//			new GeneticMatchAlgorithm(jctChain, sinkNodes, nPopForGene, gT);
//	geneOperator->SetIterationCounts(iterationCounts);
//	optChrom =geneOperator->DoGeneticMatch();
	if (optChrom == NULL) {
		NS_LOG_ERROR("SenseSinkMatchTable::GeneMatchAlgorithm ERROR: "
				"GeneticMatchAlgorithm failed with NULL, exit -1");
		Simulator::Stop();
		std::exit(-1);
	}

	//Generate SenseSinkMatchTable via optChrom
	MatchNode *currentMatchNode = GetHead();
	TableNode *currentTableNode = jctChain.GetHead();
	for (uint32_t i = 0; i < senseNum; i++) {
		MatchNode *newMatchNode = new MatchNode;
		newMatchNode->senseAdr = currentTableNode->jcTable->GetLocalAdr();
		newMatchNode->next = NULL;
		for (uint32_t j = 0; j < sinkNum; j++) {
			if (optChrom->gene[i][j] == 1) {
				newMatchNode->sinkAdr =
						sinkNodes.Get(j)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
				newMatchNode->jumps = currentTableNode->jcTable->GetJumps(
						newMatchNode->sinkAdr);
			}
		}
		if (currentMatchNode == NULL) {
			head = newMatchNode;
			currentMatchNode = newMatchNode;
			currentTableNode=currentTableNode->next;
		} else {
			currentMatchNode->next = newMatchNode;
			currentMatchNode = newMatchNode;
			currentTableNode=currentTableNode->next;
		}
	}
	totalJumps=optChrom->fitValue;
	sinkVariance=optChrom->ufitValue;

//	tt = new	TrafficTable;
//	tt=optChrom->tt;
//	tt->ListTrafficTable();
//	ListTable();
	NS_LOG_DEBUG("SenseSinkMatchTable::GeneMatchAlgorithm: Done");
	NS_LOG_UNCOND("SenseSinkMatchTable::GeneMatchAlgorithm: fitValue/ufitValue="
			<<optChrom->fitValue<<"/"<<optChrom->ufitValue);
}*/

/*void SenseSinkMatchTable::SetGeneIteration(uint32_t iteration) {
	NS_LOG_DEBUG("SenseSinkMatchTable::SetGeneIteration: iteration="<<iteration);
	iterationCounts = iteration;
}

void SenseSinkMatchTable::SetPopNum(uint32_t nPop) {
	NS_LOG_DEBUG("SenseSinkMatchTable::SetPopNum: nPop="<<nPop);
	nPopForGene = nPop;
}*/
}//namespace ns3
