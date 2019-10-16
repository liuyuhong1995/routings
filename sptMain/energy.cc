#include "energy.h"
#include <string>
#include <iomanip>
#include "ns3/mobility-module.h"
#include "tool.h"
#include "jump-count-table-chain.h"

using namespace std;
using namespace ns3;

extern string thisSimPath;//simulation path
extern string exeName;
//声明外部变量
extern double totalConsumed;
extern double initialJ;
extern double TxPara;//0.000006J/Bytes for Tx
extern double RxPara;//0.000006J/Bytes for Rx
extern uint32_t nDrain;
extern bool firstDrainFlag;
extern uint32_t firstDrainedNodeId;
extern double firstDrainedNodeTime;
extern uint32_t nDrain;
extern double simStopTime;
extern double remaingJ[MAX_NUM];
//extern bool flag;
extern uint32_t pktSize;
extern bool Banim;
extern uint32_t firstDrainedSinkId;
extern AnimationInterface *Panim;
extern double simStopTime;


extern InetSocketAddress broadcastAdr;
extern TypeId tid;

extern JumpCountTableChain jctChain;

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Energy");
/*
 * 测试手动将某个节点的能量设置为最低量
 */
void ManualDrainTest(uint32_t index){
	remaingJ[index]=pktSize*2*TxPara;          //最低量为什么等于这些？
}
/*
 * 用于处理当senseNode收到drainNotice_Type的pkt
 */
int32_t ProcessDrainNoticePacket(Ptr<Node> thisNode,     //说明要把source从网关表中删掉
		Ptr<Packet> packet, Ipv4Header h){
	Ipv4Address targetAdr = h.GetSource();
	Ipv4Address thisAdr = GetNodeIpv4Address(thisNode);
	NS_LOG_LOGIC(std::endl<<TIME_STAMP_FUC<<thisAdr<<" received a drainNotice_Type pkt from "
			<<targetAdr);

	TableNode *thisTn = jctChain.InquireNode(GetNodeIpv4Address(thisNode));
	NS_ASSERT(thisTn!=NULL);

	uint32_t nDstSinks = thisTn->jcTable->GetSinkNum();
	RoutingNode *currentRn=thisTn->jcTable->GetHead();
	NS_ASSERT(currentRn!=NULL);

	for (uint32_t i = 0; i < nDstSinks; i++) {
		currentRn->gt->DeleteNode(targetAdr);
		currentRn = currentRn->next;
	}
	return 0;
}


void BroadcastDrainNotice(Ptr<Node> node){
	NS_LOG_LOGIC(TIME_STAMP_FUC<<"BroadcastDrainNotice from "
			<<GetNodeIpv4Address(node));
	Ipv4Header h;
	h.SetIdentification(drainNotice_Type);
	h.SetSource(GetNodeIpv4Address(node));
	Ptr<Packet> drainNoticePkt = Create<Packet>(0);
	drainNoticePkt->AddHeader(h);
	Ptr<Socket> drainNoticeSocket = Socket::CreateSocket(
			node, tid);
	drainNoticeSocket->Connect(broadcastAdr);
	drainNoticeSocket->SetAllowBroadcast(true);
	drainNoticeSocket->Send(drainNoticePkt);
	drainNoticeSocket->Close();
}

void InstallEnergy(NodeContainer senseNodes){
	for (uint32_t i = 0; i < MAX_NUM; i++) {
		remaingJ[i] = -1.0;               //为什么这里是-1.0？
	}
	for (uint32_t i = 0; i < senseNodes.GetN(); i++) {
		remaingJ[senseNodes.Get(i)->GetId()] = initialJ;   //为什么只有感知节点安装能量，汇聚节点没有安装（一开始只有感知节点和ms节点）
	}
}
/*
 * Record the final remaining energy of each senseNodes(including sinkNodes)
 */
void EnergyFinalRecord(NodeContainer senseNodes,double remaingJ[]){  //记录节点Id、剩余能量、总消耗能量
		std::stringstream ss;
		std::stringstream si;
		ss << thisSimPath<<exeName<<"-final-enegy.record";
		std::ofstream of(ss.str().c_str());
		ss << std::endl << "Initial energy = " << initialJ << std::endl;
		ss << std::setw(15) << std::left << "Node Number";
		ss << std::setw(15) << std::left << "Remaining" << std::endl;
		ss << std::setw(15) << std::left << "(global)";
		ss << std::setw(25) << std::left << "Energy" << std::endl;

		uint32_t nNodes = senseNodes.GetN();
		for (uint32_t i = 0; i < nNodes; i++) {
			si << std::setw(15) << std::left << senseNodes.Get(i)->GetId();
			si << std::setw(15) << std::left
					<< remaingJ[senseNodes.Get(i)->GetId()] << std::endl;
			ss << si.str();
			si.str("");
			si.clear();
		}
		ss << "Total comsumed energy = " << totalConsumed;
		NS_LOG_INFO(ss.str().c_str());
		of << ss.str().c_str();
		of.close();
}


bool CheckRemainingJ(Ptr<Node> n, Ptr<Packet> pkt) {
	uint32_t id = n->GetId();
	return remaingJ[id] >= pkt->GetSize() * TxPara;    //检查剩余能量是否还够发送一个包
}

bool CheckRemainingJ(Ptr<Node> n) {
	uint32_t id = n->GetId();
	return remaingJ[id] >= pktSize * TxPara;
}


/*
 * Change nodes color by energy level
 */
void UpdateNodesColorByEnergy(Ptr<Node> n, double per, nodeType nT){
	if (per < 1 && per >= 0.9) {
		if (nT == sink_Type) {
			Panim->UpdateNodeColor(n, 0, 0, 200);      //这四个参数代表啥？(节点指针、红、绿、蓝)
		} else {
			Panim->UpdateNodeColor(n, 200, 0, 0);
		}
	} else if (per < 0.9 && per >= 0.5) {
		if (nT == sink_Type) {
			Panim->UpdateNodeColor(n, 0, 0, 150);     //汇聚节点是蓝色的
		} else {
			Panim->UpdateNodeColor(n, 150, 0, 0);     //感知节点是红色的
		}
	} else if (per < 0.5 && per >= 0.25) {
		if (nT == sink_Type) {
			Panim->UpdateNodeColor(n, 0, 0, 100);
		} else {
			Panim->UpdateNodeColor(n, 100, 0, 0);
		}
	} else {
		if (nT == sink_Type) {
			Panim->UpdateNodeColor(n, 0, 0, 50);
		} else {
			Panim->UpdateNodeColor(n, 50, 0, 0);
		}
	}
}

void UpdateEnergySources(Ptr<Node> n, Ptr<Packet> p, uint16_t flag,NodeContainer sinkNodes,NodeContainer mobileSinkNode) {
	uint32_t id = n->GetId();
	uint32_t thisPktSize = p->GetSize();
	nodeType thisNt = CheckNodeType(n,sinkNodes,mobileSinkNode.Get(0));
	double oldValue = remaingJ[id];
	double newValue = oldValue;
	double consumedJ;
	NS_ASSERT(oldValue > 0);
	switch (flag) {
	case 0: {//Tx
		thisPktSize=p->GetSize();
		
		consumedJ=thisPktSize * TxPara;
		newValue = newValue - consumedJ;
//		if (id == 18) {
//			NS_LOG_DEBUG(TIME_STAMP<<
//					"Node[18] consumed "<<consumedJ<<" for transmission, "<<newValue<<" left");
//		}
		if (newValue <= pktSize * TxPara) {   //剩下的能量不足以再发送一个包
			NS_LOG_DEBUG(
					TIME_STAMP_FUC<<"Node["<<id<<"]'s energy used up on transmission");
			if (Banim) {
				Panim->UpdateNodeColor(n, 0, 0, 0);
			}
			remaingJ[id] = 0;
			totalConsumed+=oldValue;
			if (firstDrainFlag) {
				firstDrainedNodeId = id;
				firstDrainedNodeTime = Simulator::Now().GetSeconds();
				firstDrainFlag=false;
			}
			if (thisNt == sink_Type) {
				NS_LOG_DEBUG(
						TIME_STAMP_FUC<<"Node["<<id<<"] is a sinkNode, stop the simulation");
				firstDrainedSinkId = id;
				nDrain++;
				DoDrainRecord(n);
				simStopTime = Simulator::Now().GetSeconds();
				Simulator::Stop();    //为什么如果干涸的是汇聚节点就要停止仿真？(汇聚节点干涸说明无法传送信息到ms节点了)
			} else {
				nDrain++;
				DoDrainRecord(n);
//				if (Bdrain == true && aType == my_gene_Algor)
//					BroadcastDrainNotice(n);
			}
		} else {
			remaingJ[id] = newValue;
			totalConsumed+=consumedJ;

			if (Banim) {
				double remainingPer = remaingJ[id] / initialJ;
				UpdateNodesColorByEnergy(n, remainingPer, thisNt);
			}
		}
		break;
	}
	case 1: {//Rx
		consumedJ=thisPktSize * RxPara;
		newValue = newValue - consumedJ;
//		if (id == 18) {
//			NS_LOG_DEBUG(TIME_STAMP<<
//					"Node[18] consumed "<<consumedJ<<" for reception, "<<newValue<<" left");
//		}
		if (newValue <= pktSize * RxPara) {
			NS_LOG_DEBUG(
					TIME_STAMP_FUC<<"Node["<<id<<"]'s energy used up on reception");
			if (Banim) {
				Panim->UpdateNodeColor(n, 0, 0, 0);
			}
			remaingJ[id] = 0;
			totalConsumed+=oldValue;
			if (firstDrainFlag) {
				firstDrainedNodeId = id;
				firstDrainedNodeTime = Simulator::Now().GetSeconds();
				firstDrainFlag=false;
			}
			if (thisNt == sink_Type) {
				NS_LOG_DEBUG(
								TIME_STAMP_FUC<<"Node["<<id<<"] is a sinkNode, stop the simulation");
				firstDrainedSinkId = id;
				nDrain++;
				DoDrainRecord(n);
				simStopTime = Simulator::Now().GetSeconds();
				Simulator::Stop();
			} else {
				nDrain++;
				DoDrainRecord(n);
//				if (Bdrain == true && aType == my_gene_Algor)
//					BroadcastDrainNotice(n);
			}
		} else {
			remaingJ[id] = newValue;
			totalConsumed+=consumedJ;
			if (Banim) {
				double remainingPer = remaingJ[id] / initialJ;
				UpdateNodesColorByEnergy(n, remainingPer, thisNt);
			}
		}
		break;
	}
	default: {
		break;
	}
	}
}


void DoDrainRecord(Ptr<Node> thisNode){   //记录干涸的时间、地址、Id、坐标
	stringstream ss,si;
	ss << thisSimPath << exeName << "-drain.record";
	ofstream of;
	of.open(ss.str().c_str(), ios::app);
	ss.str("");
	ss.clear();

	double timeNow=Simulator::Now().GetSeconds();
	Ptr<ConstantPositionMobilityModel> cpmm = thisNode->GetObject<
						ConstantPositionMobilityModel>();
	Vector thisLocation = cpmm->GetPosition();	//获取当前节点的坐标
	Ipv4Address thisAdr=GetNodeIpv4Address(thisNode);
	si<<thisAdr;
	uint32_t thisId=thisNode->GetId();

	ss<<setw(10)<<left<<timeNow;

	ss<<setw(15)<<left<<si.str();
	si.str("");
	si.clear();

	ss<<setw(5)<<left<<thisId;

	si<<thisLocation.x;
	si<<":";
	si<<thisLocation.y;
	ss<<si.str();
	si.str("");
	si.clear();

	ss<<endl;

	of<<ss.str().c_str();
	ss.str("");
	ss.clear();
}



}//namespace ns3
