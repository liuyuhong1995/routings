#include "sink_check.h"
#include <string>
#include <iomanip>
#include "ns3/mobility-module.h"
#include "tool.h"
#include "sink-record.h"
#include "mobile-sink-record.h"
using namespace std;
using namespace ns3;


//声明外部变量
extern uint32_t nSinks;
extern bool doSinkCheck;
extern TypeId tid;
extern InetSocketAddress broadcastAdr;
extern double sinkCheckReqInterval;
extern Ipv4InterfaceContainer mobileSinkIf;
extern MobileSinkRecord msRecord;
extern SinkRecord sRecord;
extern bool Banim;
extern AnimationInterface *Panim ;

namespace ns3{
NS_LOG_COMPONENT_DEFINE("Sink_Check");
void SetSinkCheckFlag(bool flag){
	doSinkCheck = flag;
}

/*
 *用于senseNodes和mobileSinkNode收到sinkCheck_Type的packet后的处理方法
 */
//int32_t ProcessSinkCheckPacket(Ptr<Node> thisNode,
//		Ptr<Packet> packet, Ipv4Header h,NodeContainer sinkNodes,NodeContainer mobileSinkNode,NodeContainer senseNodes) {
//	nodeType nType = CheckNodeType(thisNode,sinkNodes,mobileSinkNode.Get(0));	//获取节点类型
//	stringstream recvData;
//	packet->CopyData(&recvData, packet->GetSize());	//获取Packet的内容
//	Vector sourceLocation = Vector(0, 0, 0);
//	switch (nType) {
//	case mobileSink_Type: {
//		cout<<"4：回复广播包以后，Mobile节点收到广播包然后从sense中分离出sink====="<<endl;
//		NS_LOG_LOGIC(FUC<<"mobileSink_Type");
//		Ipv4Address sourceAdr = h.GetSource();	//或者Packet的source节点地址
//		AnalyzeSinkCheckPacket(recvData, sourceLocation);
//
//		NS_LOG_LOGIC(TIME_STAMP<<"Mobile sink received a sink check reply from "
//						<<sourceAdr<<", inquire it in mobile sink record...");
//		MobileSinkRecordNode *msrN = msRecord.InquireRecordNode(sourceAdr);	//获取source节点的SinkRecord
//		if (msrN != NULL) {	//照理说不可能发生，因为如果已经存在了这个SinkRecordNode，sinkNode是不会再给mobileSinkNode发送这个包的
//			return msRecord.UpdateRecordNode(sourceAdr, 0, 0);
//		} else {
//			cout<<"5:进入sink节点分类处理====="<<endl;
//			NS_LOG_INFO(TIME_STAMP<<"Ascertained "
//								<<sourceAdr
//								<<", insert it into MobileSinkRecord and color to blue");
//			if (Banim) {
//				Panim->UpdateNodeColor(GetNodePtrFromIpv4Adr(sourceAdr,sinkNodes,senseNodes,mobileSinkNode), 0, 0,
//						255);
//			}
//			sRecord.InsertRecordNode(sourceAdr);
//			sinkNodes.Add(GetNodePtrFromIpv4Adr(sourceAdr,sinkNodes,senseNodes,mobileSinkNode));
//			cout<<"测试用：====="<<sinkNodes.GetN()<<endl;
//			nSinks++;
//			return msRecord.InsertRecordNode(sourceAdr, sourceLocation, 0);
//		}
//		break;
//	}
//	default: {
//		NS_LOG_LOGIC(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
//				<<" received a sink check packet from mobileSinkNode, inquire it in mobile sink record...");
//		NS_LOG_LOGIC(FUC<<"default");
//		MobileSinkRecordNode *msrN = msRecord.InquireRecordNode(GetNodeIpv4Address(thisNode));	//获取source节点的SinkRecord
//		if (msrN == NULL) {	//SinkRecord里没有这个节点的信息
//			//产生随机的发包间隔
//				double interval = RandomDoubleVauleGenerator(0.0, 0.5);
//				NS_LOG_LOGIC(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
//							<<" is for the 1st time to receive a sink check packet, prepare to reply it...");
//			Ptr<ConstantPositionMobilityModel> cpmm = thisNode->GetObject<
//					ConstantPositionMobilityModel>();
//			sourceLocation = cpmm->GetPosition();	//获取当前节点的坐标
//
//			cout<<"2:sense节点收到sink check广播包通过回调将进入ReplySinkCheck处理下一步====="<<endl;
//			Simulator::Schedule(Seconds(interval), &ReplySinkCheck, thisNode,
//					sourceLocation);
//		}else
//			NS_LOG_LOGIC(
//					TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
//					<<" is already in mobile sink record, ignore this sink check packet");
//		break;
//	}
//	}
//	return 0;
//}

/*
 * Sort sinkNodes by Ipv4Address
 */
void SortSinkNodesByAdr(NodeContainer sinkNodes) {
	cout<<"当前时间："<<TIME_STAMP<<endl;
	cout<<"sinkcheck结束以后，sink的节点数是：==="<<sinkNodes.GetN()<<"nsinks is:"<<nSinks<<endl;
	if (sinkNodes.GetN() > 0) {
		uint32_t sinkIds[nSinks];
		NodeContainer tempC;
		for (uint32_t i = 0; i < nSinks; i++) {
			sinkIds[i] = sinkNodes.Get(i)->GetId();    //sinkIds存放各个汇聚节点的id
		}
		std::stringstream ss;
		for (uint32_t i = 0; i < nSinks; i++) {
			ss << sinkIds[i] << " ";
		}
		NS_LOG_LOGIC(FUC<<"before: "<<ss.str());
		ss.str("");
		ss.clear();

		uint32_t *thisId;
		uint32_t temp;
		for (uint32_t i = nSinks - 1; i > 0; i--) {
			NS_LOG_LOGIC(std::endl<<"i="<<i);
			thisId = sinkIds;
			for (uint32_t j = i; j > 0; j--) {    //冒泡排序，把节点Id从小到达排列
				NS_LOG_LOGIC("j="<<j);
				NS_LOG_LOGIC("this="<<*thisId<<", next="<<*(thisId + 1));
				if ((*thisId) > (*(thisId + 1))) {
					NS_LOG_LOGIC("Greater, exchange");
					temp = *thisId;
					*thisId = *(thisId + 1);
					*(thisId + 1) = temp;
					thisId = thisId + 1;
				} else {
					NS_LOG_LOGIC("Smaller, skip");
					thisId = (thisId + 1);
					NS_LOG_LOGIC("thisId="<<*thisId);
				}

			}
		}
		for (uint32_t i = 0; i < nSinks; i++) {
			ss << sinkIds[i] << " ";
		}
		NS_LOG_LOGIC(FUC<<"after: "<<ss.str());
		ss.str("");
		ss.clear();

		for (uint32_t i = 0; i < nSinks; i++) {
			tempC.Add(GetNodePtrFromGlobalId(sinkIds[i], sinkNodes));   //Add里面的参数是指针？
		}
		sinkNodes = tempC;   //啥意思？
	}else
		NS_LOG_WARN(TIME_STAMP_FUC<<"WARN: nSinks<=0");


}

/*
 *sink check
 */
void MobileSinkRequestSinkCheck(NodeContainer mobileSinkNode) {
	if (doSinkCheck) {
		NS_LOG_INFO(endl<<TIME_STAMP_FUC<<"...");
		Ptr<Node> n = mobileSinkNode.Get(0);
		Ipv4Header h;
		h.SetIdentification(sinkCheck_Type);
		h.SetSource(GetNodeIpv4Address(n));
		Ptr<Packet> sinkCheckPkt = Create<Packet>(0);
		sinkCheckPkt->AddHeader(h);
		Ptr<Socket> sinkCheckSocket = Socket::CreateSocket(
				n, tid);
		sinkCheckSocket->Connect(broadcastAdr);
		sinkCheckSocket->SetAllowBroadcast(true);	//socket发送广播必须有这么一个设置
		sinkCheckSocket->Send(sinkCheckPkt);
		sinkCheckSocket->Close();  //sinkCheckReqInterval为1s
		Simulator::Schedule(Seconds(sinkCheckReqInterval), &MobileSinkRequestSinkCheck,mobileSinkNode);
	}
}

	/*
	 * Prepare sink check request for mobileSinkNode
	 */
void PrepareRequestSinkCheck(NodeContainer mobileSinkNode){
		if (doSinkCheck) {  //该标志初始化为true
				NS_LOG_DEBUG(std::endl<<TIME_STAMP_FUC<<"...");
				Simulator::Schedule(Seconds(0.0), &MobileSinkRequestSinkCheck,mobileSinkNode);
		} else {
			Simulator::Schedule(Seconds(1.0), &PrepareRequestSinkCheck,mobileSinkNode);
		}
	}

/*
 * 用于回复SinkCheck信息给mobileSinkNode
 */
void ReplySinkCheck(Ptr<Node> sinkNode, Vector location) {
	NS_LOG_LOGIC(TIME_STAMP<<GetNodeIpv4Address(sinkNode)<<" reply to mobile sink...");



	Ipv4Address sinkAdr = GetNodeIpv4Address(sinkNode);
	pktType pktType = sinkCheck_Type;
	cout<<"3:回复Mobile节点收到了广播包====="<<endl;
	//packet data
	stringstream ssData;
	ssData << location.x<< "/"<<location.y<< "/"<<location.z;
	NS_LOG_LOGIC(FUC<<GetNodeIpv4Address(sinkNode)
			<<"sink check packet stringstream: "<<ssData.str());
	stringstream dataSs(ssData.str().c_str());
	Ptr<Packet> dataPkt = Create<Packet>((uint8_t *) dataSs.str().c_str(),
			dataSs.str().length());

	//packet header
	Ipv4Header h;
	h.SetDestination(mobileSinkIf.GetAddress(0));
	h.SetSource(sinkAdr);
	h.SetIdentification(pktType);
	dataPkt->AddHeader(h);

	//send packet by socket
	InetSocketAddress mobileSinkAdr = InetSocketAddress(mobileSinkIf.GetAddress(0), 80);
	Ptr<Socket> srcSoc = Socket::CreateSocket(sinkNode, tid);
	srcSoc->Connect(mobileSinkAdr);
	srcSoc->Send(dataPkt);
//	srcSoc->Close();
}


/*
 *用于senseNodes和mobileSinkNode收到sinkCheck_Type的packet后的处理方法
 */
//static inline int32_t ProcessSinkCheckPacket(Ptr<Node> thisNode,
//		Ptr<Packet> packet, Ipv4Header h,NodeContainer sinkNodes,NodeContainer mobileSinkNode)
// int32_t ProcessSinkCheckPacket(Ptr<Node> thisNode,Ptr<Packet> packet,
//		Ipv4Header h,NodeContainer senseNodes,NodeContainer sinkNodes,NodeContainer mobileSinkNode) {
//	nodeType nType = CheckNodeType(thisNode,sinkNodes,mobileSinkNode.Get(0));	//获取节点类型
//	stringstream recvData;
//	packet->CopyData(&recvData, packet->GetSize());	//获取Packet的内容
//
//	Vector sourceLocation = Vector(0, 0, 0);
//
//	switch (nType) {
//	case mobileSink_Type: {
//		cout<<"4：回复广播包以后，Mobile节点收到广播包然后从sense中分离出sink====="<<endl;
//		NS_LOG_LOGIC(FUC<<"mobileSink_Type");
//		Ipv4Address sourceAdr = h.GetSource();	//或者Packet的source节点地址
//		AnalyzeSinkCheckPacket(recvData, sourceLocation);
//
//		NS_LOG_LOGIC(TIME_STAMP<<"Mobile sink received a sink check reply from "
//						<<sourceAdr<<", inquire it in mobile sink record...");
//		MobileSinkRecordNode *msrN = msRecord.InquireRecordNode(sourceAdr);	//获取source节点的SinkRecord
//		if (msrN != NULL) {	//照理说不可能发生，因为如果已经存在了这个SinkRecordNode，sinkNode是不会再给mobileSinkNode发送这个包的
//			return msRecord.UpdateRecordNode(sourceAdr, 0, 0);
//		} else {
//
//
//			cout<<"5:进入sink节点分类处理====="<<endl;
//			NS_LOG_INFO(TIME_STAMP<<"Ascertained "
//								<<sourceAdr
//								<<", insert it into MobileSinkRecord and color to blue");
//			if (Banim) {
//				Panim->UpdateNodeColor(GetNodePtrFromIpv4Adr(sourceAdr,sinkNodes,senseNodes,mobileSinkNode), 0, 0,
//						255);
//			}
//			sRecord.InsertRecordNode(sourceAdr);
////			remaingJ[GetNodePtrFromIpv4Adr(sourceAdr)->GetId()]=2;//sinkNodes more energy test
//			sinkNodes.Add(GetNodePtrFromIpv4Adr(sourceAdr,sinkNodes,senseNodes,mobileSinkNode));
//			cout<<"测试用：====="<<sinkNodes.GetN()<<endl;
//
//			nSinks++;
////			nSenses--;//没有必要自减，后面的RemoveFutilNodes会重新统计
//			return msRecord.InsertRecordNode(sourceAdr, sourceLocation, 0);
//		}
//		break;
//	}
//	default: {
//		NS_LOG_LOGIC(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
//				<<" received a sink check packet from mobileSinkNode, inquire it in mobile sink record...");
//		NS_LOG_LOGIC(FUC<<"default");
//		MobileSinkRecordNode *msrN = msRecord.InquireRecordNode(GetNodeIpv4Address(thisNode));	//获取source节点的SinkRecord
//		if (msrN == NULL) {	//SinkRecord里没有这个节点的信息
//			//产生随机的发包间隔
//				double interval = RandomDoubleVauleGenerator(0.0, 0.5);
//				NS_LOG_LOGIC(TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
//							<<" is for the 1st time to receive a sink check packet, prepare to reply it...");
////			NS_LOG_LOGIC(FUC<<
////									"Random interval = "<<interval<<" for " <<GetNodeIpv4Address(thisNode));
//			Ptr<ConstantPositionMobilityModel> cpmm = thisNode->GetObject<
//					ConstantPositionMobilityModel>();
//			sourceLocation = cpmm->GetPosition();	//获取当前节点的坐标
//
//			cout<<"2:sense节点收到sink check广播包通过回调将进入ReplySinkCheck处理下一步====="<<endl;
//			Simulator::Schedule(Seconds(interval), &ReplySinkCheck, thisNode,
//					sourceLocation);
//		}else
//			NS_LOG_LOGIC(
//					TIME_STAMP_FUC<<GetNodeIpv4Address(thisNode)
//					<<" is already in mobile sink record, ignore this sink check packet");
//		break;
//	}
//	}
//	return 0;
//}


}
