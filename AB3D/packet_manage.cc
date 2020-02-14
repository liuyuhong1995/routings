#include "packet_manage.h"
using namespace std;
using namespace ns3;

extern nodestate* NS;
extern NodeContainer BS;
extern NodeContainer unodes;
/*路由表*/
extern map<uint32_t,vector<neighbor>> loop_map;
extern int RPN; 
extern int TPN;
extern int maxPackets;
extern int* flag;
extern bool isTest;

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("PACKET_MANAGE");
static TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
static InetSocketAddress broadcastAdr = InetSocketAddress(Ipv4Address::GetBroadcast(), 80);


//===========================================================================
//节点广播packet
//node：发送节点  ptype： 包类型 size：大小
//===========================================================================
void BroadcastPacket(Ptr<Node> node,pktType ptype,int size){	
	Ipv4Address Adr = node->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
    Ptr<Packet> pkt = Create<Packet>(size);
	//packet header
	Ipv4Header h;
	h.SetSource(Adr);
	h.SetIdentification(ptype);
	pkt->AddHeader(h);
	//send packet by socket
    Ptr<Socket> send_Socket = Socket::CreateSocket(node, tid);
	send_Socket->Connect(broadcastAdr);
	send_Socket->SetAllowBroadcast(true);	//socket发送广播必须有这么一个设置
	send_Socket->Send(pkt);
	send_Socket->Close();
	UpdateEnergyRX(node, pkt);
	//cout<<"hello send"<<endl;
}
//===========================================================================
//单播发送packet
//
//===============================================================================
void SendPacket(Ptr<Node>thisNode,Ptr<Node>tonode,pktType ptype,int size){
    Ptr<Packet> pkt = Create<Packet>(size); 
	//packet header
	Ipv4Address fromAdr = thisNode->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
    Ipv4Address toAdr = tonode->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
	Ipv4Header h;
	h.SetSource(fromAdr);
	h.SetDestination(toAdr);
	h.SetIdentification(ptype);
	pkt->AddHeader(h);
    InetSocketAddress Toadr = InetSocketAddress(toAdr, 80);
	Ptr<Socket> send_Socket = Socket::CreateSocket(thisNode, tid);
	send_Socket->Connect(Toadr);
	send_Socket->Send(pkt);
	send_Socket->Close();
	UpdateEnergyTX(thisNode, tonode, pkt);
}

//===========================================================================
//接收回调
//
//===============================================================================
void RecvPacketCallback(Ptr<Socket> socket){
	Ptr<Packet> pkt;
	Address from;
	Ptr<Node> thisNode = socket->GetNode();//这里这个socket是received socket，不是发送的那个
	if(NS[thisNode->GetId()-1].isdead ==false){
		while ((pkt = socket->RecvFrom(from))) {
			Ptr<Packet> packet = pkt->Copy();	//	get packet
			UpdateEnergyRX(thisNode, packet);
			pktType pType;
			if (packet->GetSize() > 0) {
				Ipv4Header h;
				packet->PeekHeader(h);
				//packet->RemoveHeader(h);
				pType = pktType(h.GetIdentification());
					switch(pType){
						case data_t:{
							Simulator::Schedule(Seconds(RandomDoubleVauleGenerator(0, 0.01)), &F_DataPacket,thisNode,packet);
							break;          
						}
						case hello_t:{
							Ptr<Node>tonode = GetNodePtrFromIpv4Adr(h.GetSource(),unodes);
							SendPacket(thisNode,tonode,reqh_t,50);
							break;
						}
						case reqh_t:{
							Ptr<Node>tonode = GetNodePtrFromIpv4Adr(h.GetSource(),unodes);
							if(tonode==NULL)
								tonode=BS.Get(0);
							inerstmap(thisNode,tonode);
							break;
						}
						default:
							break;
					}
			}
		}
	}	
}
//===========================================================================
//BS接受回调
//
//===============================================================================
void BS_RecvPacketCallback(Ptr<Socket> socket){
	// cout<<"BS_RES"<<endl;
	Ptr<Packet> pkt;
	Address from;
	Ptr<Node> thisNode = socket->GetNode();//这里这个socket是received socket，不是发送的那个
	while ((pkt = socket->RecvFrom(from))) {
		Ptr<Packet> packet = pkt->Copy();	//	get packet
		pktType pType;
		if (packet->GetSize() > 0) {
			Ipv4Header h;
			packet->PeekHeader(h);
			packet->RemoveHeader(h);
			pType = pktType(h.GetIdentification());
				switch(pType){
					case data_t:{	
						Ptr<Node> sourcenode=GetNodePtrFromIpv4Adr(h.GetSource(), unodes);
						if(flag[sourcenode->GetId()]==0){
							RPN+=1;
							flag[sourcenode->GetId()]=1;
						cout<<"received a packet ,soure is "<<h.GetSource()<<endl;
						}
						break;          
					}
					case hello_t:{
						Ptr<Node>tonode = GetNodePtrFromIpv4Adr(h.GetSource(),unodes);
						SendPacket(thisNode,tonode,reqh_t,50);
						break;
					}
					case reqh_t:{
						break;
					}
					default:
						break;
				}
		}
	}
}

//==============================================================================
//邻居发现：每个节点依次广播hello包
//nodes：节点    circletime：每轮的时间
//===============================================================================
void neighbordiscover(NodeContainer nodes,double circletime){	
	cout<<"TPN: "<<TPN<<"  RPN: "<<RPN<<endl;
	if(loop_map.empty())
		cout<<"empty"<<endl;
	//printposition(loop_map);
	loop_map.clear();
	circletime*=0.1;
	for(uint32_t i=0;i<nodes.GetN();i++){
		Simulator::Schedule(Seconds(i*circletime/nodes.GetN()), &BroadcastPacket,nodes.Get(i),hello_t,50);
		if(isTest){
			cout<<"node"<<i<<" send hello packet!"<<endl;	
		}			
	}
}	

//===========================================================================
//路由表插入
//thisNode：收到回复信息的节点   nenode：邻居节点
//===============================================================================
void inerstmap(Ptr<Node>thisNode,Ptr<Node>nenode){
	vector<neighbor> cmap=loop_map[thisNode->GetId()];
	for(vector<neighbor>::iterator itr=cmap.begin();itr!=cmap.end();itr++){
		if(itr->id==nenode->GetId()){
			itr->pos=(nenode->GetObject<MobilityModel>())->GetPosition();
			loop_map[thisNode->GetId()]=cmap;
			return;
		}
	}
	//邻居表信息：节点ID，节点IP地址，节点的3D位置
	neighbor nei={nenode->GetId(),nenode->GetObject<Ipv4>()->GetAddress(1,0).GetLocal(),(nenode->GetObject<MobilityModel>())->GetPosition()};
	cmap.push_back(nei);
	loop_map[thisNode->GetId()]=cmap;
	if(isTest){
		cout<<"node"<<thisNode->GetId()<<"insert neigbhor node"<<nenode->GetId()<<endl;		
	}
}

//==============================================================================
//数据传输
//===============================================================================
void data_trans(NodeContainer nodes,double circletime){	
	uint32_t nodenum=nodes.GetN();
	circletime*=0.125;
	memset(flag,0,sizeof(int)*nodes.GetN());
	for(uint32_t i=0;i<nodenum;i++){
		Ptr<Packet> pkt = Create<Packet>(1500);
		Ipv4Header h;
		h.SetSource(nodes.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
		h.SetDestination(BS.Get(0)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
		h.SetIdentification(data_t);
		pkt->AddHeader(h);	
		Simulator::Schedule(Seconds(i*circletime/nodenum), &F_DataPacket,nodes.Get(i),pkt);
		TPN+=1;
		if(TPN==maxPackets)
			Simulator::Stop(Seconds(5.0));
		
	}
}
//==============================================================================
//最佳节点选择发送
//===============================================================================
void F_DataPacket(Ptr<Node>thisnode,Ptr<Packet>pkt){
	Vector cpos = thisnode->GetObject<MobilityModel>()->GetPosition();
	Vector dpos = BS.Get(0)->GetObject<MobilityModel>()->GetPosition();
	vector<uint32_t>BFS=BF_AB3D(cpos,dpos,loop_map[thisnode->GetId()]);	
	if(!BFS.empty()){
		Ptr<Node>tonode;
		for(auto elem:BFS){
			//cout<<"F_node: node"<<elem<<endl;
			if(elem==0)
				tonode=BS.Get(0);
			else
				tonode=unodes.Get(elem-1);
    		Ipv4Address toAdr =tonode->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();	
   	 		InetSocketAddress Toadr = InetSocketAddress(toAdr, 80);
			Ptr<Socket> send_Socket = Socket::CreateSocket(thisnode, tid);
			send_Socket->Connect(Toadr);
			send_Socket->Send(pkt);
			send_Socket->Close();
			cout<<"datapacket from node"<<thisnode->GetId()<<"--->node"<<tonode->GetId()<<endl;
			UpdateEnergyTX(thisnode, tonode, pkt);
		}
	}
	if(isTest){
		if(BFS.empty()){
			cout<<"node"<<thisnode->GetId()<<" have no available Fnode!"<<endl;
		}
		else{
			cout<<"node"<<thisnode->GetId()<<" BFnode: ";
			for(auto elem:BFS){
				cout<<" node"<<elem;
			}
			cout<<endl;
		}
	}

}



}