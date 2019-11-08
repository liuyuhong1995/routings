#include <string>
#include <iomanip>
#include "ns3/mobility-module.h"
#include "tool.h"
// #include "global.h"
// #include "jump-count-table-chain.h"

using namespace std;

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Tool");

enum nodeType isSinkType(Ptr<Node> node,Ptr<Node> mobilenode){
	if(node->GetId()==mobilenode->GetId()){
		// cout<<"msink type!!!"<<endl;
		return sink_Type;
	}
	// cout<<"sense type!!!"<<endl;
	return sense_Type;
}

/*
 * 返回某个节点的Ipv4Address
 */
Ipv4Address GetNodeIpv4Address(Ptr<Node> n){
	return n->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();//1表示interface 的index，0表示address index
}

}//namespace ns3


