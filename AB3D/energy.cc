#include "energy.h"
#include <string>
#include <iomanip>
#include "ns3/mobility-module.h"
#include "tool.h"

using namespace std;
using namespace ns3;


static double totalConsumed;
static double ETX = 50 * 0.000000001;
static double ERX = 50 * 0.000000001;
static double Efs = 10 * 0.000000000001;
static double Emp = 0.0013 * 0.000000000001;
static double d0 = sqrt(Efs / Emp);//距离阈值
//声明外部变量
extern nodestate* NS;
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Energy");

//发送包的节点的能量更新函数
void UpdateEnergyTX(Ptr<Node> fromnode, Ptr<Node> tonode,Ptr<Packet> p) {
	if(fromnode->GetId()==0)
		return;
	uint32_t thisPktSize = p->GetSize();
	double distance = GetDistanceOf2Nodes(fromnode, tonode);
	uint32_t idf = fromnode->GetId()-1;	
	double oldValuef = NS[idf].remainingJ;
	double consumedJf;
	if(distance>=d0)
	  consumedJf = ETX*thisPktSize + Emp*thisPktSize* (pow(distance,4));
	else
	  consumedJf = ETX*thisPktSize + Efs*thisPktSize* (pow(distance,2));
    if(oldValuef>consumedJf)
	{
		NS[idf].remainingJ-=consumedJf;
		totalConsumed=totalConsumed+consumedJf;
	}
	else{  
        NS[idf].isdead = true;
		NS[idf].remainingJ=0;
		cout<<"dead";
	}
}

//接收包的节点的能量更新函数
void UpdateEnergyRX(Ptr<Node> node, Ptr<Packet> p) {
	uint32_t thisPktSize = p->GetSize();
	uint32_t id = node->GetId()-1;
	double oldValuef = NS[id].remainingJ;
	double consumedJ = ERX*thisPktSize;
    if(oldValuef>consumedJ)
	{
		NS[id].remainingJ-=consumedJ;
		totalConsumed=totalConsumed+consumedJ;
	}
	else{  
        NS[id].isdead = true;
		NS[id].remainingJ=0;
		cout<<"dead";
	}
}


}//namespace ns3
