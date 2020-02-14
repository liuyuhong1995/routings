
#ifndef GLOBAL_H
#define GLOBAL_H
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/node-container.h"
#include "ns3/mobility-module.h"
#include <iomanip>
#include <fstream>
using namespace std;
namespace ns3 {
/*数据包类型*/
enum pktType {
	data_t = 1, 
	hello_t = 2, 
	reqh_t = 3,		
};

/*邻居表内容*/
struct neighbor{
	uint32_t id;
	Ipv4Address adr;
	Vector pos;
};

/* 节点状态 */
struct nodestate
{
    bool isdead;
    double remainingJ;
};
}
#endif

