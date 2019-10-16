#include "data_gather.h"
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
extern bool doMobileSink;

namespace ns3{
NS_LOG_COMPONENT_DEFINE("data_gather");

/*
 * 设置接收端是否开始采集数据标志
 */
void SetDataGatheringFlag(bool flag){
	doMobileSink = flag;
}


}
