#ifndef TOOL_H
#define TOOL_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/animation-interface.h"
#include "tool.h"

#include <fstream>

namespace ns3 {
	#define TIME_STAMP Simulator::Now().GetSeconds()<<": "
	#define TIME_STAMP_FUC Simulator::Now().GetSeconds()<<": ("<<__FUNCTION__<<") "
	#define FUC __FUNCTION__<<": "
	#define MAX_NUM 255
	/*
	 * 定义Node类型的枚举
	 */
	enum nodeType {
		sense_Type = 0,
		sink_Type = 1,
		// mobileSink_Type = 2,
	} ;

	Ipv4Address GetNodeIpv4Address(Ptr<Node> n);
	enum nodeType isSinkType(Ptr<Node> node,Ptr<Node> mobilenode);
}

#endif
