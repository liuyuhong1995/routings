#ifndef DATA_GATHER_H
#define DATA_GATHER_H

#include "ns3/core-module.h"
#include <iomanip>
#include <string>
#include "ns3/internet-module.h"
#include "ns3/animation-interface.h"
#include "energy.h"
#include "tool.h"

#include <fstream>


using namespace std;
using namespace ns3;
namespace ns3 {
	void SetDataGatheringFlag(bool flag);
}
#endif
