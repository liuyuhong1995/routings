#ifndef FINAL_RECORD_H
#define FINAL_RECORD_H

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/animation-interface.h"
#include "tool.h"
#include <string>
#include <fstream>

using namespace std;

namespace ns3 {
	void recordtable();
	void recordfile();
	void gnuplot();
}

#endif
