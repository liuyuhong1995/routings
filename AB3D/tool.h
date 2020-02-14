#ifndef TOOL_H
#define TOOL_H
#include "global.h"
#include "math.h"
#include "vector3.h"
using namespace std;
namespace ns3 {

double GetDistanceOf2Nodes(Ptr<Node> srcN, Ptr<Node> remN);
double RandomDoubleVauleGenerator(double min, double max);
Ipv4Address GetNodeIpv4Address(Ptr<Node> n);
Ptr<Node> GetNodePtrFromIpv4Adr(Ipv4Address adr,NodeContainer nodes);
void printposition(map<uint32_t,vector<neighbor>> loop_map);
uint32_t BF_AB3D_old(Vector cpos,Vector dpos,const vector<neighbor>& nebs);
vector<uint32_t> BF_AB3D(Vector cpos,Vector dpos,const vector<neighbor>& nebs);
}
#endif
