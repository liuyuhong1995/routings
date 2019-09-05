#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h" 
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/ns2-mobility-helper.h"


using namespace ns3;
int main(int argc, char **argv)
{
  uint32_t size=30;
  double totalTime=100000;

  NodeContainer nodes;
  nodes.Create(size);

  std::string traceFile = "scratch/RPGM/test5.ns_movements";
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
  ns2.Install ();
  
  NodeContainer relay;
  relay.Create(1);
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(relay);
  relay.Get(0)->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(100, 100, 0));

  AnimationInterface anim ("/home/zlb/sim_temp/RPGM/output.xml");
    
  Simulator::Stop (Seconds(totalTime));
  Simulator::Run ();
  Simulator::Destroy ();
  
  std::cout<<"\n\n***** OUTPUT *****\n\n";
  std::cout<<"Finished !"<<std::endl;
}