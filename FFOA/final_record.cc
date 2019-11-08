#include "final_record.h"
#include <string>
#include <iomanip>
#include "ns3/mobility-module.h"
#include "global.h"


using namespace std;
using namespace ns3;


//extern string thisSimPath;//simulation path
//extern string exeName;
////声明外部变量
//extern double totalConsumed;
//extern uint32_t nDrain;

extern string thisSimPath;
extern string exeName;
extern string algType;
extern bool rewrite ;
extern string timeStr;
extern double simRunTime ;
extern double RngRun;
extern double initialJ;
extern uint32_t nNodes;
extern uint32_t nSinks;
extern uint32_t nSenses;
//extern uint32_t iterationCounts;
extern double dataInterval;
extern uint32_t totalBytesGathered;
extern double totalEnergyConsumed ;
extern double networkLifetime;
extern double dataGatherPercentage;
extern double firstDrainedNodeTime;
extern uint32_t firstDrainedNodeId;
extern uint32_t firstDrainedSinkId;
extern double EnergyConsumeRate;
extern uint32_t totalJumps;
extern double sinkVariance;
extern uint32_t nDrain;


namespace ns3 {
NS_LOG_COMPONENT_DEFINE("TYPEMSG");


/**Record some tables when the simulation finished**/
void recordtable(){
	stringstream ssi;
	// ssi<<thisSimPath<< exeName<< "-mobile.record";
	// msRecord.ListMobileSinkRecord(ssi.str().c_str());
	// ssi.clear();
	// ssi.str("");

	// ssi<<thisSimPath<< exeName << "-final.route";
	// jctChain.ListJumpCountTableChain(ssi.str().c_str());
	// ssi.clear();
	// ssi.str("");

	// ssi<<thisSimPath<< exeName << "-sink.record";
	// sRecord.ListSinkRecord(ssi.str().c_str());
	// ssi.clear();
	// ssi.str("");

	// ssi<<thisSimPath << exeName << "-"<<algType << ".route";

	ssi.clear();
	ssi.str("");
}

/**Log when the simulation finished simulation**/
void recordfile(){
	stringstream ss;
	stringstream si;
	ofstream of;
	if (rewrite) {
		of.open("/home/zlb/workspace/ns3/result-pp2.record");
	} else {
		of.open("/home/zlb/workspace/ns3/result-pp2.record", ios::app);
	}

	NS_LOG_UNCOND("");
	si<<"Date&Time: "<<timeStr<<", Runtime="<<simRunTime
			<<"s, RngRun=" << RngRun<<std::endl;
	ss<<si.str();
	si.str("");
	si.clear();


	ss<<si.str();
	si.str("");
	si.clear();

	si << "Network Lifetime=" << networkLifetime<<" s"<<std::endl
			<<"First node drain time="<<firstDrainedNodeTime<<" s ("<<firstDrainedNodeId<<")"<< std::endl
			<< "Total Bytes="<< totalBytesGathered*1.0/1000 << " KB" "(" << dataGatherPercentage * 100<< "%)" << std::endl
			<< "totalEnergyConsumed="<< totalEnergyConsumed << " J"<<std::endl
			<<"TEE="<< EnergyConsumeRate << " J/s" << std::endl
			<< "EUE="	<< (totalBytesGathered/1000)/totalEnergyConsumed  << " KB/J"<< std::endl
			<< "Total Jumps=" << totalJumps << ", sinkVariance="<< sinkVariance << ", sinkNode[" << firstDrainedSinkId
			<< "] drained" << ", nDrain=" << nDrain << std::endl << std::endl;

	ss<<si.str();
	si.str("");
	si.clear();
	NS_LOG_UNCOND(ss.str().c_str());
	of<<ss.str().c_str();
	of.close();

	si<<thisSimPath<<exeName<<".result";
	of.open(si.str().c_str());
	of<<ss.str().c_str();
	of.close();
	si.str("");
	si.clear();

}
void gnuplot (){
	string simFolderName="/home/zlb/sim_temp/";
	stringstream si;
	ofstream of;
	si<<thisSimPath<<exeName<<"-dynamic-DGP.gnu";
	of.open(si.str().c_str());
	si.str("");
	si.clear();
	si<<simFolderName<<"template.gnu";
	ifstream inf(si.str().c_str(),ios::in);
	si.str("");
	si.clear();

	if(!inf.is_open()){
		NS_LOG_WARN("gnuplot template open fail");
		return;
	}
	string line;
	while (getline(inf, line)) {
		of<<line<<endl;
	}
	inf.close();
	of.close();
}
}
