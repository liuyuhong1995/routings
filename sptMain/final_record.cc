#include "final_record.h"
#include <string>
#include <iomanip>
#include "ns3/mobility-module.h"
#include "tool.h"
#include "mobile-sink-record.h"
#include "sink-record.h"
#include "jump-count-table-chain.h"
#include "sense-sink-match-table.h"
using namespace std;
using namespace ns3;


//extern string thisSimPath;//simulation path
//extern string exeName;
////声明外部变量
//extern double totalConsumed;
//extern uint32_t nDrain;
extern MobileSinkRecord msRecord;
extern SinkRecord sRecord;
extern string thisSimPath;
extern SenseSinkMatchTable *PssmTable;
extern string exeName;
extern JumpCountTableChain jctChain;
extern string algType;
extern bool rewrite ;
extern string timeStr;
extern double simRunTime ;
extern double RngRun;
extern algorithmType aType;
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
NS_LOG_COMPONENT_DEFINE("TYPEMSG");     //括号里为什么不是“final_record”？


/**Record some tables when the simulation finished**/
void recordtable(){
	stringstream ssi;
	ssi<<thisSimPath<< exeName<< "-mobile.record";
	msRecord.ListMobileSinkRecord(ssi.str().c_str());
	ssi.clear();
	ssi.str("");

	ssi<<thisSimPath<< exeName << "-final.route";
	jctChain.ListJumpCountTableChain(ssi.str().c_str());
	ssi.clear();
	ssi.str("");

	ssi<<thisSimPath<< exeName << "-sink.record";
	sRecord.ListSinkRecord(ssi.str().c_str());
	ssi.clear();
	ssi.str("");

	ssi<<thisSimPath << exeName << "-"<<algType << ".route";
	if (PssmTable) {
		PssmTable->ListTable(ssi.str().c_str());
	}
	ssi.clear();
	ssi.str("");
}

/**Log when the simulation finished simulation**/
void recordfile(){
	stringstream ss;
	stringstream si;
	ofstream of;
	if (rewrite) {
		of.open("/home/wsn/sim_temp/result-pp2.record");
	} else {
		of.open("/home/wsn/sim_temp/record_files/result-pp2.record", ios::app);   //后面什么意思？
	}

	NS_LOG_UNCOND("");
	si<<"Date&Time: "<<timeStr<<", Runtime="<<simRunTime
			<<"s, RngRun=" << RngRun<<std::endl;
	ss<<si.str();
	si.str("");
	si.clear();

	if (aType == gene_Algor ||aType == my_gene_Algor) {
		si << "Initial Energy=" << initialJ << "J, Nodes Number=" << nNodes
						<< "(" << nSenses << "/" << nSinks << "), Algorithm Type="
						<< algType <<  std::endl;
	} else {
		si << "Initial Energy=" << initialJ << "J, Nodes Number=" << nNodes
				<< "(" << nSenses << "/" << nSinks << "), Algorithm Type="
				<< algType<<", data interval="<<dataInterval<< std::endl;
	}
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
//	if (aType != my_gene_Algor) {
//		si << "Max Traffic=" << maxTrafficCounts << "(Node[" << maxTrafficNodeId
//				<< "])" << "Total Traffic="
//				<< PssmTable->GetTrafficTable()->GetTotalTrafficCounts();
//	}
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
void gnuplot (){     //这个有啥作用呢？
	string simFolderName="/home/wsn/sim_temp/";
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
