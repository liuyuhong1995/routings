
#include "tool.h"
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Tool");

//计算两个节点间的距离
double GetDistanceOf2Nodes(Ptr<Node> srcN, Ptr<Node> remN) 
{
	Ptr<MobilityModel> srcCpmm = srcN->GetObject<MobilityModel>();
	Ptr<MobilityModel> remCpmm = remN->GetObject<MobilityModel>();
	Vector srcP = srcCpmm->GetPosition();
	Vector remP = remCpmm->GetPosition();
	return std::sqrt((srcP.x - remP.x)*(srcP.x - remP.x) + (srcP.y - remP.y)*(srcP.y - remP.y)+(srcP.z - remP.z)*(srcP.z - remP.z));
}

//获得min-max之间的一个随机数
double RandomDoubleVauleGenerator(double min, double max) {

	Ptr<UniformRandomVariable> r = CreateObject<UniformRandomVariable>();
	r->SetAttribute("Min", DoubleValue(min));
	r->SetAttribute("Max", DoubleValue(max));
	return r->GetValue();
}




// 获得节点的Ipv4地址
Ipv4Address GetNodeIpv4Address(Ptr<Node> n){
	return n->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();//1表示interface 的index，0表示address index
}
// 获得拥有该地址的节点指针
Ptr<Node> GetNodePtrFromIpv4Adr(Ipv4Address adr,NodeContainer nodes) 
{
	for (NodeContainer::Iterator i = nodes.Begin(); i!= nodes.End();i++) 
	{
		Ptr<Node> n = *i;
		if (GetNodeIpv4Address(n) == adr) {
			return n;
		}
	}
	return NULL;
}

void printposition(map<uint32_t,vector<neighbor>> loop_map)
{
	for(auto elem:loop_map){
		cout<<"node"<<elem.first<<" negihbours:"<<endl;
		for(auto sub_elem:elem.second){
			cout<<sub_elem.id<<" postion:("<<sub_elem.pos.x<<","<<sub_elem.pos.y<<","<<sub_elem.pos.z<<")"<<endl;
		}
	}
}

// AB3D地理选择
uint32_t BF_AB3D_old(Vector cpos,Vector dpos,const vector<neighbor>& nebs){	
	if(nebs.empty())
		return 0xffffffff; 	
	double distance=30;
	double cdis;
	uint32_t bestid=0;
	for(auto elem:nebs){
		cdis=std::sqrt((dpos.x - elem.pos.x)*(dpos.x - elem.pos.x) + (dpos.y - elem.pos.y)*(dpos.y - elem.pos.y)+(dpos.z - elem.pos.z)*(dpos.z - elem.pos.z));
		if(cdis<distance){
			distance=cdis;
			bestid=elem.id;
		}
	}
	return bestid;
}
//==============================================================================================
// AB3D地理选择：cpos 当前节点位置 dpos 目的节点位置 nebs 当前节点的邻居表
//==============================================================================================
vector<uint32_t> BF_AB3D(Vector cpos,Vector dpos,const vector<neighbor>& nebs){	
	vector<uint32_t>Fnode;
	if(nebs.empty())
		return Fnode; 	
	double distance=std::sqrt((dpos.x - cpos.x)*(dpos.x - cpos.x) + (dpos.y - cpos.y)*(dpos.y - cpos.y)+(dpos.z - cpos.z)*(dpos.z - cpos.z));
	double ndis[nebs.size()];
	double cdis;
	uint32_t bestid=0;
//找到离目标节点最近的邻居
	for(uint32_t i=0;i<nebs.size();i++){
		if(nebs[i].id==0){
			Fnode.push_back(0);
			return Fnode;
		}
		cdis=std::sqrt((dpos.x - nebs[i].pos.x)*(dpos.x - nebs[i].pos.x) + (dpos.y - nebs[i].pos.y)*(dpos.y - nebs[i].pos.y)+(dpos.z - nebs[i].pos.z)*(dpos.z - nebs[i].pos.z));
		ndis[i]=cdis;
		if(cdis<distance){
			distance=cdis;
			bestid=nebs[i].id;
		}
	}
	if(bestid==0)
		return Fnode;
//筛选最佳转发节点
	else{
		Fnode.push_back(bestid);
		uint32_t bestup=0;
		uint32_t bestdown=0;
		double mindup=std::sqrt((dpos.x - cpos.x)*(dpos.x - cpos.x) + (dpos.y - cpos.y)*(dpos.y - cpos.y)+(dpos.z - cpos.z)*(dpos.z - cpos.z));
		double minddown=mindup;
		Plane pl(cpos,dpos,nebs[bestid].pos);//当前节点 目的节点 最近节点形成的平面
		for(uint32_t i=0;i<nebs.size();i++){
			if(nebs[i].id!=bestid){
				if(pl.classifyPoint(nebs[i].pos)==0&&ndis[i]<mindup){
					bestup=nebs[i].id;
					mindup=ndis[i];//转发节点up
				}
				if(pl.classifyPoint(nebs[i].pos)==1&&ndis[i]<minddown){
					bestdown=nebs[i].id;
					minddown=ndis[i];//转发节点down
				}
			}
		}
		if(bestup!=0)
			Fnode.push_back(bestup);
		if(bestdown!=0)
			Fnode.push_back(bestdown);
		return Fnode;
	}
}

}
