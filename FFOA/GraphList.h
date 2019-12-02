#include <iostream>
#include <cstdio>
#include "global.h"


 using namespace ns3;
using namespace std;

struct Node1{
	int id;
	double Attract;
	struct Node1* next;
};

class GraphList
{
public:
    ~GraphList();
	void clear();
	void addSenseNodeToList(int nFrom, double Attract, int nTo);
	void addSinkNodeToList(int vFrom, double Attract, int vTo);
	void makeNodeArray();
	uint32_t  findBestAttract(uint32_t thisId,nodeType nType_1,nodeType nType_2);
	void print();
	//void clearNode();
	void addSinkSinkToList(int vFrom, double Attract, int vTo);
	void print1(uint32_t id);

public:
	int nCount;
	int nSinkCount;
	int adjcent_Matrix[22][22]={};
	Node1* sense2Sense;
	Node1* sense2Sink;
	Node1* sink2Sink;
};

//ä¿?æ”¹éƒ¨åˆ?
//********************************************************************************************************

GraphList::~GraphList()
{}


void GraphList::addSenseNodeToList(int vFrom, double Attract, int vTo)
{
	Node1* pNode1 = new Node1();
	pNode1->id = vTo;
	pNode1->Attract = Attract;
	pNode1->next = NULL;
	if (sense2Sense[vFrom].next){
		Node1* tmp = sense2Sense[vFrom].next;
		while(tmp->next){
			tmp = tmp->next;
		}
		tmp->next = pNode1;
	}else{
		sense2Sense[vFrom].next = pNode1;
	}
}

void GraphList::addSinkNodeToList(int vFrom, double Attract, int vTo)
{
	Node1* pNode1 = new Node1();
	pNode1->id = vTo;
	pNode1->Attract = Attract;
	pNode1->next = NULL;
	if (sense2Sink[vFrom].next){
		Node1* tmp = sense2Sink[vFrom].next;
		while(tmp->next){
			tmp = tmp->next;
		}
		tmp->next = pNode1;
	}else{
		sense2Sink[vFrom].next = pNode1;
	}
}

void GraphList::addSinkSinkToList(int vFrom, double Attract, int vTo)
{
	Node1* pNode1 = new Node1();
	pNode1->id = vTo;
	pNode1->Attract = Attract;
	pNode1->next = NULL;
	if (sink2Sink[vFrom-nCount].next){
		Node1* tmp = sink2Sink[vFrom-nCount].next;
		while(tmp->next){
			tmp = tmp->next;
		}
		tmp->next = pNode1;
	}else{
		sink2Sink[vFrom-nCount].next = pNode1;
	}
}





void GraphList::makeNodeArray()
{
	sense2Sense = new Node1[nCount];
	sense2Sink=new Node1[nCount];
	sink2Sink=new Node1[nSinkCount];
	for (int i = 0; i < nCount; ++i){
		sense2Sense[i].id = i;
		sense2Sense[i].next = NULL;
		sense2Sense[i].Attract=0;
		sense2Sink[i].id = i;
		sense2Sink[i].next = NULL;
		sense2Sink[i].Attract=0;
	}
	for(int j = 0; j < nSinkCount; ++j){
		sink2Sink[j].id=j;
		sink2Sink[j].next = NULL;
		sink2Sink[j].Attract=0;
	}
}





uint32_t GraphList::findBestAttract(uint32_t thisId,nodeType nType_1,nodeType nType_2){
	double maxAttract;
	uint32_t maxId;
	if(nType_1==sense_Type){
		if(nType_2==sense_Type){
			Node1* tmp = sense2Sense[thisId].next;
			if(tmp){
				maxAttract=tmp->Attract;
				maxId=tmp->id;
			    while(tmp->next){
					tmp=tmp->next;
					if(tmp->Attract>maxAttract){
						maxAttract=tmp->Attract;
						maxId=tmp->id;
					}
				}
			    return maxId;
			}
		    else
				return 100;
		}
		else{          //sense to sink
			Node1* tmp = sense2Sink[thisId].next;
			if(tmp){
				maxAttract=tmp->Attract;
				maxId=tmp->id;
			    while(tmp->next){
					tmp=tmp->next;
					if(tmp->Attract>maxAttract){
						maxAttract=tmp->Attract;
						maxId=tmp->id;
					}
				}
			    return maxId;
			}
		    else
				return 100;
		}
	}
	else{
		Node1* tmp = sink2Sink[thisId-nCount].next;
		if(tmp){
			maxAttract=tmp->Attract;
			maxId=tmp->id;
			while(tmp->next){
				tmp=tmp->next;
				if(tmp->Attract>maxAttract){
					maxAttract=tmp->Attract;
					maxId=tmp->id;
				}
			}
			return maxId;
		}
		else
			return 100;

	}	
	return 0;

}




//********************************************************************************************************



 
void GraphList::print(){
	cout<<"<------------------------------------------------------------->"<<endl;
	cout<<"sense sense neighbor"<<endl;
	for (int i = 0; i < nCount; ++i){
		Node1* tmp = sense2Sense[i].next;
		int id1 = sense2Sense[i].id;
		cout <<id1<< "->";
		while(tmp){
			cout << tmp-> id << "->"<<tmp->Attract<<"   ";
			tmp = tmp->next;
		}
		cout<<endl;
	}
	cout<<"<------------------------------------------------------------->"<<endl;
	cout<<"sense sink neighbor"<<endl;
	for (int i = 0; i < nCount; ++i){
		Node1* tmp = sense2Sink[i].next;
		int id1 = sense2Sink[i].id;
		cout <<id1<< "->";
		while(tmp){
			cout << tmp-> id << "->"<<tmp->Attract<<"   ";
			tmp = tmp->next;
		}
		cout<<endl;
	}
	cout<<"<------------------------------------------------------------->"<<endl;
	cout<<"sink sink neighbor"<<endl;
	for (int i = 0; i < nSinkCount; ++i){
		Node1* tmp = sink2Sink[i].next;
		int id1 = sink2Sink[i].id;
		cout <<id1<< "->";
		while(tmp){
			cout << tmp-> id << "->"<<tmp->Attract<<"   ";
			tmp = tmp->next;
		}
		cout<<endl;
	}
	cout<<"s3"<<endl;
}




void GraphList::clear()
{
	for (int i = 0; i < nCount; ++i){
			sense2Sense[i].next = NULL;
			sense2Sink[i].next = NULL;
		}
	for (int i = 0; i < nSinkCount; ++i)
			sink2Sink[i].next = NULL;
}




void GraphList::print1(uint32_t id){
	Node1* tmp = sink2Sink[id-nCount].next;
		int id1 = sink2Sink[id-nCount].id;
		cout <<id1<< "->";
		while(tmp){
			cout << tmp-> id << "->"<<tmp->Attract<<"   ";
			tmp = tmp->next;
		}
		cout<<endl;
}