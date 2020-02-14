#include <iostream>
#include <cstdio>
 
using namespace std;
 
// ��
struct Edge{
	int id;
	double weight;
	struct Edge* next;
};

/*struct Node1{
	int id;
	int weight;
	struct Node1* next;
};*/
 
// 
struct Vertex{
	int id;
	struct Edge* next; // next我的邻居
	//struct Node1* nodeNext; // 我的下一跳节点
};
 
// ����ͼ
class GraphList
{
public:
	~GraphList();
	void createGraph();
	void printGraph();
	void clear();
	void addEdgeToList(int vFrom, double weight, int vTo);
	void makeVertexArray();
	void inputVertexCount();
	void print();
	void clearNode(uint32_t id);
	void addNewEdgeToList(int vFrom, double weight, int vTo);
	bool isExist(int vFrom, int vTo);
	void printNoteNeighbor(uint32_t id);

public:
	int m_vCount;
	int adjcent_Matrix[22][22]={};
	Vertex* m_vVertex;
};

GraphList::~GraphList(){
	for (int i = 0; i < m_vCount; ++i){
		Edge* tmp = m_vVertex[i].next;
		Edge* edge = NULL;
		while(tmp){
			edge = tmp;
			tmp = tmp->next;
			delete edge;
			edge = NULL;
		}
	}
	delete[] m_vVertex;
}
 
void GraphList::inputVertexCount()
{
	cout << "please input count of vertex:";
	cin >> m_vCount;
}

void GraphList::clear()
{
	cout << "begin clear"<<endl;
	for (int i = 0; i < m_vCount; ++i){
		Edge* tmp = m_vVertex[i].next;
		Edge* next = NULL;
		while(tmp){//
			next = tmp->next;
			delete tmp;
			tmp = NULL;
			tmp = next;
		}
	}
	delete[] m_vVertex;
	m_vCount = 0;
	//
	int row = sizeof(adjcent_Matrix)/sizeof(adjcent_Matrix[0]);
	int col = sizeof(adjcent_Matrix[0])/sizeof(adjcent_Matrix[0][0]); 
	for(int i=0;i<row;i++){
		for(int j=0;j<col;j++){
			adjcent_Matrix[i][j]=0;
		}
	}
	cout << "clear done"<<endl;
}

void GraphList::clearNode(uint32_t id)
{
	int _id = (int)id;
	for (int i = 0; i < m_vCount; ++i){
//		Vertex* vertex = m_vVertex[i];
		if(m_vVertex[i].id != _id){
			continue;
		}else{
			m_vVertex[i].next = NULL;
//			Edge* tmp = m_vVertex[i].next;
//			Edge* next = NULL;
//			while(tmp){//
//				next = tmp->next;
//				delete tmp;
//				//tmp = NULL;
//				tmp = next;
//			}
			break;
		}
	}
}

void GraphList::printNoteNeighbor(uint32_t id)
{
	int _id = (int)id;
	for (int i = 0; i < m_vCount; ++i){
		if(m_vVertex[i].id != _id){
			continue;
		}else{
			Edge* tmp = m_vVertex[i].next;
			cout << "list:" << m_vVertex[i].id << "->";
			while(tmp){
				cout << "(" << tmp->weight << ")";
				cout << tmp->id << "->";
				tmp = tmp->next;
			}
			cout << "NULL" << endl;
			break;
		}
	}
}
 
void GraphList::makeVertexArray()
{
	m_vVertex = new Vertex[m_vCount];
	// ��ʼ��
	for (int i = 0; i < m_vCount; ++i){
		m_vVertex[i].id = i;
		m_vVertex[i].next = NULL;
	}
}
 

 
void GraphList::addEdgeToList(int vFrom, double weight, int vTo)
{
	Edge* edge = new Edge();
	edge->id = vTo;
	edge->weight = weight;
	edge->next = NULL;
	if (m_vVertex[vFrom].next){
		Edge* tmp = m_vVertex[vFrom].next;
		while(tmp->next){
			tmp = tmp->next;
		}
		tmp->next = edge;
	}else{
		m_vVertex[vFrom].next = edge;
	}
}



 
void GraphList::printGraph()
{
	for (int i = 0; i < m_vCount; ++i){
		Edge* tmp = m_vVertex[i].next;
		cout << "list:" << m_vVertex[i].id << "->";
		while(tmp){
			cout << "(" << tmp->weight << ")";
			cout << tmp->id << "->";
			tmp = tmp->next;
		}
		cout << "NULL" << endl;
	}
}




void GraphList::print(){
	cout<<"<------------------------------------------------------------->"<<endl;
	for (int i = 0; i < m_vCount; ++i){
		Edge* tmp = m_vVertex[i].next;
		int curId = m_vVertex[i].id;
		cout << curId << "->";
		while(tmp){
			cout << tmp->id << "(" << tmp->weight<< ") ->";
			tmp = tmp->next;
		}
		cout<<endl;
	}
}

void GraphList::addNewEdgeToList(int vFrom, double weight, int vTo)
{	
	if(!isExist(vFrom,vTo)){
		Edge* edge = new Edge();
		edge->id = vTo;
		edge->weight = weight;
		edge->next = NULL;
		if (m_vVertex[vFrom].next){
			Edge* tmp = m_vVertex[vFrom].next;
			while(tmp->next){
				tmp = tmp->next;
			}
			tmp->next = edge;
		}else{
			m_vVertex[vFrom].next = edge;
		}
	}else{
			Edge* tmp = m_vVertex[vFrom].next;
			while(tmp){
				if(tmp->id == vTo)		
					break;
				tmp = tmp->next;
			}
			tmp->weight=weight;
	}
}






bool GraphList::isExist(int vFrom, int vTo){
	bool res = false;
	if (m_vVertex[vFrom].next){// 
		Edge* tmp = m_vVertex[vFrom].next;
		while(tmp){
			if(tmp->id == vTo){
				res = true;
				break;
			}
			tmp = tmp->next;
		}
	}
	return res; 
}


 
// **************************************************************************
// ���̿���
// **************************************************************************
void GraphList::createGraph()
{
	inputVertexCount();
	makeVertexArray();
	printGraph();
}

