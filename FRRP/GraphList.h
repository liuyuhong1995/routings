#include <iostream>
#include <cstdio>
 
using namespace std;
 
// ��
struct Edge{
	int id;
	double weight;
	struct Edge* next;
};

struct Node1{
	int id;
	int weight;
	struct Node1* next;
};
 
// 
struct Vertex{
	int id;
	struct Edge* next; // next我的邻居
	struct Node1* nodeNext; // 我的下一跳节点
};
 
// ����ͼ
class GraphList
{
public:
	~GraphList();
	void createGraph();
	void printGraph();
	void updateArray();
	void clear();
	void addEdgeToList(int vFrom, double weight, int vTo);
	void inputVertexCount();
	void makeVertexArray();
	void print();
	void clearNode(uint32_t id);
	void addNewEdgeToList(int vFrom, double weight, int vTo);
	bool isExist(int vFrom, int vTo);
	void printNoteNeighbor(uint32_t id);
	void printRoute();
	void createRoute();
 
private:
	void inputEdgeCount();
	void inputEdgeInfo();
	

public:
	int m_vCount;
	int m_eCount;
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
	m_eCount = 0;
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
		m_vVertex[i].nodeNext = NULL;
	}
}
 
void GraphList::inputEdgeCount()
{
	cout << "please input count of edge:";
	cin >> m_eCount;
}
 
void GraphList::inputEdgeInfo()
{
	cout << "please input edge information:" << endl;
	for (int i = 0; i < m_eCount; ++i){
		cout << "the edge " << i << ":" << endl;
 
		// ���
		int from = 0;
		cout << "From: ";
		cin >> from;
		
		// Ȩֵ
		double weight = 0;
//		cout << "Weight:";
//		cin >> weight;
 
		// �յ�
		int to = 0;
		cout << "To: ";
		cin >> to;
		cout << endl;
 
		addEdgeToList(from, weight, to);
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


void GraphList::updateArray()
{
	//更新,先清空,然后重新push
	for(int i=0;i<22;i++){
		for(int j=0;j<22;j++){
			adjcent_Matrix[i][j]=0;
		}
	}
	// 
	for (int i = 0; i < m_vCount; ++i){
		int curId = m_vVertex[i].id;
		Node1* edge = m_vVertex[i].nodeNext;
		while(edge){
			int id = edge->id;
			//cout<<"id:"<<id<<endl;
			adjcent_Matrix[curId][id]=1;
			edge = edge->next;
		}
	}
	// cout<<"updateArray====================="<<endl;
	// for(int i=0;i<22;i++){
	// 	for(int j=0;j<22;j++){
	// 		cout<<adjcent_Matrix[i][j]<<" ";
	// 	}
	// 	cout<<endl;
	// }
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

void GraphList::printRoute()
{
	cout<<"printRoute()--->"<<endl;
	for (int i = 0; i < m_vCount; ++i){
		Node1* tmp = m_vVertex[i].nodeNext;
		cout << "list:" << m_vVertex[i].id << "->";
		while(tmp){
			// cout << "(" << tmp->weight << ")";
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
	}

}

void GraphList::createRoute()
{
	cout<<"createRoute"<<endl;
	for (int i = 0; i < m_vCount; ++i){
		Node1* edge = m_vVertex[i].nodeNext;
		if(edge != NULL){
			edge->next = NULL;
			edge = NULL;
		}

	}
	for (int i = 0; i < m_vCount; ++i){
		int curId = m_vVertex[i].id;
		Edge* edge = m_vVertex[i].next;
		int minWeight=1024;
		int idTmp = -1;
		while(edge){
			int id = edge->id;
			int weight = edge->weight;
			if(weight<minWeight){
				minWeight = weight;
				idTmp = id;
			}
			edge = edge->next;	
		}
		// 如果有下一跳节点
		if(minWeight!=1024 && idTmp!=-1){
			Node1* node = new Node1();
			node->id = idTmp;
			node->weight = minWeight;
			node->next = NULL;
		    //cout<<"添加节点的id： "<<idTmp<<"  weight:"<<minWeight<<endl;
			// 如果当前节点的nodeNext已经存在
	/*		if (m_vVertex[curId].nodeNext){
				Node1* tmp = m_vVertex[curId].nodeNext;
				while(tmp->next){
					tmp = tmp->next;
				}
				tmp->next = node;
			}else{
				m_vVertex[curId].nodeNext = node;
			}		 */	
			
			m_vVertex[curId].nodeNext = node;
		}
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
	inputEdgeCount();
	inputEdgeInfo();
	printGraph();
	updateArray();
}

