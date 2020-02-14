#ifndef VECTOR3_H
#define VECTOR3_H
#include "global.h"
#include "math.h"
using namespace std;
namespace ns3{

//归一化向量
void normalize(Vector v);

//向量 v1xv2
Vector crossProduct(const Vector v1,const Vector v2);
//平面类
class Plane{
	public:
		double x,y,z;
		double dis;
		enum PLAN_STATES
		{
			PLANT_FRONT,
			PLANT_BACK,
			PLANT_ON_PLANE,
		};		
		Plane():x(0),y(0),z(0),dis(0){}
		Plane(double x1,double y1,double z1,double dis1):x(x1),y(y1),z(z1),dis(dis1){}
		//创建三个点确定的平面
		Plane(Vector v1,Vector v2,Vector v3);	
		//	判断一点在平面的哪侧
		int classifyPoint(Vector v);
};

}
#endif