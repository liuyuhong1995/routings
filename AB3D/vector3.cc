#include "vector3.h"
namespace ns3{
void normalize(Vector v){	
	double len = sqrt(v.x*v.x+v.y*v.y+v.z*v.z);		
	len = 1/len; 	
	v.x *= len;	
	v.y *= len;	
	v.z *= len;
}
 
Vector crossProduct(const Vector v1,const Vector v2){	
	return Vector(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z,	v1.x * v2.y - v1.y * v2.x);
}

//=====================================================================================================================
//根据三个点确定平面
//=====================================================================================================================
Plane::Plane(Vector v1,Vector v2,Vector v3){
	Vector t1(v2.x-v1.x,v2.y-v1.y,v2.z-v1.z);
	
	Vector t2(v3.x-v1.x,v3.y-v1.y,v3.z-v1.z);
	Vector n=crossProduct(t1,t2);
	normalize(n);
	x = n.x;
	y = n.y;
	z = n.z;
	dis = -(x*v1.x + y*v1.y + z*v1.z);	
}
//====================================================================================================================
//根据确定空间中一点位于一平面的哪侧
//====================================================================================================================
int Plane::classifyPoint(Vector v){
	double dist=x*v.x+y*v.y+z*v.z+dis;
	if(dist>0)
		return 0;
	else
		return 1;
}

}