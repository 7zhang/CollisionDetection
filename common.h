#ifndef __COMMON_H__
#define __COMMON_H__

#include <math.h>

#define iszero(u) (fabs(u) < 1e-6)

typedef struct _vector3d
{
	float x;
	float y;
	float z;
}vector3d;

typedef struct _triangle
{
	vector3d normalvector;
	vector3d vertex1;
	vector3d vertex2;
	vector3d vertex3;
}triangle;

typedef struct _stldata
{
	char modelname[80];
	long num;
	triangle* ptriangle;
}stldata;
			
inline void vectoradd(const vector3d &v1, const vector3d &v2, vector3d &sum)
{
	sum.x = v1.x + v2.x;
	sum.y = v1.y + v2.y;
	sum.z = v1.z + v2.z;
	return;
}

inline void vectorminus(const vector3d &lhs, const vector3d &rhs, vector3d &diff)
{
	diff.x = lhs.x - rhs.x;
	diff.y = lhs.y - rhs.y;
	diff.z = lhs.z - rhs.z;
	return;
}

inline void vectordot(const vector3d &v1, const vector3d &v2, float &dot)
{
	dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	return;
}
inline void vectorcross(const vector3d &lhs, const vector3d &rhs, vector3d &cross)
{
	cross.x = lhs.y * rhs.z - lhs.z * rhs.y;
	cross.y = lhs.z * rhs.x - lhs.x * rhs.z;
	cross.z = lhs.x * rhs.y - lhs.y * rhs.x;
	return;
}

#endif
