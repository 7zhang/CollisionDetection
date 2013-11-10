#ifndef _GEOM_H_
#define _GEOM_H_


#include <math.h>

#define ISZERO(u) (fabs(u) < 1e-6)
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)
#define POSITIVEZERO 10e-12
#define NEGATIVEZERO -10e-12
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
			
static inline void vectoradd(const vector3d *v1, const vector3d *v2, vector3d *sum)
{
	sum->x = v1->x + v2->x;
	sum->y = v1->y + v2->y;
	sum->z = v1->z + v2->z;
}

static inline void vectorminus(const vector3d *lhs, const vector3d *rhs, vector3d *diff)
{
	diff->x = lhs->x - rhs->x;
	diff->y = lhs->y - rhs->y;
	diff->z = lhs->z - rhs->z;
}

static inline void vectordot(const vector3d *v1, const vector3d *v2, float *dot)
{
	*dot = v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

static inline void vectorcross(const vector3d *lhs, const vector3d *rhs, vector3d *cross)
{
	cross->x = lhs->y * rhs->z - lhs->z * rhs->y;
	cross->y = lhs->z * rhs->x - lhs->x * rhs->z;
	cross->z = lhs->x * rhs->y - lhs->y * rhs->x;
}

static inline float commonpoint(vector3d *p1, vector3d *tangent1, vector3d *p2, vector3d *tangent2)
{
	vector3d diff;
	vectorminus(p2, p1, &diff);
	return (diff.x * tangent2->y - diff.y * tangent2->x)
		/ (tangent1->x * tangent2->y - tangent1->y * tangent2->x);
}
#endif /* _GEOM_H_ */



















