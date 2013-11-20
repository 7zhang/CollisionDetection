#ifndef _GEOM_H_
#define _GEOM_H_


#include <math.h>

#define ISZERO(u) (fabs(u) < 1e-6)
#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)
#define POSITIVEZERO 1e-12
#define NEGATIVEZERO -1e-12
#define MYZERO 1e-6
typedef struct _vector3d
{
	double x;
	double y;
	double z;
}vector3d;

typedef double matrix[3][3];

typedef struct _triangle
{
	vector3d normalvector;
	vector3d vertex1;
	vector3d vertex2;
	vector3d vertex3;
	unsigned short attr;
}triangle;

typedef struct _stldata
{
	char modelname[80];
	int num;
	triangle *ptriangle;
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

static inline void vectordot(const vector3d *v1, const vector3d *v2, double *dot)
{
	*dot = v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

static inline void vectorcross(const vector3d *lhs, const vector3d *rhs, vector3d *cross)
{
	cross->x = lhs->y * rhs->z - lhs->z * rhs->y;
	cross->y = lhs->z * rhs->x - lhs->x * rhs->z;
	cross->z = lhs->x * rhs->y - lhs->y * rhs->x;
}

static inline double commonpoint(vector3d *p1, vector3d *tangent1, vector3d *p2, vector3d *tangent2)
{
	vector3d diff;
	vectorminus(p2, p1, &diff);
	return (diff.x * tangent2->y - diff.y * tangent2->x)
		/ (tangent1->x * tangent2->y - tangent1->y * tangent2->x);
}

static inline void point_transform(const matrix m, const vector3d *vec, const vector3d *from, vector3d *to)
{
	to->x = m[0][0] * from->x
		+ m[0][1] * from->y + m[0][2] * from->z + vec->x;
	to->y = m[1][0] * from->x
		+ m[1][1] * from->y + m[1][2] * from->z + vec->y;
	to->z = m[2][0] * from->x
		+ m[2][1] * from->y + m[2][2] * from->z + vec->z;
}

static inline void triangle_transform(const matrix m, const vector3d *vec, const triangle *from, triangle *to)
{
	vector3d tmp = {0.0, 0.0, 0.0};

	point_transform(m, &tmp, &from->normalvector, &to->normalvector);
	point_transform(m, vec, &from->vertex1, &to->vertex1);
	point_transform(m, vec, &from->vertex2, &to->vertex2);
	point_transform(m, vec, &from->vertex3, &to->vertex3);
}

#endif /* _GEOM_H_ */











