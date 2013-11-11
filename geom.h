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

static inline void coordinatetransform(const matrix m, const vector3d *vec, const triangle *original, triangle *point)
{
	point->normalvector.x = m[0][0] * original->normalvector.x
		+ m[0][1] * original->normalvector.y + m[0][2] * original->normalvector.z;
	point->normalvector.y = m[1][0] * original->normalvector.x
		+ m[1][1] * original->normalvector.y + m[1][2] * original->normalvector.z;
	point->normalvector.z = m[2][0] * original->normalvector.x
		+ m[2][1] * original->normalvector.y + m[2][2] * original->normalvector.z;

	point->vertex1.x = m[0][0] * original->vertex1.x
		+ m[0][1] * original->vertex1.y + m[0][2] * original->vertex1.z + vec->x;
	point->vertex1.y = m[1][0] * original->vertex1.x
		+ m[1][1] * original->vertex1.y + m[1][2] * original->vertex1.z + vec->y;
	point->vertex1.z = m[2][0] * original->vertex1.x
		+ m[2][1] * original->vertex1.y + m[2][2] * original->vertex1.z + vec->z;

	point->vertex2.x = m[0][0] * original->vertex2.x
		+ m[0][1] * original->vertex2.y + m[0][2] * original->vertex2.z + vec->x;
	point->vertex2.y = m[1][0] * original->vertex2.x
		+ m[1][1] * original->vertex2.y + m[1][2] * original->vertex2.z + vec->y;
	point->vertex2.z = m[2][0] * original->vertex2.x
		+ m[2][1] * original->vertex2.y + m[2][2] * original->vertex2.z + vec->z;

	point->vertex3.x = m[0][0] * original->vertex3.x
		+ m[0][1] * original->vertex3.y + m[0][2] * original->vertex3.z + vec->x;
	point->vertex3.y = m[1][0] * original->vertex3.x
		+ m[1][1] * original->vertex3.y + m[1][2] * original->vertex3.z + vec->y;
	point->vertex3.z = m[2][0] * original->vertex3.x
		+ m[2][1] * original->vertex3.y + m[2][2] * original->vertex3.z + vec->z;
}



#endif /* _GEOM_H_ */

