#ifndef _CD_H_
#define _CD_H_

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

typedef struct _volume {
	double xmin, xmax, ymin,
		ymax, zmin, zmax;
}volume;

typedef struct _volumenode {
	triangle *tarry;
	int tarraysize;
	int *tindex;
	int trianglenum;
	int depth;
	volume v;
	int last;

	struct _volumenode *parent;
	struct _volumenode *child1, *child2;

	double *m;
	vector3d *vector;

}volumenode;

typedef struct _cd_parameter
{
	int max_triangle;
	int max_length;
}cd_parameter;

#endif /* _CD_H_ */
