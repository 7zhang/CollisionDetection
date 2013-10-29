#ifndef __LOADSTL_H__
#define __LOADSTL_H__
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

int loadstl(const char *path, stldata *pstldata);

#endif
