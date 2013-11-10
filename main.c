#include <unistd.h>
#include <fcntl.h>
#include "common.h"
#include "loadstl.h"
#include "constructAABBTrees.h"
#include "get_robot_angles.h"
#include "collision_detection.h"
#include <globals.h>
typedef struct _AABBBox
{
	float xmin;
	float ymin;
	float zmin;
	float xmax;
	float ymax;
	float zmax;
	struct _AABBBox *left;
	struct _AABBBox *right;
}AABBBox;

int main(void)
{
	loadstl(robot_stl/J1.STL, &robot_triangles1);
	loadstl(robot_stl/J2.STL, &robot_triangles2);
	loadstl(robot_stl/J3.STL, &robot_triangles3);
	loadstl(robot_stl/J4.STL, &robot_triangles4);
	loadstl(robot_stl/J5.STL, &robot_triangles5);
	loadstl(robot_stl/J6.STL, &robot_triangles6);
	loadstl(robot_stl/part.STL, &ex_triangles);
	constructAABBTrees();
	get_robot_angles();
	if( collision_detection() ){
		printf("collision detected\n");
	}
	else
	{
		printf("no collision\n");
	}
	
	return 0;	
}


void constructAABBTrees(const stldata *data, AABBBox *node)
{
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	float zmin;
	float zmax;
	
	xmin = data->ptriangle[0].x;
	xmax = xmin;
	ymin = data->ptriangle[0].y;
	ymax = ymin;
	zmin = data->ptriangle[0].z;
	zmax = zmin;


	for(int i = 0; i < data->num; i++)
	{
		if(xmin > data->ptriangle[i].x)
			xmin = data->ptriangle[i].x;	
		if(xmax < data->ptriangle[i].x)
			xmax = data->ptriangle[i].x;
		if(ymin > data->ptriangle[i].x)
			ymin = data->ptriangle[i].y;	
		if(ymax < data->ptriangle[i].x)
			ymax = data->ptriangle[i].y;
		if(zmin > data->ptriangle[i].x)
			zmin = data->ptriangle[i].y;	
		if(zmax < data->ptriangle[i].x)
			zmax = data->ptriangle[i].y;
	}
	node.xmin = xmin;
	node.xmax = xmax;
	node.ymin = ymin;
	node.ymax = ymax;
	node.zmin = zmin;
	node.zmax = zmax;
}

int TestAABBAABB(AABBBox *a, AABBBox *b)
{
	if (a->xmax < b->xmin || a->xmin > b->xmax)
		return 0;
	if (a->ymax < b->ymin || a->ymin > b->ymax)
		return 0;
	if (a->zmax < b->zmin || a->zmin > b->zmax)
		return 0;
	return 1;
}
