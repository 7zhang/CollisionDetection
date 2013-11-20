#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "volume.h"
#include "loadstl.h"
#include "triangleCD.h"
#include "Transform.h"

int maxdepth = 0;

#ifdef DEBUG
int volumecount = 0;
int cdcount = 0;
int last_count = 0;
int triangle_cd_count = 0;
#endif

int main(int argc, char *argv[])
{
	stldata left_model, right_model;
	
	if (loadstl(argv[1], &left_model) != 0) {
		printf("loadstl %s error!\n", argv[1]);
		exit(1);
	}
	
	if (loadstl(argv[2], &right_model) != 0) {
		printf("loadstl %s error!\n", argv[2]);
		exit(1);
	}
	
	volumenode left_top_node, right_top_node;
	
	left_top_node.tarry = left_model.ptriangle;
	left_top_node.tarraysize = left_model.num;
	left_top_node.tindex = (int *)malloc(sizeof(int) * left_model.num);

	right_top_node.tarry = right_model.ptriangle;
	right_top_node.tarraysize = right_model.num;
	right_top_node.tindex = (int *)malloc(sizeof(int) * right_model.num);
	
	int i;
	
	for (i = 0; i < left_model.num; ++i)
		left_top_node.tindex[i] = i;
	
	for (i = 0; i < right_model.num; ++i)
		right_top_node.tindex[i] = i;

	left_top_node.trianglenum = left_model.num;
	buildvolume(&left_top_node);
	left_top_node.parent = NULL;

	right_top_node.trianglenum = right_model.num;
	buildvolume(&right_top_node);
	right_top_node.parent = NULL;

#ifdef DEBUG
	volumecount++;
#endif

	JAngle robotangle(atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]), atof(argv[8]));
	/* JAngle robotangle(0.0, -23.00, 52.500, 16.00, 19.00, 0.00); */
	JAngle exangle(0.00, 0.00, 0.00, 0.00, 0.00, 0.00);
	

	TRANS j0_trans = Transform::getTransWorldToBase(exangle);
	TRANS j1_trans, j2_trans, j3_trans, j4_trans, j5_trans, j6_trans;
	TRANS part_trans, newgun_trans;

	Transform::getTransBaseToJoints(robotangle, j1_trans, j2_trans, j3_trans, j4_trans, j5_trans, j6_trans);
	newgun_trans = Transform::getTrans6ToGun();
	
	j1_trans = j0_trans * j1_trans;
	j2_trans = j0_trans * j2_trans;
	j3_trans = j0_trans * j3_trans;
	j4_trans = j0_trans * j4_trans;
	j5_trans = j0_trans * j5_trans;
	j6_trans = j0_trans * j6_trans;
	newgun_trans = j6_trans * newgun_trans;
	
	part_trans = Transform::getTransWorldToWorkpiece(exangle);
	
	maxdepth = atoi(argv[9]);

	vector3d tmp1, tmp2;
	tmp1.x = newgun_trans.pos.dx;
	tmp1.y = newgun_trans.pos.dy;
	tmp1.z = newgun_trans.pos.dz;

	tmp2.x = part_trans.pos.dx;
	tmp2.y = part_trans.pos.dy;
	tmp2.z = part_trans.pos.dz;
	
	left_top_node.m = (double *)newgun_trans.rot.mem;
	left_top_node.vector = &tmp1;
	right_top_node.m = (double *)part_trans.rot.mem;
	right_top_node.vector = &tmp2;

	if (recurbuildtree(&left_top_node, 0, newgun_trans.rot.mem, &tmp1) == -1) 
		printf("recurbuildtree error!\n");

#ifdef DEBUG
	printf("volumecount1 = %d, lastcount1 = %d\n", volumecount, last_count);
#endif

	if (recurbuildtree(&right_top_node, 0, part_trans.rot.mem, &tmp2) == -1) 
		printf("recurbuildtree error!\n");
#ifdef DEBUG	
	printf("volumecount2 = %d, lastcount2 = %d\n", volumecount, last_count);
#endif

	clock_t start, end;
	start = clock();

	for (i = 0; i < 1; ++i) {
#ifdef DEBUG
		if (collision_detection_recur(&left_top_node, &right_top_node)) {
			printf("collision detected!\n");
		} else {
			printf("no collision!\n");
		}
#else
		collision_detection_recur(&left_top_node, &right_top_node);
#endif
	}
	
	end = clock();
	
	printf("run %d times, time elapsed %f\n", i, (double)(end - start) / CLOCKS_PER_SEC);

#ifdef DEBUG
	printf("cdcount = %d\n", cdcount);

	printf("triangle_cd_count = %d\n", triangle_cd_count);
#endif

//	recurshowtree(&left_top_node, 0);
	return 0;
}

















