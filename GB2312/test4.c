#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "volume.h"
#include "Transform.h"

#ifdef DEBUG
int volumecount = 0;
int cdcount = 0;
int last_count = 0;
int triangle_cd_count = 0;
#endif

int main(int argc, char *argv[])
{
	volumenode *left_node, *right_node;

	parameter p;
	p.max_length = atoi(argv[9]);
	p.max_triangle = atoi(argv[10]);

	left_node = cd_init(argv[1], &p);
	right_node = cd_init(argv[2], &p);
	
	clock_t start, end;
	start = clock();
	
	JAngle robotangle(atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]),
		  atof(argv[7]), atof(argv[8]));
	
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
	
	vector3d tmp1, tmp2;
	tmp1.x = newgun_trans.pos.dx;
	tmp1.y = newgun_trans.pos.dy;
	tmp1.z = newgun_trans.pos.dz;

	tmp2.x = part_trans.pos.dx;
	tmp2.y = part_trans.pos.dy;
	tmp2.z = part_trans.pos.dz;

	int i;

	for (i = 0; i < 1; ++i) {
		if (collision_detection2(right_node, part_trans.rot.mem, &tmp2, left_node, newgun_trans.rot.mem, &tmp1)) {
			printf("\ncollision detected!\n\n");
		} else {
			printf("\nno collision!\n\n");
		}
		/* collision_detection2(left_top_node, right_top_node); */
	}
	
	end = clock();
	
	printf("run %d times, time elapsed %f\n", i, (double)(end - start) / CLOCKS_PER_SEC);

#ifdef DEBUG
	printf("cdcount = %d\n", cdcount);

	printf("triangle_cd_count = %d\n", triangle_cd_count);
#endif

#ifdef SHOW
	show_tree_recur(left_node, 0);
	printf("rightnode\n");
	show_tree_recur(right_node, 0);
#endif
	
	cd_finish(left_node);
	cd_finish(right_node);

	return 0;
}

