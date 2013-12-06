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

	cd_parameter p;
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
	
	/* newgun_trans.rot.mem[0][0] = -0.31028574744121573; */
	/* newgun_trans.rot.mem[0][1] = -0.76055620150283443; */
	/* newgun_trans.rot.mem[0][2] = 0.57033062278859459; */
	/* newgun_trans.rot.mem[1][0] = -0.20326345994589520; */
	/* newgun_trans.rot.mem[1][1] = 0.63914569443991687; */
	/* newgun_trans.rot.mem[1][2] = 0.74173900202816612; */
	/* newgun_trans.rot.mem[2][0] = -0.92865855985161161; */
	/* newgun_trans.rot.mem[2][1] = 0.11422366494950395; */
	/* newgun_trans.rot.mem[2][2] = -0.35291108452389802; */
	/* /\* newgun_trans.pos.dx = -600.00000000000000; *\/ */
	/* /\* newgun_trans.pos.dy = 429.99999998934874; *\/ */
	/* /\* newgun_trans.pos.dz = 2087.0000000021946; *\/ */
	
	/* newgun_trans.pos.dx = 94.771302128496586; */
	/* newgun_trans.pos.dy = -505.20287976357167; */
	/* newgun_trans.pos.dz = 1222.0426991063437; */

	newgun_trans.rot.mem[0][0] = -0.036801385144649992;
	newgun_trans.rot.mem[0][1] = -0.74802231932451657;
	newgun_trans.rot.mem[0][2] = 0.66265244875711904;
	newgun_trans.rot.mem[1][0] = 0.73624785273584337;
	newgun_trans.rot.mem[1][1] = 0.42807339281869838;
	newgun_trans.rot.mem[1][2] = 0.52411093263024766;
	newgun_trans.rot.mem[2][0] = -0.67571055740849195;
	newgun_trans.rot.mem[2][1] = 0.50716445079782491;
	newgun_trans.rot.mem[2][2] = 0.53497613260186283;
	
	newgun_trans.pos.dx = 159.55751427696521;
	newgun_trans.pos.dy = 700.79742899056282;
	newgun_trans.pos.dz = 2790.4503015644741;

	part_trans.rot.mem[0][0] =-5.1036340138841535e-012;
	part_trans.rot.mem[0][1] = 1.0000000000000000;
	part_trans.rot.mem[0][2] = -5.1036340138841535e-012;
	part_trans.rot.mem[1][0] = -1.0000000000000000;
	part_trans.rot.mem[1][1] = -5.1036340138581062e-012;
	part_trans.rot.mem[1][2] = -5.1036340139102000e-012;
	part_trans.rot.mem[2][0] = -5.1036340138841527e-012;
	part_trans.rot.mem[2][1] = -5.1036340139102008e-012;
	part_trans.rot.mem[2][2] = 1.0000000000000000;


	part_trans.pos.dx = -600.0000000000;
	part_trans.pos.dy = 429.99999998934874;
	part_trans.pos.dz = 2087.0000000021946;

	/* TRANS gun; */
	/* TRANS part; */
	
	/* newgun_trans.rot.mem[0][0] = -0.31028574744121573; */
	/* newgun_trans.rot.mem[0][1] = -0.76055620150283443; */
	/* newgun_trans.rot.mem[0][2] = 0.57033062278859459; */
	/* newgun_trans.rot.mem[1][0] = -0.20326345994589520; */
	/* newgun_trans.rot.mem[1][1] = 0.63914569443991687; */
	/* newgun_trans.rot.mem[1][2] = 0.74173900202816612; */
	/* newgun_trans.rot.mem[2][0] = -0.92865855985161161; */
	/* newgun_trans.rot.mem[2][1] = 0.11422366494950395; */
	/* newgun_trans.rot.mem[2][2] = -0.35291108452389802; */
	/* newgun_trans.pos.dx = -600.00000000000000; */
	/* newgun_trans.pos.dy = 429.99999998934874; */
	/* newgun_trans.pos.dz = 2087.0000000021946; */

	/* part_trans.rot.mem[0][0] =-5.1036340138841535e-012; */
	/* part_trans.rot.mem[0][1] = 1.0000000000000000; */
	/* part_trans.rot.mem[0][2] = -5.1036340138841535e-012; */
	/* part_trans.rot.mem[1][0] = -1.0000000000000000; */
	/* part_trans.rot.mem[1][1] = -5.1036340138581062e-012; */
	/* part_trans.rot.mem[1][2] = -5.1036340139102000e-012; */
	/* part_trans.rot.mem[2][0] = -5.1036340138841527e-012; */
	/* part_trans.rot.mem[2][1] = -5.1036340139102008e-012; */
	/* part_trans.rot.mem[2][2] = 1.0000000000000000; */
	/* part_trans.pos.dx = 94.771302128496586; */
	/* part_trans.pos.dy = -505.20287976357167; */
	/* part_trans.pos.dz = 1222.0426991063437; */
		

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

