#include <stdio.h>
#include <stdlib.h>
#include "loadstl.h"
#include "triangleCD.h"
#include "Transform.h"

int main(void)
{
	stldata newgun;
	stldata part;
	
	if (loadstl("robot_stl/newgun.STL", &newgun) != 0){
		printf("load newgun stl error\n");
		exit(1);
	}
		
	
	if (loadstl("robot_stl/Part.STL", &part) != 0) {
		printf("load part stl error!\n");
		exit(1);
	}
		
	printf("stlmodel1: %s, %d triangles\n", newgun.modelname, newgun.num);
	printf("stlmodel2: %s, %d tirangles\n", part.modelname, part.num);
	printf("starting collisiondetection...\n");

	JAngle robotangle(0.0, -30.00, 72.00, 0.00, 0.00, 0.00);
	JAngle exangle(0.00, 0.00, 0.00, 0.00, 0.00, 0.00);
	

	TRANS j0_trans = Transform::getTransWorldToBase(exangle);
	TRANS j1_trans, j2_trans, j3_trans, j4_trans, j5_trans, j6_trans;
	TRANS workpiece_trans, gun_trans;

	Transform::getTransBaseToJoints(robotangle, j1_trans, j2_trans, j3_trans, j4_trans, j5_trans, j6_trans);

	
	j1_trans = j0_trans * j1_trans;
	j2_trans = j0_trans * j2_trans;
	j3_trans = j0_trans * j3_trans;
	j4_trans = j0_trans * j4_trans;
	j5_trans = j0_trans * j5_trans;
	j6_trans = j0_trans * j6_trans;
	gun_trans = j6_trans * Transform::getTrans6ToGun();
	
	workpiece_trans = Transform::getTransWorldToWorkpiece(exangle);
	
	
	int i, j;
	triangle ttri1, ttri2;
	vector3d tvec1, tvec2;

	int collision = 0;

	for (i = 0; i < part.num; ++i) {
		tvec1.x = workpiece_trans.pos.dx;
		tvec1.y = workpiece_trans.pos.dy;
		tvec1.z = workpiece_trans.pos.dz;
		
		coordinatetransform(workpiece_trans.rot.mem, &tvec1, &part.ptriangle[i], &ttri1);
		
		for (j = 0; j < newgun.num; ++j) {
			tvec2.x = gun_trans.pos.dx;
			tvec2.y = gun_trans.pos.dy;
			tvec2.z = gun_trans.pos.dz;

			coordinatetransform(gun_trans.rot.mem, &tvec2, &newgun.ptriangle[j], &ttri2);
						
			if (triangleCD(&ttri1, &ttri2)) {
				collision = 1;
				break;
			}
		}

		if (collision == 1)
			break;
	}
	
	if (collision == 1) {
		printf("collision detected!\n");
	} else {
		printf("no collision\n");
	}

	free(newgun.ptriangle);
	free(part.ptriangle);
	return 0;
}













