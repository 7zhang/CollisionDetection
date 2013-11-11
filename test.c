#include <stdio.h>
#include <stdlib.h>
#include "loadstl.h"
#include "triangleCD.h"
#include "Transform.h"

int main(void)
{
	stldata newgun;
	stldata part;
	
	if (loadstl("robot_stl/newgun.STL", &newgun) != 0)
		printf("load newgun stl error\n");
	
	if (loadstl("robot_stl/J4.STL", &part) != 0)
		printf("load part stl error!\n");

	printf("stlmodel1: %s, %d triangles\n", newgun.modelname, newgun.num);
	printf("stlmodel2: %s, %d tirangles\n", part.modelname, part.num);
	printf("starting collisiondetection...\n");

	JAngle robotangle(0.0, -90.00, 90.00, 0.00, 0.00, 0.00);
	JAngle exangle(0.00, 0.00, 0.00, 0.00, 0.00, 0.00);
	
	TRANS j0_trans = Transform::getTransWorldToBase(exangle);
	TRANS j1_trans, j2_trans, j3_trans, j4_trans, j5_trans, j6_trans;
	Transform::getTransBaseToJoints(robotangle, j1_trans, j2_trans, j3_trans, j4_trans, j5_trans, j6_trans);
	TRANS gun_trans = Transform::getTrans6ToGun();
	
	j1_trans = j0_trans * j1_trans;
	j2_trans = j0_trans * j2_trans;
	j3_trans = j0_trans * j3_trans;
	j4_trans = j0_trans * j4_trans;
	j5_trans = j0_trans * j5_trans;
	j6_trans = j0_trans * j6_trans;
	gun_trans = j6_trans * gun_trans;
	
	TRANS workpiece_trans = Transform::getTransWorldToWorkpiece(exangle);
	
	
	int i, j;
	triangle ttri1, ttri2;
	vector3d tvec1, tvec2;

	int collision = 0;

	for (i = 0; i < newgun.num; ++i) {
		tvec1.x = workpiece_trans.pos.dx;
		tvec1.y = workpiece_trans.pos.dy;
		tvec1.z = workpiece_trans.pos.dx;
		
		coordinatetransform(gun_trans.rot.mem, &tvec1, &ttri1.normalvector);
		coordinatetransform(gun_trans.rot.mem, &tvec1, &ttri1.vertex1);
		coordinatetransform(gun_trans.rot.mem, &tvec1, &ttri1.vertex2);
		coordinatetransform(gun_trans.rot.mem, &tvec1, &ttri1.vertex3);
		for (j = 0; j < part.num; ++j) {
			tvec2.x = gun_trans.pos.dx;
			tvec2.y = gun_trans.pos.dy;
			tvec2.z = gun_trans.pos.dz;

			coordinatetransform(gun_trans.rot.mem, &tvec2, &ttri2.normalvector);
			coordinatetransform(gun_trans.rot.mem, &tvec2, &ttri2.vertex1);
			coordinatetransform(gun_trans.rot.mem, &tvec2, &ttri2.vertex2);
			coordinatetransform(gun_trans.rot.mem, &tvec2, &ttri2.vertex3);
			
			if (triangleCD(ttri1, ttri2)) {
				collision = 1;
				break;
			}
		}
		if (collision == 1)
			break;
	}
	
	if (collision == 1)
		printf("collision detected!\n");

	free(newgun.ptriangle);
	free(part.ptriangle);
	return 0;
}













