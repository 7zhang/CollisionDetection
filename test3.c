#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "loadstl.h"
#include "triangleCD.h"
#include "Transform.h"

float distance(vector3d *p1, vector3d *p2)
{
	return sqrt(pow(p1->x - p2->x, 2) + pow(p1->y - p2->y, 2) + powl(p1->z - p2->z, 2));
}

void tran(TRANS *t, stldata *stlfrom, stldata *stlto)
{
	int i;
	vector3d tvec1;

	tvec1.x = t->pos.dx;
	tvec1.y = t->pos.dy;
	tvec1.z = t->pos.dz;

	for (i = 0; i < stlfrom->num; ++i) {
		triangle_transform(t->rot.mem, &tvec1, &stlfrom->ptriangle[i], &stlto->ptriangle[i]);
		stlto->ptriangle[i].attr = stlfrom->ptriangle[i].attr;
	}
}

int main(void)
{
	stldata newgun;
	stldata part;
	stldata j0, j1, j2, j3, j4, j5, j6;
	
	if (loadstl("robot_stl/newgun.STL", &newgun) != 0)
		printf("load newgun stl error\n");
	
	if (loadstl("robot_stl/Part.STL", &part) != 0)
		printf("load part stl error!\n");

	if (loadstl("robot_stl/J0.STL", &j0) != 0)
		printf("load newgun stl error\n");

	if (loadstl("robot_stl/J1.STL", &j1) != 0)
		printf("load newgun stl error\n");

	if (loadstl("robot_stl/J2.STL", &j2) != 0)
		printf("load newgun stl error\n");

	if (loadstl("robot_stl/J3.STL", &j3) != 0)
		printf("load newgun stl error\n");

	if (loadstl("robot_stl/J4.STL", &j4) != 0)
		printf("load newgun stl error\n");

	if (loadstl("robot_stl/J5.STL", &j5) != 0)
		printf("load newgun stl error\n");

	if (loadstl("robot_stl/J6.STL", &j6) != 0)
		printf("load newgun stl error\n");

	printf("newgun: %s, %d triangles\n", newgun.modelname, newgun.num);
	printf("part: %s, %d tirangles\n", part.modelname, part.num);
	printf("j0: %s, %d triangles\n", j0.modelname, j0.num);
	printf("j1: %s, %d triangles\n", j1.modelname, j1.num);
	printf("j2: %s, %d triangles\n", j2.modelname, j2.num);
	printf("j3: %s, %d triangles\n", j3.modelname, j3.num);
	printf("j4: %s, %d triangles\n", j4.modelname, j4.num);
	printf("j5: %s, %d triangles\n", j5.modelname, j5.num);
	printf("j6: %s, %d triangles\n", j6.modelname, j6.num);

	JAngle robotangle(0.0, -25.00, 65.00, 0.00, 0.00, 0.00);
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
	
	stldata newgunbuffer, partbuffer, j0buffer, j1buffer, j2buffer, j3buffer, j4buffer, j5buffer, j6buffer;

	strcpy(newgunbuffer.modelname, newgun.modelname);
	strcpy(partbuffer.modelname, part.modelname);
	strcpy(j0buffer.modelname, j0.modelname);
	strcpy(j1buffer.modelname, j1.modelname);
	strcpy(j2buffer.modelname, j2.modelname);
	strcpy(j3buffer.modelname, j3.modelname);
	strcpy(j4buffer.modelname, j4.modelname);
	strcpy(j5buffer.modelname, j5.modelname);
	strcpy(j6buffer.modelname, j6.modelname);

	newgunbuffer.num = newgun.num;
	partbuffer.num = part.num;
	j0buffer.num = j0.num;
	j1buffer.num = j1.num;
	j2buffer.num = j2.num;
	j3buffer.num = j3.num;
	j4buffer.num = j4.num;
	j5buffer.num = j5.num;
	j6buffer.num = j6.num;

	newgunbuffer.ptriangle = (triangle *)malloc(newgun.num * sizeof(triangle));
	partbuffer.ptriangle = (triangle *)malloc(part.num * sizeof(triangle));
	j0buffer.ptriangle = (triangle *)malloc(j0.num * sizeof(triangle));
	j1buffer.ptriangle = (triangle *)malloc(j1.num * sizeof(triangle));
	j2buffer.ptriangle = (triangle *)malloc(j2.num * sizeof(triangle));
	j3buffer.ptriangle = (triangle *)malloc(j3.num * sizeof(triangle));
	j4buffer.ptriangle = (triangle *)malloc(j4.num * sizeof(triangle));
	j5buffer.ptriangle = (triangle *)malloc(j5.num * sizeof(triangle));
	j6buffer.ptriangle = (triangle *)malloc(j6.num * sizeof(triangle));

	tran(&newgun_trans, &newgun, &newgunbuffer);
	tran(&part_trans, &part, &partbuffer);
	tran(&j0_trans, &j0, &j0buffer);
	tran(&j1_trans, &j1, &j1buffer);
	tran(&j2_trans, &j2, &j2buffer);
	tran(&j3_trans, &j3, &j3buffer);
	tran(&j4_trans, &j4, &j4buffer);
	tran(&j5_trans, &j5, &j5buffer);
	tran(&j6_trans, &j6, &j6buffer);

	if (writestl("mystl/newgun.STL", &newgunbuffer))
		printf("stl newgun write error!\n");		
	if (writestl("mystl/part.STL", &partbuffer))
		printf("stl part write error!\n");
	if (writestl("mystl/j0.STL", &j0buffer))
		printf("stl j0 write error!\n");
	if (writestl("mystl/j1.STL", &j1buffer))
		printf("stl j1 write error!\n");
	if (writestl("mystl/j2.STL", &j2buffer))
		printf("stl j2 write error!\n");
	if (writestl("mystl/j3.STL", &j3buffer))
		printf("stl j3 write error!\n");
	if (writestl("mystl/j4.STL", &j4buffer))
		printf("stl j4 write error!\n");
	if (writestl("mystl/j5.STL", &j5buffer))
		printf("stl j5 write error!\n");
	if (writestl("mystl/j6.STL", &j6buffer))
		printf("stl j6 write error!\n");

	/* for (i = 0; i < newgun.num; ++i) { */
	/* 	printf("Point %d: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f), (%f, %f, %f)\n", */
	/* 	       i, newgun.ptriangle[i].normalvector.x, newgun.ptriangle[i].normalvector.y, */
	/* 	       newgun.ptriangle[i].normalvector.z, newgun.ptriangle[i].vertex1.x, */
	/* 	       newgun.ptriangle[i].vertex1.y, newgun.ptriangle[i].vertex1.z, */
	/* 	       newgun.ptriangle[i].vertex2.x, newgun.ptriangle[i].vertex2.y, */
	/* 		newgun.ptriangle[i].vertex2.z, newgun.ptriangle[i].vertex3.x, */
	/* 	       newgun.ptriangle[i].vertex3.y, newgun.ptriangle[i].vertex3.z); */
	/* } */

	/* for (i = 0; i < newgun.num; ++i) { */
	/* 	printf("distance %d: %f  %f\n", i, distance(&newgun.ptriangle[i].vertex1, &newgun.ptriangle[i].vertex2), distance(&newgunbuffer.ptriangle[i].vertex1, &newgunbuffer.ptriangle[i].vertex2)); */
	/* } */

	/* printf("part\n"); */

	/* for (i = 0; i < part.num; ++i) { */
	/* 	printf("distance %d: %f  %f\n", i, distance(&part.ptriangle[i].vertex1, &part.ptriangle[i].vertex2), distance(&partbuffer.ptriangle[i].vertex1, &partbuffer.ptriangle[i].vertex2)); */
	/* } */


	free(newgun.ptriangle);
	free(part.ptriangle);
	free(j0.ptriangle);
	free(j1.ptriangle);
	free(j2.ptriangle);
	free(j3.ptriangle);
	free(j4.ptriangle);
	free(j5.ptriangle);
	free(j6.ptriangle);

	free(newgunbuffer.ptriangle);
	free(partbuffer.ptriangle);
	free(j0buffer.ptriangle);
	free(j1buffer.ptriangle);
	free(j2buffer.ptriangle);
	free(j3buffer.ptriangle);
	free(j4buffer.ptriangle);
	free(j5buffer.ptriangle);
	free(j6buffer.ptriangle);

	return 0;
}
