#include <stdio.h>
#include <stdlib.h>
#include "loadstl.h"
#include "triangleCD.h"

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
	
	long n1, n2;
	n1 = 0;
	n2 = 0;
	
	int i, j;
	for (i = 0; i < newgun.num; i++)
		for (j = 0; j < part.num; j++)
			triangleCD(&newgun.ptriangle[i], &part.ptriangle[j]);
	
	printf("there are %ld potential collision triangle\n, n1 = %d, n2 = %d\n", n1 + n2, n1, n2);
	free(newgun.ptriangle);
	free(part.ptriangle);
	return 0;
}













