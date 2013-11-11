#include <stdio.h>
#include <stdlib.h>
#include "loadstl.h"
#include "triangleCD.h"
#include "Transform.h"

int main(void)
{
	stldata newgun;
	stldata part;
	
	if (loadstl("mystl/newgun.STL", &newgun) != 0)
		printf("load newgun stl error\n");
	
	if (loadstl("mystl/part.STL", &part) != 0)
		printf("load part stl error!\n");

	printf("stlmodel1: %s, %d triangles\n", newgun.modelname, newgun.num);
	printf("stlmodel2: %s, %d tirangles\n", part.modelname, part.num);
	printf("starting collisiondetection...\n");

	int collision = 0;

	int i, j;
	for (i = 0; i < part.num; ++i) {		
		for (j = 0; j < newgun.num; ++j) {						
			if (triangleCD(&newgun.ptriangle[j], &part.ptriangle[i])) {
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

