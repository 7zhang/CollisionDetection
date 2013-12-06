#include <stdio.h>
#include <stdlib.h>
#include "loadstl.h"
#include "triangle_cd.h"
#include "volume.h"

int volumecount = 0;
int maxdepth = 0;
int cdcount = 0;
int last_count = 0;
int triangle_cd_count = 0;

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
		printf("i = %d, j = %d\n", i, j);
	} else {
		printf("no collision\n");
	}
	
	show_triangle(part.ptriangle, 897);
	show_triangle(newgun.ptriangle, 57812);

	free(newgun.ptriangle);
	free(part.ptriangle);
	return 0;
}

