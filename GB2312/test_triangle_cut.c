#include <stdio.h>
#include <stdlib.h>
#include "volume.h"
#include "loadstl.h"

int main(int argc, char *argv[])
{
	stldata stl;
	if (loadstl(argv[1], &stl) != 0) {
		printf("loadstl %s error!\n", argv[1]);
		exit(1);
	}

	printf("triangle number = %d\n", stl.num);
	
	int *n = (int *)malloc(sizeof(int));
	int *index = (int *)malloc(sizeof(int));
	triangle *tmp = (triangle *)malloc(sizeof(triangle) * 2 * stl.num);
	triangle **array = (triangle **)malloc(sizeof(triangle *));
	*n = stl.num;
	*index = 0;
	*array = tmp;
	
	int i;
	for (i = 0; i < stl.num; ++i)
	{
		triangle_cut_recur(array, n, index, &stl.ptriangle[i]);
	}

	for (i = 0; i < *index; ++i)
		show_triangle(*array, i);

	return 0;
}


















