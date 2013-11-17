#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "volume.h"
#include "loadstl.h"
#include "triangleCD.h"

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

	maxdepth = 10;

	if (recurbuildtree(&left_top_node, 0) == -1) 
		printf("recurbuildtree error!\n");

#ifdef DEBUG
	printf("volumecount1 = %d, lastcount1 = %d\n", volumecount, last_count);
#endif

	if (recurbuildtree(&right_top_node, 0) == -1) 
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

















