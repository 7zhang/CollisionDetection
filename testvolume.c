#include <stdio.h>
#include <stdlib.h>
#include "volume.h"
#include "loadstl.h"

int volumecount = 0;
int maxdepth = 0;

int main(int argc, char *argv[])
{
	stldata stl;
	
	if (loadstl(argv[1], &stl) != 0) {
		printf("loadstl %s error!\n", argv[1]);
		exit(1);
	}
	
	volumenode topnode;
	topnode.tarry = stl.ptriangle;
	topnode.tarraysize = stl.num;
	topnode.tindex = (int *)malloc(sizeof(int) * stl.num);
	
	int i;
	for (i = 0; i < stl.num; ++i)
		topnode.tindex[i] = i;
	
	topnode.trianglenum = stl.num;
	buildvolume(&topnode);
	topnode.parent = NULL;

	volumecount++;
	maxdepth = 30;

	if (recurbuildtree(&topnode, 0) == -1) 
		printf("recurbuildtree error!\n");
	
	printf("volumecount = %d\n", volumecount);
	
//	recurshowtree(&topnode, 0);
	return 0;
}

















