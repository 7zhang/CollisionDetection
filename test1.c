#include <stdio.h>
#include <stdlib.h>
#include "triangleCD.h"
#include "math.h"

int main(int argc, char *argv[])
{
	triangle test1 = {{1, 0, 0}, {0, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	triangle test2 = {{0, 1, 0}, {1, 0, 0}, {0, 0, 0}, {0, 0, 1}};
	triangle test3 = {{0, 1, 0}, {1, 0, 0}, {-1, 0, 0}, {-1, 0, 1}};
	triangle test4 = {{1/sqrt(3), 1/sqrt(3), 1/sqrt(3)}, {0, 0, 2}, {2, 0, 0}, {0, 2, 0}};
	triangle test5 = {{1/sqrt(11), 1/sqrt(11), 3/sqrt(11)}, {0, 0, 1}, {3, 0, 0}, {0, 3, 0}};
	triangle test6 = {{1/sqrt(11), 1/sqrt(11), 3/sqrt(11)}, {10, 0, 1}, {13, 0, 0}, {10, 3, 0}};
	triangle test7 = {{1, 0, 0}, {0, 10000, 0}, {0, 10000+1, 0}, {0, 10000, 1}};
	
	if (triangleCD(&test1, &test2) != 0) {
		printf("test1 error\n");
	} else {
		printf("test1 passed\n");
	}

	if (triangleCD(&test1, &test3) != 0) {
		printf("test2 not passed\n");
	} else {
		printf("test2 passed\n");
	}
	
	if (triangleCD(&test4, &test5) != 1) {
		printf("test3 not passed\n");
	} else {
		printf("test3 passed\n");
	}
		
	if (triangleCD(&test4, &test6) != 0) {
		printf("test4 not passed\n");
	} else {
		printf("test4 passed\n");
	}

	if (triangleCD(&test1, &test7) != 0) {
		printf("test5 not passed\n");
	} else {
		printf("test5 passed\n");
	}
		
	return 0;
}










