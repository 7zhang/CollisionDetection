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
	
	triangle test8 = {{0.0, 0.0, 1.0}, {1916.150024, -236.330002, 1009.500000}, {1916.150024, 263.670013, 1009.500000}, {1516.150024, 263.670013, 1009.500000}};
	triangle test9 = {{0.879605, 0.086934, 0.467694}, {1821.348755, 0.155291, 1009.083191}, {1790.851196, -0.000000, 1066.484375}, {1821.366821, -0.000000, 1009.092773}};

	if (triangleCD(&test1, &test2) != 0 || triangleCD(&test2, &test1) != 0) {
		printf("test1 error\n");
	} else {
		printf("test1 passed\n");
	}

	if (triangleCD(&test1, &test3) != 0 || triangleCD(&test3, &test1) != 0) {
		printf("test2 not passed\n");
	} else {
		printf("test2 passed\n");
	}
	
	if (triangleCD(&test4, &test5) != 1 || triangleCD(&test5, &test4) != 1) {
		printf("test3 not passed\n");
	} else {
		printf("test3 passed\n");
	}
		
	if (triangleCD(&test4, &test6) != 0 || triangleCD(&test6, &test4) != 0) {
		printf("test4 not passed\n");
	} else {
		printf("test4 passed\n");
	}

	if (triangleCD(&test1, &test7) != 0 || triangleCD(&test7, &test1) != 0) {
		printf("test5 not passed\n");
	} else {
		printf("test5 passed\n");
	}

	if (triangleCD(&test8, &test9) != 1) {
		printf("test6 not passed\n");
	} else {
		printf("test6 passed\n");
	}
	
	if (triangleCD(&test9, &test8) != 1) {
		printf("test7 not passed\n");
	} else {
		printf("test7 passed\n");
	}

	return 0;
}










