#include <stdio.h>
#include <stdlib.h>
#include "triangleCD.h"

int main(int argc, char *argv[])
{
	triangle test1 = {{1, 0, 0}, {0, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	triangle test2 = {{0, 1, 0}, {1, 0, 0}, {0, 0, 0}, {0, 0, 1}};
	triangle test3 = {{0, 1, 0}, {1, 0, 0}, {-1, 0, 0}, {-1, 0, 1}};
	printf("test1 result: %d:\n", triangleCD(&test1, &test2));
	printf("test2 result: %d:\n", triangleCD(&test1, &test3));
	return 0;
}










