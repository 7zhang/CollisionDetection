#include "loadstl.h"
#include <stdio.h>
#include <stdlib.h>

int loadstl(const char *path, stldata *pstldata)
{
	FILE *file;
	if((file = fopen(path, "rb")) == NULL)
	{
		printf("open file error\n");
		return 1;
	}
	if(fread(&(pstldata->modelname), sizeof(char), 80, file) != 80)
	{
		printf("modelname read error\n");
		return 1;
	}
	if(fread(&(pstldata->num), 4, 1, file) !=1)
	{
		printf("number read error\n");
		return 1;
	}
	pstldata->ptriangle=(triangle *)malloc(pstldata->num * sizeof(triangle));
	triangle *iter = pstldata->ptriangle;
	for(int i = 0; i < pstldata->num; i++)
	{
		fread(&(iter->normalvector.x), sizeof(float), 1, file);
		fread(&(iter->normalvector.y), sizeof(float), 1, file);
		fread(&(iter->normalvector.z), sizeof(float), 1, file);
		fread(&(iter->vertex1.x), sizeof(float), 1, file);
		fread(&(iter->vertex1.y), sizeof(float), 1, file);
		fread(&(iter->vertex1.z), sizeof(float), 1, file);
		fread(&(iter->vertex2.x), sizeof(float), 1, file);
		fread(&(iter->vertex2.y), sizeof(float), 1, file);
		fread(&(iter->vertex2.z), sizeof(float), 1, file);
		fread(&(iter->vertex3.x), sizeof(float), 1, file);
		fread(&(iter->vertex3.y), sizeof(float), 1, file);
		fread(&(iter->vertex3.z), sizeof(float), 1, file);
		fseek(file, 2, SEEK_CUR);
		iter++;
	}
	return 0;
}





	
