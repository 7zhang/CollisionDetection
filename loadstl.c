#include "loadstl.h"
#include <stdio.h>
#include <stdlib.h>

int loadstl(const char *path, stldata *pstldata)
{
	FILE *file;
	if((file = fopen(path, "rb")) == NULL) {
		printf("open file error\n");
		return 1;
	}
	if(fread(&(pstldata->modelname), sizeof(char), 80, file) != 80)	{
		printf("modelname read error\n");
		return 1;
	}
	if(fread(&(pstldata->num), 4, 1, file) !=1) {
		printf("number read error\n");
		return 1;
	}
	pstldata->ptriangle=(triangle *)malloc(pstldata->num * sizeof(triangle));
	triangle *iter = pstldata->ptriangle;
	
	int i;
	float tmp;
	unsigned short tmp1;
	
	for(i = 0; i < pstldata->num; i++) {
		fread(&(tmp), sizeof(float), 1, file);
		iter->normalvector.x = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->normalvector.y = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->normalvector.z = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex1.x = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex1.y = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex1.z = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex2.x = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex2.y = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex2.z = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex3.x = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex3.y = tmp;
		fread(&(tmp), sizeof(float), 1, file);
		iter->vertex3.z = tmp;
		fread(&(tmp1), sizeof(unsigned short), 1, file);
		iter->attr = tmp;
		
		iter++;
	}

	if (fread(&(tmp), 1, 1, file) == 1)
		return 1;
	
	return 0;
}


int writestl(const char *path, stldata stl[])
	{
	FILE *file;
	if((file = fopen(path, "wb")) == NULL) {
		printf("open file error\n");
		return 1;
	}
	if(fwrite(&(stl->modelname), sizeof(char), 80, file) != 80)	{
		printf("modelname read error\n");
		return 1;
	}
	if(fwrite(&(stl->num), 4, 1, file) !=1) {
		printf("number read error\n");
		return 1;
	}

	triangle *iter = stl->ptriangle;
	
	int i;
	float tmp;
	unsigned short tmp1;

	for(i = 0; i < stl->num; i++) {
		tmp = iter->normalvector.x;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->normalvector.y;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->normalvector.z;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex1.x;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex1.y;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex1.z;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex2.x;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex2.y;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex2.z;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex3.x;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex3.y;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp = iter->vertex3.z;
		fwrite(&(tmp), sizeof(float), 1, file);
		tmp1 = iter->attr;
		fwrite(&(tmp1), sizeof(unsigned short), 1, file);
		
		iter++;
	}
	return 0;
}

