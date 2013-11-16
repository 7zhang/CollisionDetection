#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "volume.h"

/* use the tarray and tindex to build the volume v
 */
int buildvolume(volumenode *vnode)
{
	int i;
	triangle *ttmp = &vnode->tarry[vnode->tindex[0]];
	volume *vtmp = &vnode->v;
	vtmp->xmin = ttmp->vertex1.x;
	vtmp->xmax = ttmp->vertex1.x;
	vtmp->ymin = ttmp->vertex1.y;
	vtmp->ymax = ttmp->vertex1.y;
	vtmp->zmin = ttmp->vertex1.z;
	vtmp->zmax = ttmp->vertex1.z;

	if (vnode->trianglenum == 0) {
		printf("buildvolume error! vnode->trianglenum = 0\n");
		return 1;
	}

	for (i = 0; i < vnode->trianglenum; ++i) {
		if (vnode->tindex[i] >= vnode->tarraysize) {
			printf("index error! vnode->tindex[%d] = %d, while vnode->trianglenum = %d\n",
			       i, vnode->tindex[i], vnode->trianglenum);
			return 1;
		}
		  
		ttmp = &vnode->tarry[vnode->tindex[i]];

		if (ttmp->vertex1.x < vtmp->xmin)
			vtmp->xmin = ttmp->vertex1.x;
		if (ttmp->vertex1.x > vtmp->xmax)
			vtmp->xmax = ttmp->vertex1.x;
		if (ttmp->vertex1.y < vtmp->ymin)
			vtmp->ymin = ttmp->vertex1.y;
		if (ttmp->vertex1.y > vtmp->ymax)
			vtmp->ymax = ttmp->vertex1.y;
		if (ttmp->vertex1.z < vtmp->zmin)
			vtmp->zmin = ttmp->vertex1.z;
		if (ttmp->vertex1.z > vtmp->zmax)
			vtmp->zmax = ttmp->vertex1.z;

		if (ttmp->vertex2.x < vtmp->xmin)
			vtmp->xmin = ttmp->vertex2.x;
		if (ttmp->vertex2.x > vtmp->xmax)
			vtmp->xmax = ttmp->vertex2.x;
		if (ttmp->vertex2.y < vtmp->ymin)
			vtmp->ymin = ttmp->vertex2.y;
		if (ttmp->vertex2.y > vtmp->ymax)
			vtmp->ymax = ttmp->vertex2.y;
		if (ttmp->vertex2.z < vtmp->zmin)
			vtmp->zmin = ttmp->vertex2.z;
		if (ttmp->vertex2.z > vtmp->zmax)
			vtmp->zmax = ttmp->vertex2.z;
	
		if (ttmp->vertex3.x < vtmp->xmin)
			vtmp->xmin = ttmp->vertex3.x;
		if (ttmp->vertex3.x > vtmp->xmax)
			vtmp->xmax = ttmp->vertex3.x;
		if (ttmp->vertex3.y < vtmp->ymin)
			vtmp->ymin = ttmp->vertex3.y;
		if (ttmp->vertex3.y > vtmp->ymax)
			vtmp->ymax = ttmp->vertex3.y;
		if (ttmp->vertex3.z < vtmp->zmin)
			vtmp->zmin = ttmp->vertex3.z;
		if (ttmp->vertex3.z > vtmp->zmax)
			vtmp->zmax = ttmp->vertex3.z;
	}

	if (vnode->v.xmax - vnode->v.xmin < POSITIVEZERO && vnode->v.ymax - vnode->v.ymin < POSITIVEZERO && vnode->v.zmax - vnode->v.zmin < POSITIVEZERO) {
		printf("error:x y z length all equal 0");
	}

	/* if (vnode->v.ymax - vnode->v.ymin < POSITIVEZERO) { */
	/* 	printf("y length equal 0"); */
	/* } */
	
	/* if (vnode->v.zmax - vnode->v.zmin < POSITIVEZERO) { */
	/* 	printf("z length equal 0"); */
	/* } */
	
	return 0;
}

int recurbuildtree(volumenode *vnode, int depth)
{
	volumenode *left = (volumenode *)malloc(sizeof(volumenode));
	volumenode *right = (volumenode *)malloc(sizeof(volumenode));
	
	if (left == NULL || right == NULL) {
		printf("recurbuildtree malloc error!\n");
		return -1;
	}
		
	memset(left, 0, sizeof(volumenode));
	memset(right, 0, sizeof(volumenode));

	left->tarry = vnode->tarry;
	right->tarry = vnode->tarry;
	left->tarraysize = vnode->tarraysize;
	right->tarraysize = vnode->tarraysize;

	int tmp = triangleallocation(vnode, left, right);

	switch(tmp){
	case 0:
		if (buildvolume(left) != 0) {
			printf("buildvolume(left) error!\n");
			free(left->tindex);
			free(right->tindex);
			free(left);
			free(right);
			
			return -1;
		}
			      
		if (buildvolume(right) != 0) {
			printf("buildvolume(right) error!\n");
			free(left->tindex);
			free(right->tindex);
			free(left);
			free(right);

			return -1;
		}
		
		vnode->last = 0;
		last_count++;
		
		left->parent = vnode;
		right->parent = vnode;
		
		vnode->child1 = left;
		vnode->child2 = right;
		
		free(vnode->tindex);
		vnode->trianglenum = 0;

		volumecount += 2;

		if (depth < maxdepth) {
			if (recurbuildtree(left, depth + 1) == -1) {
				printf("recurbuildtree(left) error! left->trianglenum = %d\n",
				       left->trianglenum);
				return -1;
			}
		
			if (recurbuildtree(right, depth + 1) == -1) {
				printf("recurbuildtree(right) error! right->trianglenum = %d\n",
				       right->trianglenum);
				return -1;
			}
		}
		
		return 0;
	case 1:
		free(left);
		free(right);
		vnode->last = 1;
		vnode->child1 = NULL;
		vnode->child2 = NULL;
		/* printf("depth = %d\n", depth); */

		return 0;		
	case 2:
		free(left);
		free(right);
		printf("triangleallocation error!\n");
		
		return -1;
	}
}

int triangleallocation(const volumenode *parent, volumenode *left, volumenode *right)
{
	if (parent->trianglenum < 1) {
		printf("parent->trianglenum = %d\n", parent->trianglenum);
		return 1;
	}

	int *tmp1, *tmp2;
	tmp1 = (int *)malloc(sizeof(int) * parent->trianglenum);

	if (tmp1 == NULL){
		printf("triangleallocation malloc error!\n");
		
		left->tindex = NULL;
		left->trianglenum = 0;
		right->tindex = NULL;
		right->trianglenum = 0;

		return 2;
	}

	tmp2 = (int *)malloc(sizeof(int) * parent->trianglenum);

	if (tmp2 == NULL){
		printf("triangleallocation malloc error!\n");
		
		free(tmp1);
		left->tindex = NULL;
		left->trianglenum = 0;
		right->tindex = NULL;
		right->trianglenum = 0;

		return 2;
	}

	memset(tmp1, 0, sizeof(int) * parent->trianglenum);
	memset(tmp2, 0, sizeof(int) * parent->trianglenum);

	//try x axis
	double len1, len2, len3;
	
	len1 = parent->v.xmax - parent->v.xmin;
	len2 = parent->v.ymax - parent->v.ymin;
	len3 = parent->v.zmax - parent->v.zmin;

	if (len1 < MYZERO && len2 < MYZERO && len3 < MYZERO)
	{
		/* free(tmp1); */
		/* free(tmp2); */
		/* left->tindex = NULL; */
		/* left->trianglenum = 0; */
		/* right->tindex = NULL; */
		/* right->trianglenum = 0; */
	
		printf("fatal error!\n");
	}

	int seq[3];
	if (len1 > len2 && len2 > len3) {
		seq[0] = 1;
		seq[1] = 2;
		seq[2] = 3;
	} else if (len1 > len3 && len3 > len2) {
		seq[0] = 1;
		seq[1] = 3;
		seq[2] = 2;
	} else if (len2 > len1 && len1 > len3) {
		seq[0] = 2;
		seq[1] = 1;
		seq[2] = 3;
	} else if (len2 > len3 && len3 > len1) {
		seq[0] = 2;
		seq[1] = 3;
		seq[2] = 1;
	} else if (len3 > len1 && len1 > len2) {
		seq[0] = 3;
		seq[1] = 1;
		seq[2] = 2;
	} else if (len3 > len2 && len2 > len1) {
		seq[0] = 3;
		seq[1] = 2;
		seq[2] = 1;
	} else {
		seq[0] = 1;
		seq[1] = 2;
		seq[2] = 3;
	}
	
	int i, j;
	int leftcount = 0;
	int rightcount = 0;
	int success = 0;
	int middlex, middley, middlez, centerx, centery, centerz;
		
	for (i = 0; i < 3; ++i) {
		leftcount = 0;
		rightcount = 0;

		switch(seq[i]) {
		case 1:
			middlex = (parent->v.xmin + parent->v.xmax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centerx = (parent->tarry[parent->tindex[j]].vertex1.x
					+ parent->tarry[parent->tindex[j]].vertex2.x
					  + parent->tarry[parent->tindex[j]].vertex3.x)/3.0;

				/* if (parent->tindex[j] == 897) */
				/* 	printf("897\n"); */
				
				if (centerx < middlex) {
					leftcount++;
					tmp1[leftcount-1] = parent->tindex[j];
				} else {
					rightcount++;
					tmp2[rightcount-1] = parent->tindex[j];
				}
			}
			
			if (leftcount > 0 && rightcount > 0) {
				success = 1;
			}
			
			break;
		case 2:
			middley = (parent->v.ymin + parent->v.ymax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centery = (parent->tarry[parent->tindex[j]].vertex1.y
					+ parent->tarry[parent->tindex[j]].vertex2.y
					  + parent->tarry[parent->tindex[j]].vertex3.y)/3.0;
				
				/* if (parent->tindex[j] == 897) */
				/* 	printf("897\n"); */
				
				if (centery < middley) {
					leftcount++;
					tmp1[leftcount-1] = parent->tindex[j];
				} else {
					rightcount++;
					tmp2[rightcount-1] = parent->tindex[j];
				}
			}
			
			if (leftcount > 0 && rightcount > 0) {
				success = 1;
			}
			
			break;
		case 3:
			middlez = (parent->v.zmin + parent->v.zmax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centerz = (parent->tarry[parent->tindex[j]].vertex1.z
					+ parent->tarry[parent->tindex[j]].vertex2.z
					  + parent->tarry[parent->tindex[j]].vertex3.z)/3.0;

				/* if (parent->tindex[j] == 897) */
				/* 	printf("897\n"); */
				
				if (centerz < middlez) {
					leftcount++;
					tmp1[leftcount-1] = parent->tindex[j];
				} else {
					rightcount++;
					tmp2[rightcount-1] = parent->tindex[j];
				}
			}
			
			if (leftcount > 0 && rightcount > 0)
				success = 1;

			break;
		default:
			success = 0;
			printf("warning: triangleallocation switch case error\n!");
			break;
		}

		if (success == 1) {
			/* printf("i = %d\n", i);		 */
			break;
		}
	}

	if (success == 0) {
		/* printf("can't split!\n"); */
		/* printf("parent->trianglenum = %d\n", parent->trianglenum); */
		/* printf("leftcount = %d, tmp1[0] = %d, tmp1[leftcount-1] = %d\n", */
		/*        leftcount, tmp1[0], tmp1[leftcount-1]); */
		/* printf("rightcount = %d, tmp2[0] = %d, tmp2[rightcount-1] = %d\n", */
		/*        rightcount, tmp2[0], tmp2[rightcount-1]); */

		/* if (leftcount > 0 || rightcount > 0) { */
		/* 	printf("len1 = %lf, len2 = %lf, len3 = %lf\n", len1, len2, len3); */
		/* 	for (i = 0; i < leftcount; ++i)  */
		/* 		show_triangle(parent->tarry, tmp1[i]); */
		/* } */

		/* if (parent->trianglenum > 30) { */
		/* 	printf("parent->trianglenum too large: %d\n", parent->trianglenum); */
		/* } */
		


		left->tindex = NULL;
		right->tindex = NULL;
		left->trianglenum = 0;
		right->trianglenum = 0;
		free(tmp1);
		free(tmp2);
		
		return 1;
	}
		


	left->tindex = (int *)malloc(sizeof(int) * leftcount);
	if (left->tindex == NULL) {
		left->tindex = NULL;
		right->tindex = NULL;
		leftcount = 0;
		rightcount = 0;

		free(tmp1);
		free(tmp2);

		return 2;
	}

	right->tindex = (int *)malloc(sizeof(int) * rightcount);				
	if (right->tindex == NULL) {
		left->tindex = NULL;
		right->tindex = NULL;
		leftcount = 0;
		rightcount = 0;
		
		free(left->tindex);
		free(tmp1);
		free(tmp2);
		
		return 2;
	}
	
	memcpy(left->tindex, tmp1, sizeof(int) * leftcount);
	memcpy(right->tindex, tmp2, sizeof(int) * rightcount);
	
	left->trianglenum = leftcount;
	right->trianglenum = rightcount;

	free(tmp1);
	free(tmp2);

	return 0;
}

//show the volume tree
void recurshowtree(const volumenode *vnode, int depth)
{
	int i;
	
	if (vnode->child1 != NULL && vnode->child1->trianglenum != 0) {
		for (i = 0; i < depth; ++i) 
			printf("  ");

		printf("%d\n", vnode->child1->trianglenum);
	}

	if (vnode->child2 != NULL && vnode->child2->trianglenum != 0) { 
		for (i = 0; i < depth; ++i) 
			printf("  ");

		printf("%d\n", vnode->child2->trianglenum);

	
	}
	
	if (vnode->child1 != NULL && vnode->child2 != NULL) {
		recurshowtree(vnode->child1, depth + 1);
		recurshowtree(vnode->child2, depth + 1);
	}
}

void show_triangle(triangle *t, int index) 
{
	printf("triangle %d:", index);
	printf("normal vector: x = %f, y = %f, z = %f\n",
	       t[index].normalvector.x, t[index].normalvector.y, t[index].normalvector.z);
	printf("vertex1: x = %f, y = %f, z = %f\n", t[index].vertex1.x, t[index].vertex1.y, t[index].vertex1.z);
	printf("vertex2: x = %f, y = %f, z = %f\n", t[index].vertex2.x, t[index].vertex2.y, t[index].vertex2.z);
	printf("vertex3: x = %f, y = %f, z = %f\n", t[index].vertex3.x, t[index].vertex3.y, t[index].vertex3.z);
}

int collision_detection_recur(const volumenode *vnode1, const volumenode *vnode2)
{
	if (vnode1 == NULL || vnode2 == NULL) {
		return 0;
	}	
	
	cdcount++;
	int tmp;
	int i, j;
	int collision_flag = 0;
	if (volumecd(&vnode1->v, &vnode2->v)) {
		tmp = vnode1->last + vnode2->last * 2;
		switch(tmp) {
		case 0:
			return (collision_detection_recur(vnode1->child1, vnode2->child1) ||
				collision_detection_recur(vnode1->child1, vnode2->child2) ||
				collision_detection_recur(vnode1->child2, vnode2->child1) ||
				collision_detection_recur(vnode1->child2, vnode2->child2));
		case 1:
			return (collision_detection_recur(vnode1, vnode2->child1) ||
				collision_detection_recur(vnode1, vnode2->child2));
		case 2:
			return (collision_detection_recur(vnode1->child1, vnode2) ||
				collision_detection_recur(vnode1->child2, vnode2));
		case 3:
			triangle_cd_count++;

			if (vnode1->trianglenum < 1 || vnode2->trianglenum < 1) 
				printf("number error!\n");
			
			for (i = 0; i < vnode1->trianglenum; ++i) {
				for (j = 0; j < vnode2->trianglenum; ++j) {
//					if (vnode1->tindex[i] == 897 &&  vnode2->tindex[j] == 57812)
					/* printf("tindex[i] = %d, tindex[j] = %d\n", */
					/*        vnode1->tindex[i], vnode2->tindex[j]); */



					collision_flag = triangleCD(&vnode1->tarry[vnode1->tindex[i]],
								    &vnode2->tarry[vnode2->tindex[j]]);
					if (collision_flag) {
						printf("triangle collision!index1 = %d, index2 = %d\n",
						       vnode1->tindex[i], vnode2->tindex[j]);
						break;
					}
				}
				if (collision_flag) 
					break;
			}

			if (collision_flag) 
				return 1;
			
			return 0;
		default:
			printf("vnode->last error!vnode1->last = %d, vnode2->last = %d\n",
			       vnode1->last, vnode2->last);
			return 2;
		}
	}

	printf("novolumecd!\n");
	
	return 0;
}










