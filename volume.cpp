#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "volume.h"

void buildvolume(const triangle *t, volume *v)
{
	if (t->vertex1.x < v->xmin)
		v->xmin = t->vertex1.x;
	if (t->vertex1.x > v->xmax)
		v->xmax = t->vertex1.x;
	if (t->vertex1.y < v->ymin)
		v->ymin = t->vertex1.y;
	if (t->vertex1.y > v->ymax)
		v->ymax = t->vertex1.y;
	if (t->vertex1.z < v->zmin)
		v->zmin = t->vertex1.z;
	if (t->vertex1.z > v->zmax)
		v->zmax = t->vertex1.z;

	if (t->vertex2.x < v->xmin)
		v->xmin = t->vertex2.x;
	if (t->vertex2.x > v->xmax)
		v->xmax = t->vertex2.x;
	if (t->vertex2.y < v->ymin)
		v->ymin = t->vertex2.y;
	if (t->vertex2.y > v->ymax)
		v->ymax = t->vertex2.y;
	if (t->vertex2.z < v->zmin)
		v->zmin = t->vertex2.z;
	if (t->vertex2.z > v->zmax)
		v->zmax = t->vertex2.z;
	
	if (t->vertex1.x < v->xmin)
		v->xmin = t->vertex1.x;
	if (t->vertex1.x > v->xmax)
		v->xmax = t->vertex1.x;
	if (t->vertex1.y < v->ymin)
		v->ymin = t->vertex1.y;
	if (t->vertex1.y > v->ymax)
		v->ymax = t->vertex1.y;
	if (t->vertex1.z < v->zmin)
		v->zmin = t->vertex1.z;
	if (t->vertex1.z > v->zmax)
		v->zmax = t->vertex1.z;
}

bool recurbuildtree(volumenode *vnode)
{
	int i;
	volumenode *left = (volumenode *)malloc(sizeof(volumenode));
	volumenode *right = (volumenode *)malloc(sizeof(volumenode));
	
	if (left == NULL || right == NULL) {
		printf("recurbuildtree malloc error!\n");
		return 1;
	}
		
	memset(left, 0, sizeof(volumenode));
	memset(right, 0, sizeof(volumenode));

	left->tarry = vnode->tarry;
	right->tarry = vnode->tarry;
	left->tarraysize = vnode->tarraysize;
	right->tarraysize = vnode->tarraysize;

	int tmp = triangleallocation(vnode, left, right);

	if ( tmp == 0){
		for (i = 0; i < left->trianglenum; ++i) {
			buildvolume(&left->tarry[left->tindex[i]], &left->v);
		}
	
		for (i = 0; i < right->trianglenum; ++i) {
			buildvolume(&right->tarry[right->tindex[i]], &right->v);
		}
		
		vnode->last = 0;
		
		left->parent = vnode;
		right->parent = vnode;
		
		vnode->child1 = left;
		vnode->child2 = right;
		
		free(vnode->tindex);
		vnode->trianglenum = 0;

		if (recurbuildtree(left)) {
			return 1;
		}
		
		if (recurbuildtree(right)) {
			return 1;
		}
		return 0;
	} else if (tmp == 1){
		free(left);
		free(right);
		vnode->last = 1;
		return 0;		
	} else {
		free(left);
		free(right);
		return 1;
	}
}

int triangleallocation(volumenode *parent, volumenode *left, volumenode *right)
{
	int *tmp1, *tmp2;
	tmp1 = (int *)malloc(sizeof(int) * parent->tarraysize);
	tmp2 = (int *)malloc(sizeof(int) * parent->tarraysize);

	if (tmp1 == NULL || tmp2 == NULL){
		printf("triangleallocation malloc error!\n");
		return 2;
	}

	//try x axis
	int len1, len2, len3;
	
	len1 = parent->v.xmax - parent->v.xmin;
	len2 = parent->v.ymax - parent->v.ymin;
	len3 = parent->v.zmax - parent->v.zmin;
	
	int seq[3];
	if (len1 > len2 && len2 > len3) {
		seq[1] = 1;
		seq[2] = 2;
		seq[3] = 3;
	} else if (len1 > len3 && len3 > len2) {
		seq[1] = 1;
		seq[2] = 3;
		seq[3] = 2;
	} else if (len2 > len1 && len1 > len3) {
		seq[1] = 2;
		seq[2] = 1;
		seq[3] = 3;
	} else if (len2 > len3 && len3 > len1) {
		seq[1] = 2;
		seq[2] = 3;
		seq[3] = 1;
	} else if (len3 > len1 && len1 > len2) {
		seq[1] = 3;
		seq[2] = 1;
		seq[3] = 2;
	} else if (len3 > len2 && len2 > len1) {
		seq[1] = 3;
		seq[2] = 2;
		seq[3] = 1;
	}
	
	int i, j;
	int leftcount = 0;
	int rightcount = 0;
	int middlex, middley, middlez, centerx, centery, centerz;
		
	for (i = 0; i < 3; ++i) {
		switch(seq[i]) {
		case 1:
			middlex = (parent->v.xmin + parent->v.xmax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centerx = (parent->tarry[parent->tindex[j]].vertex1.x
					+ parent->tarry[parent->tindex[j]].vertex2.x
					  + parent->tarry[parent->tindex[j]].vertex3.x)/3.0;
				if (centerx < middlex) {
					leftcount++;
					tmp1[leftcount-1] = j;
				} else {
					rightcount++;
					tmp2[rightcount-1] = j;
				}
			}
			
			if (leftcount > 0 && rightcount > 0) {
				left->tindex = tmp1;
				left->trianglenum = leftcount;
				right->tindex = tmp2;
				right->trianglenum = rightcount;
				
				return 0;
			}			
			return 1;
		case 2:
			middley = (parent->v.ymin + parent->v.ymax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centery = (parent->tarry[parent->tindex[j]].vertex1.y
					+ parent->tarry[parent->tindex[j]].vertex2.y
					  + parent->tarry[parent->tindex[j]].vertex3.y)/3.0;
				if (centery < middley) {
					leftcount++;
					tmp1[leftcount-1] = j;
				} else {
					rightcount++;
					tmp2[rightcount-1] = j;
				}
			}
			
			if (leftcount > 0 && rightcount > 0) {
				left->tindex = tmp1;
				left->trianglenum = leftcount;
				right->tindex = tmp2;
				right->trianglenum = rightcount;
				
				return 0;
			}			
			return 1;
		case 3:
			middlez = (parent->v.zmin + parent->v.zmax)/2.0;
		
			for (j = 0; j < parent->trianglenum; ++j) {
				centerz = (parent->tarry[parent->tindex[j]].vertex1.z
					+ parent->tarry[parent->tindex[j]].vertex2.z
					  + parent->tarry[parent->tindex[j]].vertex3.z)/3.0;
				if (centerz < middlez) {
					leftcount++;
					tmp1[leftcount-1] = j;
				} else {
					rightcount++;
					tmp2[rightcount-1] = j;
				}
			}
			
			if (leftcount > 0 && rightcount > 0) {
				left->tindex = tmp1;
				left->trianglenum = leftcount;
				right->tindex = tmp2;
				right->trianglenum = rightcount;
				
				return 0;
			}
			return 1;
		}
	}
}



















