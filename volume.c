#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "volume.h"
#include "triangleCD.h"
#include "loadstl.h"
#include "geom.h"

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

#ifdef DEBUG
	if (vnode->trianglenum == 0) {
		printf("buildvolume error! vnode->trianglenum = 0\n");
		return 1;
	}
#endif

	for (i = 0; i < vnode->trianglenum; ++i) {

#ifdef DEBUG
		if (vnode->tindex[i] >= vnode->tarraysize) {
			printf("index error! vnode->tindex[%d] = %d, while vnode->trianglenum = %d\n",
			       i, vnode->tindex[i], vnode->trianglenum);
			return 1;
		}
#endif
		  
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

#ifdef DEBUG
	if (vnode->v.xmax - vnode->v.xmin < POSITIVEZERO && vnode->v.ymax - vnode->v.ymin < POSITIVEZERO && vnode->v.zmax - vnode->v.zmin < POSITIVEZERO) {
		printf("error:x y z length all equal 0");
	}
#endif

	/* if (vnode->v.ymax - vnode->v.ymin < POSITIVEZERO) { */
	/* 	printf("y length equal 0"); */
	/* } */
	
	/* if (vnode->v.zmax - vnode->v.zmin < POSITIVEZERO) { */
	/* 	printf("z length equal 0"); */
	/* } */
	
	return 0;
}

void update_volume(const matrix m, const vector3d *v, const volume *from, volume *to)
{
	vector3d updated_vertex[8];
	
	vector3d tmp1 = {from->xmin, from->ymin, from->zmin};
	vector3d tmp2 = {from->xmax, from->ymin, from->zmin};
	vector3d tmp3 = {from->xmin, from->ymax, from->zmin};
	vector3d tmp4 = {from->xmin, from->ymin, from->zmax};
	
	point_transform(m, v, &tmp1, &updated_vertex[0]);
	point_transform(m, v, &tmp2, &updated_vertex[1]);
	point_transform(m, v, &tmp3, &updated_vertex[2]);
	point_transform(m, v, &tmp4, &updated_vertex[3]);

	vector3d buffer1, buffer2, buffer3;
	
	vectorminus(&updated_vertex[1], &updated_vertex[0], &buffer1);
	vectorminus(&updated_vertex[2], &updated_vertex[0], &buffer2);
	vectorminus(&updated_vertex[3], &updated_vertex[0], &buffer3);

	vectoradd(&buffer1, &updated_vertex[3], &updated_vertex[4]);
	vectoradd(&buffer2, &updated_vertex[1], &updated_vertex[5]);
	vectoradd(&buffer2, &updated_vertex[3], &updated_vertex[6]);
	vectoradd(&buffer3, &updated_vertex[5], &updated_vertex[7]);

	int i;
	vector3d *tmp;

	to->xmin = updated_vertex[0].x;
	to->xmax = updated_vertex[0].x;
	to->ymin = updated_vertex[0].y;
	to->ymax = updated_vertex[0].y;
	to->zmin = updated_vertex[0].z;
	to->zmax = updated_vertex[0].z;

	for (i = 0; i < 8; ++i) {
		tmp = &updated_vertex[i];

		if (tmp->x < to->xmin)
			to->xmin = tmp->x;
		if (tmp->x > to->xmax)
			to->xmax = tmp->x;
		if (tmp->y < to->ymin)
			to->ymin = tmp->y;
		if (tmp->y > to->ymax)
			to->ymax = tmp->y;
		if (tmp->z < to->zmin)
			to->zmin = tmp->z;
		if (tmp->z > to->zmax)
			to->zmax = tmp->z;
	}
}

int recurbuildtree(volumenode *vnode, int depth)
{
	volumenode *left = (volumenode *)malloc(sizeof(volumenode));
	volumenode *right = (volumenode *)malloc(sizeof(volumenode));
	
#ifdef DEBUG
	if (left == NULL || right == NULL) {
		printf("recurbuildtree malloc error!\n");
		return -1;
	}
#endif
	
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

		left->parent = vnode;
		right->parent = vnode;
		
		vnode->child1 = left;
		vnode->child2 = right;
		left->m = vnode->m;
		left->vector = vnode->vector;
		right->m = vnode->m;
		right->vector = vnode->vector;
		
		free(vnode->tindex);
//		vnode->trianglenum = 0;
#ifdef DEBUG
		volumecount += 2;
#endif
		if (left->trianglenum > MAXTRIANGLE) {
			if (recurbuildtree(left, depth + 1) == -1) {
				printf("recurbuildtree(left) error! left->trianglenum = %d\n",
				       left->trianglenum);
				return -1;
			}
		} else {
			left->last = 1;
#ifdef DEBUG
			last_count++;
#endif
		}

		if (right->trianglenum > MAXTRIANGLE) {
			if (recurbuildtree(right, depth + 1) == -1) {
				printf("recurbuildtree(right) error! right->trianglenum = %d\n",
				       right->trianglenum);
				return -1;
			}
		} else {
			right->last = 1;
#ifdef DEBUG
			last_count++;
#endif
		}

		return 0;
	case 1:
		free(left);
		free(right);
		vnode->last = 1;
#ifdef DEBUG
		last_count++;
#endif
		vnode->child1 = NULL;
		vnode->child2 = NULL;
//		printf("depth = %d\n", depth);

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
	/* if (parent->trianglenum < 1) { */
	/* 	printf("parent->trianglenum = %d\n", parent->trianglenum); */
	/* 	return 1; */
	/* } */

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

#ifdef DEBUG
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
#endif

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
			printf("      ");

		printf("%d\n", vnode->child1->trianglenum);
	}

	if (vnode->child2 != NULL && vnode->child2->trianglenum != 0) { 
		for (i = 0; i < depth; ++i) 
			printf("      ");

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

int collision_detection_recur(const volumenode *left_node, const volumenode *right_node)
{
	if (left_node == NULL || right_node == NULL) {
		return 0;
	}
	
#ifdef DEBUG
	cdcount++;
#endif
	
	int tmp;
	int i, j;
	int collision_flag = 0;

	volume volume_tmp1, volume_tmp2;
	
#ifdef DEBUG
	if (left_node->m == NULL || right_node->m == NULL)
		printf("error!\n");
#endif

//	update_volume((double (*)[3])left_node->m, left_node->vector, &left_node->v, &volume_tmp1);
	update_volume((double (*)[3])right_node->m, right_node->vector, &right_node->v, &volume_tmp2);
	
	triangle triangle_tmp1, triangle_tmp2;

	if (volumecd(&left_node->v, &volume_tmp2)) {
		tmp = left_node->last + right_node->last * 2;
		switch(tmp) {
		case 0:
			return (collision_detection_recur(left_node->child1, right_node->child1) ||
				collision_detection_recur(left_node->child1, right_node->child2) ||
				collision_detection_recur(left_node->child2, right_node->child1) ||
				collision_detection_recur(left_node->child2, right_node->child2));
		case 1:
			return (collision_detection_recur(left_node, right_node->child1) ||
				collision_detection_recur(left_node, right_node->child2));
		case 2:
			return (collision_detection_recur(left_node->child1, right_node) ||
				collision_detection_recur(left_node->child2, right_node));
		case 3:
#ifdef DEBUG
			if (left_node->trianglenum < 1 || right_node->trianglenum < 1) 
				printf("number error!\n");
#endif		
			for (i = 0; i < left_node->trianglenum; ++i) {
				for (j = 0; j < right_node->trianglenum; ++j) {
					/* triangle_transform((double (*)[3])left_node->m, left_node->vector, */
					/* 		   &left_node->tarry[left_node->tindex[i]], &triangle_tmp1); */
					triangle_transform((double (*)[3])right_node->m, right_node->vector,
							   &right_node->tarry[right_node->tindex[j]], &triangle_tmp2);
#ifdef DEBUG
					triangle_cd_count++;
#endif
					
					collision_flag = triangleCD(&left_node->tarry[left_node->tindex[i]], &triangle_tmp2);
					if (collision_flag) {
#ifdef DEBUG
						printf("triangle collision!index1 = %d, index2 = %d\n",
						       left_node->tindex[i], right_node->tindex[j]);
						show_triangle(left_node->tarry, left_node->tindex[i]);
						show_triangle(right_node->tarry, right_node->tindex[j]);
#endif
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
#ifdef DEBUG
			printf("vnode->last error!left_node->last = %d, right_node->last = %d\n",
			       left_node->last, right_node->last);
#endif

			return 2;
		}
	}

#ifdef DEBUG
//	printf("novolumecd!\n");
#endif

	return 0;
}

int cd_init(char *path1, volumenode *left_top_node,
	    char *path2, volumenode *right_top_node)
{
	stldata left_model, right_model;
	
	if (loadstl(path1, &left_model) != 0) {
		printf("loadstl %s error!\n", path1);
		return 1;
	}
	
	if (loadstl(path2, &right_model) != 0) {
		printf("loadstl %s error!\n", path2);
		return 1;
	}

	left_top_node = (volumenode *)malloc(sizeof(volumenode));
	right_top_node = (volumenode *)malloc(sizeof(volumenode));

	left_top_node->tarry = left_model.ptriangle;
	left_top_node->tarraysize = left_model.num;
	left_top_node->tindex = (int *)malloc(sizeof(int) * left_model.num);

	right_top_node->tarry = right_model.ptriangle;
	right_top_node->tarraysize = right_model.num;
	right_top_node->tindex = (int *)malloc(sizeof(int) * right_model.num);
	
	int i;
	
	for (i = 0; i < left_model.num; ++i)
		left_top_node->tindex[i] = i;
	
	for (i = 0; i < right_model.num; ++i)
		right_top_node->tindex[i] = i;

	left_top_node->trianglenum = left_model.num;
	buildvolume(left_top_node);
	left_top_node->parent = NULL;

	right_top_node->trianglenum = right_model.num;
	buildvolume(right_top_node);
	right_top_node->parent = NULL;

	left_top_node->m = NULL;
	left_top_node->vector = NULL;
	right_top_node->m = (double *)malloc(sizeof(matrix));
	right_top_node->vector = (vector3d *)malloc(sizeof(vector3d));

	if (recurbuildtree(left_top_node, 0) == -1) 
		printf("recurbuildtree error!\n");

	if (recurbuildtree(right_top_node, 0) == -1) 
		printf("recurbuildtree error!\n");
}

int collision_detection1(volumenode *left_node, const matrix m1, const vector3d *v1,
			 volumenode *right_node, const matrix m2, const vector3d *v2)
{
	if (left_node->trianglenum < right_node->trianglenum) {
		volumenode *node_swap;
		double *m_swap;
		vector3d *v_swap;
		
		node_swap = left_node;
		left_node = right_node;
		right_node = node_swap;

		m_swap = (double *)m1;
		m1 = m2;
		m2 = (matrix)m_swap;

		v_swap = v1;
		v1 = v2;
		v2 = v_swap;
	}
	
	int i, j;
	matrix tmp_m1;
	matrix tmp_m2;
	vector3d tmp_v1;
	vector3d tmp_v2;

	for (i = 0; i < 3; ++i)
		for (j = 0; j < 3; ++j)
			tmp_m1[i][j] = m1[i][j];

	for (i = 0; i < 3; ++i)
		for (j = 0; j < 3; ++j)
			tmp_m2[i][j] = m2[i][j];
	


	vector3d buf_v1;
	vector3d buf_v2;
	
	vector3d zero = {0.0, 0.0, 0.0};



	transpose(tmp_m1);
	matrix_multiply(tmp_m1, tmp_m2, (matrix)right_node->m);

	tmp_v1.x = -v1->x;
	tmp_v1.y = -v1->y;
	tmp_v1.z = -v1->z;
	tmp_v2.x = v2->x;
	tmp_v2.y = v2->y;
	tmp_v2.z = v2->z;
		
	point_transform(tmp_m1, &zero, &tmp_v1, &buf_v1);
	point_transform(tmp_m1, &buf_v1, tmp_v2, right_node->vector);
	
	return collision_detection_recur(left_node, right_node);	
}

int collision_detection2(volumenode *left_node,	volumenode *right_node,
			 const matrix r2l_m, const vector3d *r2l_v)
{
	if (left_node->trianglenum < right_node->trianglenum) {
		
	}
	
	int i, j;
	
	for (i = 0; i < 3; ++i)
		for (j = 0; j < 3; ++j)
			right_node->m[i][j] = r2l_m[i][j];

	right_node->vector.x = r2l_v->x;
	right_node->vector.y = r2l_v->y;
	right_node->vector.z = r2l_v->z;
	
	return collision_detection_recur(left_node, right_node);
}

int cd_finish(volumenode *vnode)
{
	if (vnode == NULL)
		return 0;

	if (vnode->last == 1)
		free(vnode->tindex);

	cd_finish(vnode->child1);
	cd_finish(vnode->child2);
	free(vnode);
}







