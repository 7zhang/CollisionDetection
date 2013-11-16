#ifndef _VOLUME_H_
#define _VOLUME_H_

#include "geom.h"

typedef struct _volume {
	double xmin, xmax, ymin, ymax, zmin, zmax;
}volume;

typedef struct _volumenode {
//const element during collision detection
	triangle *tarry;
	int tarraysize;
	int *tindex;
	int trianglenum;
	int depth;
	volume v;
	int last;

	struct _volumenode *parent;
	struct _volumenode *child1, *child2;
	struct _volumenode *left, *right;

//mutable element during collision detection
	int cdflag;
	struct _volumenode *cdvolumelist;
	int volumenum;
}volumenode;

//return 0 if no collision
static inline int volumecd(const volume *v1, const volume *v2)
{
	return 1;
	if (v1->xmax < v2->xmin || v1->xmin > v2->xmax)
		return 0;
	if (v1->ymax < v2->ymin || v1->ymin > v2->ymax)
		return 0;
	if (v1->zmax < v2->zmin || v1->zmin > v2->zmax)
		return 0;
	return 1;
}

//build the volume
int buildvolume(volumenode *vnode);

//build the volume tree recursively
//return the depth if success, -1 if error
int recurbuildtree(volumenode *vnode, int depth);

//0 if success, 1 if false, 2 if error
int triangleallocation(const volumenode *parent, volumenode *left, volumenode *right);

//recursive cd
int collision_detection_recur(const volumenode *vnode1, const volumenode *vnode2);

//show triangle data
void show_triangle(triangle *t, int index);

//show the volume tree
void recurshowtree(const volumenode *vnode, int depth);

extern int volumecount;
extern int maxdepth;
extern int cdcount;
extern int last_count;
extern int triangle_cd_count;

#endif /* _VOLUME_H_ */



















