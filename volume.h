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

/* typedef struct _volumenode { */
/* 	struct _volume *v; */
/* 	struct _volumenode *left, right; */
/* }volumenode; */

//return 0 if no collision
static inline bool volumecd(const volume *v1, const volume *v2)
{
	if (v1->xmax < v2->xmin || v1->xmin > v2->xmax)
		return 0;
	if (v1->ymax < v2->ymin || v1->ymin > v2->ymax)
		return 0;
	if (v1->zmax < v2->zmin || v1->zmin > v2->zmax)
		return 0;
	return 1;
}

void buildvolume(const triangle *t, volume *v);
bool recurbuildtree(volumenode *stl);

//0 if success, 1 if false, 2 if error
int triangleallocation(volumenode *parent, volumenode *left, volumenode *right);
#endif /* _VOLUME_H_ */



















