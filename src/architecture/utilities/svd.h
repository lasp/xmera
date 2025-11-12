#ifndef _SVD_H_
#define _SVD_H_

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

int svdcmp(double *mx, size_t dim1, size_t dim2, double *w, double *v);
void solveSVD(double *u, size_t dim1, size_t dim2, double *x, double *b, double minSV);

#ifdef __cplusplus
}
#endif
#endif
