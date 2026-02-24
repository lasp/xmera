// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef _UKF_UTILITIES_H_
#define _UKF_UTILITIES_H_

#include <stdint.h>
#include <string.h>
#include <architecture/utilities/bskLogging.h>

#define UKF_MAX_DIM 20

#ifdef __cplusplus
extern "C" {
#endif

void ukfQRDJustR(double* sourceMat, int32_t nRow, int32_t nCol, double* destMat);
int32_t ukfLInv(double* sourceMat, int32_t nRow, int32_t nCol, double* destMat);
int32_t ukfUInv(double* sourceMat, int32_t nRow, int32_t nCol, double* destMat);
int32_t ukfLUD(double* sourceMat, int32_t nRow, int32_t nCol, double* destMat, int32_t* indx);
int32_t ukfLUBckSlv(double* sourceMat, int32_t nRow, int32_t nCol, int32_t* indx, double* bmat, double* destMat);
int32_t ukfMatInv(double* sourceMat, int32_t nRow, int32_t nCol, double* destMat);
int32_t ukfCholDecomp(double* sourceMat, int32_t nRow, int32_t nCol, double* destMat);
int32_t ukfCholDownDate(double* rMat, double* xVec, double beta, int32_t nStates, double* rOut);

#ifdef __cplusplus
}
#endif

#endif
