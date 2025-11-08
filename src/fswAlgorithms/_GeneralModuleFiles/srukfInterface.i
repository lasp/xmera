// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module srukfInterface
%{
   #include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.h>
%}

%include <fswAlgorithms/_GeneralModuleFiles/kalmanFilter.i>

%template(KalmanFilter) KalmanFilter<SRukfMeasurementModel>;

%include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.h>
