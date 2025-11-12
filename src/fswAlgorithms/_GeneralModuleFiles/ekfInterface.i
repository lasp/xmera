// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module ekfInterface
%{
   #include <fswAlgorithms/_GeneralModuleFiles/ekfInterface.h>
%}

%include <fswAlgorithms/_GeneralModuleFiles/kalmanFilter.i>

%template(KalmanFilter) KalmanFilter<EkfMeasurementModel<FilterStateVector>>;

%include <fswAlgorithms/_GeneralModuleFiles/ekfInterface.h>
