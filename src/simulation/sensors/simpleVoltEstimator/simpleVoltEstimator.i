// SPDX-License-Identifier: ISC
// Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simpleVoltEstimator
%{
   #include "simpleVoltEstimator.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%import <architecture/_GeneralModuleFiles/sys_model.i>

// Expose the fixed-size Gauss-Markov matrices/vectors to Python (get/set as lists or
// numpy arrays, with wrong-size assignments rejected by a ValueError at the boundary).
EIGEN_MAT_WRAP(VoltErrorMatrix, 159)

%include "simpleVoltEstimator.h"

%include <architecture/msgPayloadDef/VoltMsgPayload.h>
