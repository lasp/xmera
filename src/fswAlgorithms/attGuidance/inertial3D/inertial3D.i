// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module inertial3D
%{
   #include "inertial3D.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "inertial3D.h"
%include "inertial3DAlgorithm.h"

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
