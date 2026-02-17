// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module scalingIterativeClosestPoint
%{
   #include "scalingIterativeClosestPoint.h"
%}

%include <stdint.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_array.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "scalingIterativeClosestPoint.h"

%include <architecture/msgPayloadDef/SICPMsgPayload.h>
%include <architecture/msgPayloadDef/PointCloudMsgPayload.h>
