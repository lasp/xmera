// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module opticalFlow
%{
   #include "opticalFlow.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_array.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "opticalFlow.h"

%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>

%include <architecture/msgPayloadDef/PairedKeyPointsMsgPayload.h>
