// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module visualOdometry
%{
    #include "visualOdometry.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "visualOdometry.h"

%include <architecture/msgPayloadDef/PairedKeyPointsMsgPayload.h>
%include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
