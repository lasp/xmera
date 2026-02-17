// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module sunlineUKF
%{
   #include "sunlineUKF.h"
   #include <architecture/utilities/ukfUtilities.h>
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "sunlineUKF.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
%include <architecture/msgPayloadDef/SunlineFilterMsgPayload.h>
%include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>

%include <architecture/utilities/ukfUtilities.h>
