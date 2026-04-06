// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module pixelLineBiasUKF
%{
   #include "pixelLineBiasUKF.h"
   #include <architecture/utilities/ukfUtilities.h>
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "pixelLineBiasUKF.h"

%include <architecture/utilities/ukfUtilities.h>

%include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/PixelLineFilterMsgPayload.h>
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavCirclesMsgPayload.h>
