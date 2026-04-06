// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module cielimInterface
%{
    #include "cielimInterface.h"
    #include "../_GeneralModuleFiles/vizStructures.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_vector.i>

%include "cielimInterface.h"

%include <architecture/msgPayloadDef/CameraModelMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavCOBMsgPayload.h>
%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
%include <architecture/msgPayloadDef/ImageDiagnosticsPayload.h>
%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
%include <architecture/msgPayloadDef/EpochMsgPayload.h>

%include <architecture/msgPayloadDef/CelestialBodyParametersMsgPayload.h>
%include <architecture/msgPayloadDef/CameraRenderingMsgPayload.h>
