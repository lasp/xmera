// SPDX-License-Identifier: ISC
// Copyright (c) 2020, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module mrpFeedback
%{
   #include "mrpFeedback.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "mrpFeedback.h"
%include "mrpFeedbackAlgorithm.h"

%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
