// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module inertialUKF
%{
   #include "inertialUKF.h"
   #include <architecture/utilities/ukfUtilities.h>
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

STRUCTASLIST(LowPassFilterData)
STRUCTASLIST(STMessage)

%include "inertialUKF.h"
%include <architecture/utilities/ukfUtilities.h>
%include <architecture/utilities/signalCondition.h>

%include <architecture/msgPayloadDef/InertialFilterMsgPayload.h>
%include <architecture/msgPayloadDef/STAttMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/AccDataMsgPayload.h>
%include <architecture/msgPayloadDef/AccPktDataMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
