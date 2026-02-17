// SPDX-License-Identifier: ISC
// Copyright (c) 2019, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module headingSuKF
%{
   #include "headingSuKF.h"
   #include <architecture/utilities/ukfUtilities.h>
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "headingSuKF.h"


%include <architecture/utilities/ukfUtilities.h>

%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/HeadingFilterMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavMsgPayload.h>
%include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
