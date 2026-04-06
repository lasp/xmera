// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module thrFiringRemainder
%{
   #include "thrFiringRemainder.h"
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <attribute.i>
%attribute(ThrFiringRemainder, double, thrMinFireTime, getThrMinFireTime, setThrMinFireTime)
%attribute(ThrFiringRemainder, ThrustPulsingRegime, thrustPulsingRegime, getThrustPulsingRegime, setThrustPulsingRegime)
%attribute(ThrFiringRemainder, double, defaultControlPeriod, getDefaultControlPeriod, setDefaultControlPeriod)

%include "thrFiringRemainder.h"
%include "thrFiringRemainderTypes.h"

%include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
