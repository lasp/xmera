// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

%module thrFiringRemainder_C
%{
   #include "thrFiringRemainder_C.h"
%}

%include <attribute.i>
%attribute(ThrFiringRemainder_C, double, thrMinFireTime, getThrMinFireTime, setThrMinFireTime)
%attribute(ThrFiringRemainder_C, double, baseThrustState, getBaseThrustState, setBaseThrustState)
%attribute(ThrFiringRemainder_C, double, defaultControlPeriod, getDefaultControlPeriod, setDefaultControlPeriod)

%include <sys_model.i>
%include <swig_conly_data.i>

%include "thrFiringRemainder_C.h"

%include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
