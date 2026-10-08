// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module flybyPoint
%{
   #include "flybyPoint.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <attribute.i>
%attribute(FlybyPoint, double, timeBetweenFilterData, getTimeBetweenFilterData, setTimeBetweenFilterData)
%attribute(FlybyPoint, double, toleranceForCollinearity, getToleranceForCollinearity, setToleranceForCollinearity)
%attribute(FlybyPoint, int, signOfOrbitNormalFrameVector, getSignOfOrbitNormalFrameVector, setSignOfOrbitNormalFrameVector)
%attribute(FlybyPoint, double, maximumAccelerationThreshold, getMaximumAccelerationThreshold, setMaximumAccelerationThreshold)
%attribute(FlybyPoint, double, maximumRateThreshold, getMaximumRateThreshold, setMaximumRateThreshold)
%attribute(FlybyPoint, double, positionKnowledgeSigma, getPositionKnowledgeSigma, setPositionKnowledgeSigma)

%include "flybyPoint.h"

%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/FlybyDiagnosticMsgPayload.h>
