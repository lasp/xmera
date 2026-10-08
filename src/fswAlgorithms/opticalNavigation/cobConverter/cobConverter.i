// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module cobConverter
%{
   #include "cobConverter.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <attribute.i>
%attribute(CobConverter, double, radius, getRadius, setRadius)
%attribute(CobConverter, double, radiusUncertainty, getRadiusUncertainty, setRadiusUncertainty)
%attribute(CobConverter, Eigen::Matrix3d, attitudeCovariance, getAttitudeCovariance, setAttitudeCovariance)
%attribute(CobConverter, double, numStandardDeviations, getNumStandardDeviations, setNumStandardDeviations)
%attribute(CobConverter, double, standardDeviation, getStandardDeviation, setStandardDeviation)
%attributeval(CobConverter, CalibrationCoefficients, calibrationCoefficients, getBrownConradyCoefficients, setBrownConradyCoefficients)

%include "cobConverter.h"
%include "cobConverterAlgorithm.h"

%include <architecture/msgPayloadDef/CameraModelMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavCOBMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavCOMMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/CobConverterDiagnosticMsgPayload.h>
