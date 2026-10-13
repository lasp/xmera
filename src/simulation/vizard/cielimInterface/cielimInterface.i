// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module cielimInterface
%{
    #include "cielimInterface.h"
    #include "../_GeneralModuleFiles/vizStructures.h"
%}

%include "exception.i"

%exception {
    try {
        $action
    } catch (const std::invalid_argument& e) {
        SWIG_exception(SWIG_ValueError, e.what());
    } catch (const std::exception& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
    }
}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
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
