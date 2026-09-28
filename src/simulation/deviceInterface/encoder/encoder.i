// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module encoder
%{
   #include "encoder.h"
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

%include <architecture/_GeneralModuleFiles/sys_model.i>

%include <stdint.i>
%include <attribute.i>
%attribute(Encoder, std::size_t, numRW, getNumRW, setNumRW)
%attribute(Encoder, std::uint32_t, clicksPerRotation, getClicksPerRotation, setClicksPerRotation)
%include <architecture/utilities/macroDefinitions.h>
%include "encoder.h"

%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
