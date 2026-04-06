// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module simpleStorageUnit
%{
#include "simpleStorageUnit.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <swig_eigen.i>
%include <std_vector.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <stdint.i>

//When using scientific notation in Python (1E9), it is interpreted as float
// giving a type error when assigning storageCapacity or using setDataBuffer.
// This maps that float to int64_t in C++ in this module.
%typemap(in) int64_t {
    $1 = static_cast<int64_t>(PyFloat_AsDouble($input));
}

%include <simulation/onboardDataHandling/_GeneralModuleFiles/dataStorageUnitBase.h>
%include "simpleStorageUnit.h"
%include <architecture/msgPayloadDef/DataNodeUsageMsgPayload.h>

%include <architecture/msgPayloadDef/DataStorageStatusMsgPayload.h>
