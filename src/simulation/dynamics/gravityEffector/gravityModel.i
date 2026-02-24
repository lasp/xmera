// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module(package="xmera.simulation") gravityModel
%{
   #include <simulation/dynamics/_GeneralModuleFiles/gravityModel.h>
   #include <memory>
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <std_shared_ptr.i>
%shared_ptr(GravityModel)

%typemap(out) std::optional<std::string> {
    if ($1.has_value()) {
        $result = PyUnicode_FromString($1.value().c_str());
    } else {
        $result = Py_None;
        Py_INCREF($result);
    }
}

%include <simulation/dynamics/_GeneralModuleFiles/gravityModel.h>
