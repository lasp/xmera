// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module kalmanFilter
%{
   #include <fswAlgorithms/_GeneralModuleFiles/kalmanFilter.h>
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>

%typemap(out) std::optional<Eigen::VectorXd> %{
    if ($1.has_value()) {
        std::optional<Eigen::VectorXd> &tmp_ov = $1;
        {
            Eigen::VectorXd result = tmp_ov.value();
            $typemap(out, Eigen::VectorXd)
        }
    } else {
        $result = Py_None;
        Py_INCREF($result);
    }
%}

%include <fswAlgorithms/_GeneralModuleFiles/kalmanFilter.h>
