// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module orbitalMotion
%{
  #include "orbitalMotion.hpp"
%}

%include "std_string.i"
%include "std_vector.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"
%include "orbitalMotion.hpp"
EIGEN_MAT_WRAP(Vector6d, 170)
