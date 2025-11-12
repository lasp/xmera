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
