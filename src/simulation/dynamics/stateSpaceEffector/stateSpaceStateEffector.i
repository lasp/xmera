%module stateSpaceStateEffector
%{
   #include "stateSpaceStateEffector.h"
%}

%include <std_string.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include "stateSpaceStateEffector.h"
