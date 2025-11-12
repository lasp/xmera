%module ExtPulsedTorque
%{
   #include "ExtPulsedTorque.h"
%}

%include <std_string.i>
%include <stdint.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
%include "ExtPulsedTorque.h"
