%module nHingedRigidBodyStateEffector
%{
   #include "nHingedRigidBodyStateEffector.h"
%}

%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <std_string.i>
%include <stdint.i>


%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include "nHingedRigidBodyStateEffector.h"
