%module inertial3D
%{
   #include "inertial3D.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "inertial3D.h"
%include "inertial3DAlgorithm.h"

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
