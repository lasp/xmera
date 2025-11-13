%module inertial3D_C
%{
   #include "inertial3D_C.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "inertial3D_C.h"

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
