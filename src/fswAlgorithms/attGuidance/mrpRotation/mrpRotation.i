%module mrpRotation
%{
   #include "mrpRotation.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "mrpRotation.h"
%include "mrpRotationAlgorithm.h"

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/AttStateMsgPayload.h>
