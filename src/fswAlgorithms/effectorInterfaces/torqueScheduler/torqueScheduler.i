%module torqueScheduler
%{
   #include "torqueScheduler.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "torqueScheduler.h"

%include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>
%include <architecture/msgPayloadDef/ArrayEffectorLockMsgPayload.h>
