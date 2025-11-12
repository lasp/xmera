%module hingedRigidBodyPIDMotor
%{
   #include "hingedRigidBodyPIDMotor.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "hingedRigidBodyPIDMotor.h"

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
%include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>
