%module prescribedRot2DOF
%{
   #include "prescribedRot2DOF.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "prescribedRot2DOF.h"

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>

%include <architecture/msgPayloadDef/PrescribedRotationMsgPayload.h>
