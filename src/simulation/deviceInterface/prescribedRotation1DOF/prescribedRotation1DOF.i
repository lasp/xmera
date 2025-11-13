%module prescribedRotation1DOF
%{
    #include "prescribedRotation1DOF.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "prescribedRotation1DOF.h"

%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
%include <architecture/msgPayloadDef/PrescribedRotationMsgPayload.h>
