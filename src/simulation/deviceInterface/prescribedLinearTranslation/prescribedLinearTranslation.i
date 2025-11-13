%module prescribedLinearTranslation
%{
   #include "prescribedLinearTranslation.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "prescribedLinearTranslation.h"

%include <architecture/msgPayloadDef/PrescribedTranslationMsgPayload.h>
%include <architecture/msgPayloadDef/LinearTranslationRigidBodyMsgPayload.h>
