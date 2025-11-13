%module prescribedTrans
%{
   #include "prescribedTrans.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "prescribedTrans.h"

%include <architecture/msgPayloadDef/PrescribedTranslationMsgPayload.h>

%include <architecture/msgPayloadDef/LinearTranslationRigidBodyMsgPayload.h>
