%module prescribedMotionStateEffector
%{
   #include "prescribedMotionStateEffector.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/dynamics/_GeneralModuleFiles/stateEffector.h>
%include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
%include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
%include "prescribedMotionStateEffector.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/PrescribedTranslationMsgPayload.h>

%include <architecture/msgPayloadDef/PrescribedRotationMsgPayload.h>
