%module ephemNavConverter
%{
   #include "ephemNavConverter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include "ephemNavConverter.h"
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
