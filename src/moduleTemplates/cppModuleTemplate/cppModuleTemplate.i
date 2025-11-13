%module cppModuleTemplate
%{
   #include "cppModuleTemplate.h"
%}

%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "cppModuleTemplate.h"

%include "architecture/msgPayloadDef/ModuleTemplateMsgPayload.h"
