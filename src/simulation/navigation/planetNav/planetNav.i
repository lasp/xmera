%module planetNav
%{
    #include "planetNav.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "planetNav.h"

%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
