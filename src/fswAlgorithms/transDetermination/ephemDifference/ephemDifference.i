%module ephemDifference
%{
   #include "ephemDifference.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
STRUCTASLIST(EphemChangeConfig)
%include "ephemDifference.h"

%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>

%include <std_array.i>
%template(EphemChangeConfigArray10) std::array<EphemChangeConfig,MAX_NUM_CHANGE_BODIES>;
