%module ephemeridesRecenter
%{
   #include "ephemeridesRecenter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <std_string.i>
%include <std_array.i>
%template(StringArray10) std::array<std::string, MAX_NUM_CHANGE_BODIES>;

%include "ephemeridesRecenter.h"
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
