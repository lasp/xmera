%module sunSearch
%{
   #include "sunSearch.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>

STRUCTASLIST(SlewProperties)

%include "sunSearch.h"
%include "sunSearchAlgorithm.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
