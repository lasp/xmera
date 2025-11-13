%module horizonOpNav
%{
   #include "horizonOpNav.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
STRUCTASLIST(HorizonOpNavData)

%include "horizonOpNav.h"

%include <architecture/msgPayloadDef/OpNavLimbMsgPayload.h>
%include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavMsgPayload.h>
