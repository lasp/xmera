%module dvAttEffect
%{
   #include "dvAttEffect.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
STRUCTASLIST(ThrustGroupData)

%include "dvAttEffect.h"

%include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
struct ThrustGroupData;
struct effPairs;
