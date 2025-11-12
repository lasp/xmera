%module simSynch
%{
   #include "simSynch.h"
%}

%include <std_string.i>
%include <stdint.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "simSynch.h"

%include <architecture/msgPayloadDef/SynchClockMsgPayload.h>
