%module constrainedAttitudeManeuver
%{
   #include "constrainedAttitudeManeuver.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "constrainedAttitudeManeuver.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
