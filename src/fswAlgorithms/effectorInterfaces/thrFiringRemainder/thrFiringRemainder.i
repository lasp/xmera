%module thrFiringRemainder
%{
   #include "thrFiringRemainder.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <attribute.i>
%attribute(ThrFiringRemainder, double, thrMinFireTime, getThrMinFireTime, setThrMinFireTime)
%attribute(ThrFiringRemainder, ThrustPulsingRegime, thrustPulsingRegime, getThrustPulsingRegime, setThrustPulsingRegime)
%attribute(ThrFiringRemainder, double, defaultControlPeriod, getDefaultControlPeriod, setDefaultControlPeriod)

%include "thrFiringRemainder.h"
%include "thrFiringRemainderTypes.h"

%include <architecture/msgPayloadDef/THRArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayCmdForceMsgPayload.h>
%include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>
