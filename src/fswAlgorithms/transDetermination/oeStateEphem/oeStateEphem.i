%module oeStateEphem
%{
   #include "oeStateEphem.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_array.i>
%template(DoubleArray20) std::array<double, 20>;
%include "oeStateEphem.h"

%include <architecture/msgPayloadDef/TDBVehicleClockCorrelationMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
