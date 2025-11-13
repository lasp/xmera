%module chebyPosEphem
%{
   #include "chebyPosEphem.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

STRUCTASLIST(ChebyEphemRecord)
%include "chebyPosEphem.h"

%include <architecture/msgPayloadDef/TDBVehicleClockCorrelationMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
