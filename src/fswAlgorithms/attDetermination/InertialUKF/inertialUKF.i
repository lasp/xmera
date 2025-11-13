%module inertialUKF
%{
   #include "inertialUKF.h"
   #include <architecture/utilities/ukfUtilities.h>
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

STRUCTASLIST(LowPassFilterData)
STRUCTASLIST(STMessage)

%include "inertialUKF.h"
%include <architecture/utilities/ukfUtilities.h>
%include <architecture/utilities/signalCondition.h>

%include <architecture/msgPayloadDef/InertialFilterMsgPayload.h>
%include <architecture/msgPayloadDef/STAttMsgPayload.h>
%include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/AccDataMsgPayload.h>
%include <architecture/msgPayloadDef/AccPktDataMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
