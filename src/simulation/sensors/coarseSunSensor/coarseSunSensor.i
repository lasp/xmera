%module coarseSunSensor
%{
   #include "coarseSunSensor.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <std_vector.i>
%include <std_string.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "coarseSunSensor.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/CSSRawDataMsgPayload.h>

%include <architecture/msgPayloadDef/AlbedoMsgPayload.h>

%include <architecture/msgPayloadDef/EclipseMsgPayload.h>

%include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>


%include <architecture/msgPayloadDef/CSSConfigLogMsgPayload.h>

namespace std {
    %template(CSSVector) vector<CoarseSunSensor *, allocator<CoarseSunSensor *> >;
}
