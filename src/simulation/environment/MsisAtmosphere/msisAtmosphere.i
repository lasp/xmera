%module msisAtmosphere
%{
   #include "msisAtmosphere.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <simulation/environment/_GeneralModuleFiles/atmosphereBase.h>
%include "msisAtmosphere.h"
#include "nrlmsise-00.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/AtmoPropsMsgPayload.h>

%include <architecture/msgPayloadDef/SwDataMsgPayload.h>

%include <architecture/msgPayloadDef/EpochMsgPayload.h>


GEN_SIZEOF(ap_array)
GEN_SIZEOF(nrlmsise_input)
GEN_SIZEOF(nrlmsise_flags)
GEN_SIZEOF(nrlmsise_output)
