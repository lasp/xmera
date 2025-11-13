%module magneticFieldCenteredDipole
%{
    #include "magneticFieldCenteredDipole.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <simulation/environment/_GeneralModuleFiles/magneticFieldBase.h>
%include "magneticFieldCenteredDipole.h"

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/MagneticFieldMsgPayload.h>
