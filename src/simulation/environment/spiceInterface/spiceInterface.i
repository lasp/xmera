%module spiceInterface
%{
   #include "spiceInterface.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_string.i>
%include <std_vector.i>

%template() std::vector<std::string>;

// Declaring planetFrames %naturalvar removes the need for the StringVector wrapper:
//    mySpiceInterface.planetFrames = ["a", "b", "c"]
// is allowed, which is more pythonic than:
//    mySpiceInterface.planetFrames = spiceInterface.StringVector(["a", "b", "c"])
// (which is also allowed)
// However, modifiying in place is forbidden:
//    mySpiceInterface.planetFrames[2] = "bb"
// this raises an error because mySpiceInterface.planetFrames is returned by value
%naturalvar SpiceInterface::planetFrames;

%include <architecture/_GeneralModuleFiles/sys_model.i>

%include "spiceInterface.h"

%include <architecture/msgPayloadDef/EpochMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SpiceTimeMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>

%include <architecture/msgPayloadDef/TransRefMsgPayload.h>
