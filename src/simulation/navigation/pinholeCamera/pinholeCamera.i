%module pinholeCamera
%{
    #include "pinholeCamera.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "pinholeCamera.h"
%include <std_vector.i>


%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/LandmarkMsgPayload.h>
