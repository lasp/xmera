%module rwMotorVoltage
%{
   #include "rwMotorVoltage.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "rwMotorVoltage.h"
%include "rwMotorVoltageAlgorithm.h"

%include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
%include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/RwMotorVoltageMsgPayload.h>

%include <fswAlgorithms/fswUtilities/fswDefinitions.h>
%include <architecture/utilities/macroDefinitions.h>
