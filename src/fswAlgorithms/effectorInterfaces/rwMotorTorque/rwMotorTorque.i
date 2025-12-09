%module rwMotorTorque
%{
   #include "rwMotorTorque.h"
%}

%include <attribute.i>
%attribute(RwMotorTorque, Eigen::Matrix3d, controlAxes_B, getControlAxes, setControlAxes)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "rwMotorTorque.h"
%include "rwMotorTorqueAlgorithm.h"

%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>
%include <architecture/msgPayloadDef/RwMotorTorqueMsgPayload.h>
%include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
%include <architecture/msgPayloadDef/RWArrayConfigMsgPayload.h>

%include <fswAlgorithms/fswUtilities/fswDefinitions.h>
