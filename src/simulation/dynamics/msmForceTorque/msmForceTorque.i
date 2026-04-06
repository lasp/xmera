// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module msmForceTorque
%{
    #include "msmForceTorque.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%template(Eigen3dVector) std::vector<Eigen::Vector3d, std::allocator<Eigen::Vector3d>>;
%pythoncode %{
    def npList2EigenXdVector(list):
        """Convert a list of arrays to a list of eigen values"""
        eigenList = Eigen3dVector()
        for pos in list:
            eigenList.push_back(pos)
        return eigenList
%}

%import <architecture/_GeneralModuleFiles/sys_model.i>
%include "msmForceTorque.h"

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/VoltMsgPayload.h>

%include <architecture/msgPayloadDef/CmdTorqueBodyMsgPayload.h>

%include <architecture/msgPayloadDef/CmdForceInertialMsgPayload.h>

%include <architecture/msgPayloadDef/ChargeMsmMsgPayload.h>
