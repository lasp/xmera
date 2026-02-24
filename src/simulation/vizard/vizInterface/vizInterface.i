// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module vizInterface
%{
    #include "vizInterface.h"
    #include "../_GeneralModuleFiles/vizStructures.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_vector.i>

// Instantiate templates used by example
namespace std {
    %template(PointLineConfig) vector<PointLine>;
    %template(LocationConfig) vector<LocationPbMsg *>;
    %template(CustomModelConfig) vector<CustomModel>;
    %template(ActuatorGuiSettingsConfig) vector<ActuatorGuiSettings>;
    %template(InstrumentGuiSettingsConfig) vector<InstrumentGuiSettings>;
    %template(KeepOutInConeConfig) vector<KeepOutInCone>;
    %template(StdCameraConfig) vector<StdCameraSettings>;
    %template(VizSCVector) vector<VizSpacecraftData>;
    %template(ThrClusterVector) vector<ThrClusterMap>;
    %template(GravBodyInfoVector) vector<GravBodyInfo>;
    %template(GenericSensorVector) vector<GenericSensor *>;
    %template(LightVector) vector<Light *>;
    %template(TransceiverVector) vector<Transceiver *>;
    %template(GenericStorageVector) vector<GenericStorage *>;
    %template(MultiSphereVector) vector<MultiSphere *>;
    %template(EllipsoidVector) vector<Ellipsoid *>;
    %template(IntVector) vector<int>;
    %template(StringVector) vector<string>;
}

%include "vizInterface.h"
%include "../_GeneralModuleFiles/vizStructures.h"

%include <architecture/msgPayloadDef/CameraConfigMsgPayload.h>
%include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>
%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
%include <architecture/msgPayloadDef/CameraImageMsgPayload.h>
%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
%include <architecture/msgPayloadDef/EpochMsgPayload.h>
%include <architecture/msgPayloadDef/CSSConfigLogMsgPayload.h>
%include <architecture/msgPayloadDef/THROutputMsgPayload.h>
%include <architecture/msgPayloadDef/ChargeMsmMsgPayload.h>
