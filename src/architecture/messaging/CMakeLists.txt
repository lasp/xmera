add_library(messaging INTERFACE)
add_library(Xmera::Messaging ALIAS messaging)

target_sources(messaging
  PUBLIC FILE_SET HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    messaging.h
    msgHeader.h
    ../msgPayloadDef/definitions.h

  PUBLIC FILE_SET message_headers
  TYPE HEADERS
  BASE_DIRS "${CMAKE_SOURCE_DIR}"
  FILES
    ../msgPayloadDef/AccDataMsgPayload.h
    ../msgPayloadDef/AccPktDataMsgPayload.h
    ../msgPayloadDef/AccessMsgPayload.h
    ../msgPayloadDef/AlbedoMsgPayload.h
    ../msgPayloadDef/ArrayEffectorLockMsgPayload.h
    ../msgPayloadDef/ArrayMotorForceMsgPayload.h
    ../msgPayloadDef/ArrayMotorTorqueMsgPayload.h
    ../msgPayloadDef/AtmoPropsMsgPayload.h
    ../msgPayloadDef/AttGuidMsgPayload.h
    ../msgPayloadDef/AttRefMsgPayload.h
    ../msgPayloadDef/AttStateMsgPayload.h
    ../msgPayloadDef/BodyHeadingMsgPayload.h
    ../msgPayloadDef/BoreAngleMsgPayload.h
    ../msgPayloadDef/CMEstDataMsgPayload.h
    ../msgPayloadDef/CSSArraySensorMsgPayload.h
    ../msgPayloadDef/CSSConfigLogMsgPayload.h
    ../msgPayloadDef/CSSConfigMsgPayload.h
    ../msgPayloadDef/CSSRawDataMsgPayload.h
    ../msgPayloadDef/CSSUnitConfigMsgPayload.h
    ../msgPayloadDef/CameraConfigMsgPayload.h
    ../msgPayloadDef/CameraImageMsgPayload.h
    ../msgPayloadDef/CameraLocalizationMsgPayload.h
    ../msgPayloadDef/CameraModelMsgPayload.h
    ../msgPayloadDef/CameraRenderingMsgPayload.h
    ../msgPayloadDef/CelestialBodyParametersMsgPayload.h
    ../msgPayloadDef/ChargeMsmMsgPayload.h
    ../msgPayloadDef/ClassicElementsMsgPayload.h
    ../msgPayloadDef/CmdForceBodyMsgPayload.h
    ../msgPayloadDef/CmdForceInertialMsgPayload.h
    ../msgPayloadDef/CmdTorqueBodyMsgPayload.h
    ../msgPayloadDef/ColorMsgPayload.h
    ../msgPayloadDef/DataNodeUsageMsgPayload.h
    ../msgPayloadDef/DataStorageStatusMsgPayload.h
    ../msgPayloadDef/DesiredVelocityMsgPayload.h
    ../msgPayloadDef/DeviceCmdMsgPayload.h
    ../msgPayloadDef/DeviceStatusMsgPayload.h
    ../msgPayloadDef/DipoleRequestBodyMsgPayload.h
    ../msgPayloadDef/DirectionOfMotionMsgPayload.h
    ../msgPayloadDef/DvBurnCmdMsgPayload.h
    ../msgPayloadDef/DvExecutionDataMsgPayload.h
    ../msgPayloadDef/EclipseMsgPayload.h
    ../msgPayloadDef/EphemerisMsgPayload.h
    ../msgPayloadDef/EpochMsgPayload.h
    ../msgPayloadDef/FilterMsgPayload.h
    ../msgPayloadDef/FilterResidualsMsgPayload.h
    ../msgPayloadDef/FuelTankMsgPayload.h
    ../msgPayloadDef/GravityGradientMsgPayload.h
    ../msgPayloadDef/GroundStateMsgPayload.h
    ../msgPayloadDef/HeadingFilterMsgPayload.h
    ../msgPayloadDef/HillRelStateMsgPayload.h
    ../msgPayloadDef/HingedRigidBodyMsgPayload.h
    ../msgPayloadDef/IMUSensorBodyMsgPayload.h
    ../msgPayloadDef/IMUSensorMsgPayload.h
    ../msgPayloadDef/InertialFilterMsgPayload.h
    ../msgPayloadDef/InertialHeadingMsgPayload.h
    ../msgPayloadDef/LambertPerformanceMsgPayload.h
    ../msgPayloadDef/LambertProblemMsgPayload.h
    ../msgPayloadDef/LambertSolutionMsgPayload.h
    ../msgPayloadDef/LambertValidatorMsgPayload.h
    ../msgPayloadDef/LandmarkMsgPayload.h
    ../msgPayloadDef/LinearTranslationRigidBodyMsgPayload.h
    ../msgPayloadDef/MTBArrayConfigMsgPayload.h
    ../msgPayloadDef/MTBCmdMsgPayload.h
    ../msgPayloadDef/MTBMsgPayload.h
    ../msgPayloadDef/MagneticFieldMsgPayload.h
    ../msgPayloadDef/ModuleTemplateMsgPayload.h
    ../msgPayloadDef/MotorStepCommandMsgPayload.h
    ../msgPayloadDef/NavAttMsgPayload.h
    ../msgPayloadDef/NavTransMsgPayload.h
    ../msgPayloadDef/OpNavCOBMsgPayload.h
    ../msgPayloadDef/OpNavCOMMsgPayload.h
    ../msgPayloadDef/OpNavCirclesMsgPayload.h
    ../msgPayloadDef/OpNavFilterMsgPayload.h
    ../msgPayloadDef/OpNavLimbMsgPayload.h
    ../msgPayloadDef/OpNavMsgPayload.h
    ../msgPayloadDef/OpNavUnitVecMsgPayload.h
    ../msgPayloadDef/PairedKeyPointsMsgPayload.h
    ../msgPayloadDef/PixelLineFilterMsgPayload.h
    ../msgPayloadDef/PlasmaFluxMsgPayload.h
    ../msgPayloadDef/PointCloudMsgPayload.h
    ../msgPayloadDef/PowerNodeStatusMsgPayload.h
    ../msgPayloadDef/PowerNodeUsageMsgPayload.h
    ../msgPayloadDef/PowerStorageStatusMsgPayload.h
    ../msgPayloadDef/PrescribedMotionMsgPayload.h
    ../msgPayloadDef/PrescribedRotationMsgPayload.h
    ../msgPayloadDef/PrescribedTranslationMsgPayload.h
    ../msgPayloadDef/PyBatteryMsgPayload.h
    ../msgPayloadDef/RWArrayConfigMsgPayload.h
    ../msgPayloadDef/RWAvailabilityMsgPayload.h
    ../msgPayloadDef/RWCmdMsgPayload.h
    ../msgPayloadDef/RWConfigElementMsgPayload.h
    ../msgPayloadDef/RWConfigLogMsgPayload.h
    ../msgPayloadDef/RWConfigMsgPayload.h
    ../msgPayloadDef/RWConstellationMsgPayload.h
    ../msgPayloadDef/RwMotorTorqueMsgPayload.h
    ../msgPayloadDef/RwMotorVoltageMsgPayload.h
    ../msgPayloadDef/RWSpeedMsgPayload.h
    ../msgPayloadDef/RateCmdMsgPayload.h
    ../msgPayloadDef/RealTimeFactorMsgPayload.h
    ../msgPayloadDef/ReconfigBurnArrayInfoMsgPayload.h
    ../msgPayloadDef/ReconfigBurnInfoMsgPayload.h
    ../msgPayloadDef/SCEnergyMomentumMsgPayload.h
    ../msgPayloadDef/SCMassPropsMsgPayload.h
    ../msgPayloadDef/SCStatesMsgPayload.h
    ../msgPayloadDef/SICPMsgPayload.h
    ../msgPayloadDef/STAttMsgPayload.h
    ../msgPayloadDef/STSensorMsgPayload.h
    ../msgPayloadDef/SmallBodyNavMsgPayload.h
    ../msgPayloadDef/SmallBodyNavUKFMsgPayload.h
    ../msgPayloadDef/SolarFluxMsgPayload.h
    ../msgPayloadDef/SpicePlanetStateMsgPayload.h
    ../msgPayloadDef/SpiceTimeMsgPayload.h
    ../msgPayloadDef/StepperMotorMsgPayload.h
    ../msgPayloadDef/SunlineFilterMsgPayload.h
    ../msgPayloadDef/SwDataMsgPayload.h
    ../msgPayloadDef/SynchClockMsgPayload.h
    ../msgPayloadDef/TAMSensorBodyMsgPayload.h
    ../msgPayloadDef/TAMSensorMsgPayload.h
    ../msgPayloadDef/TDBVehicleClockCorrelationMsgPayload.h
    ../msgPayloadDef/THRArrayCmdForceMsgPayload.h
    ../msgPayloadDef/THRArrayConfigMsgPayload.h
    ../msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h
    ../msgPayloadDef/THRConfigMsgPayload.h
    ../msgPayloadDef/THROutputMsgPayload.h
    ../msgPayloadDef/TemperatureMsgPayload.h
    ../msgPayloadDef/TimeClosestApproachMsgPayload.h
    ../msgPayloadDef/TransRefMsgPayload.h
    ../msgPayloadDef/VSCMGArrayTorqueMsgPayload.h
    ../msgPayloadDef/VSCMGCmdMsgPayload.h
    ../msgPayloadDef/VSCMGConfigMsgPayload.h
    ../msgPayloadDef/VSCMGSpeedMsgPayload.h
    ../msgPayloadDef/VehicleConfigMsgPayload.h
    ../msgPayloadDef/VoltMsgPayload.h
)

target_link_libraries(messaging INTERFACE
  Xmera::Core
)

install(TARGETS messaging
  EXPORT xmera
  DESTINATION lib
  FILE_SET HEADERS
    DESTINATION include
  FILE_SET message_headers
    DESTINATION include
)

install(PROGRAMS "msgAutoSource/generateSWIGModules.py"
  DESTINATION bin
)

install(FILES "msgAutoSource/msgInterfacePy.i.in"
  DESTINATION usr/share/xmera
)

install(FILES "newMessaging.ih"
  DESTINATION include/architecture/messaging
)


# Create and build a Python SWIG extension for every message header
get_property(_message_headers TARGET messaging PROPERTY HEADER_SET_message_headers)
foreach(_header IN LISTS _message_headers)
  get_filename_component(_message "${_header}" NAME_WE)
  set_property(TARGET messaging APPEND PROPERTY XMERA_REGISTERED_MESSAGES "${_message}")

  xmera_add_swig_message("${_message}" "${_header}")
endforeach()


# Generate a summary __init__.py exporting all messages in one place
set(_file_content)

foreach(_header IN LISTS _message_headers)
  get_filename_component(_message "${_header}" NAME_WE)
  set_property(TARGET messaging APPEND PROPERTY XMERA_REGISTERED_MESSAGES "${_message}")

  list(APPEND _file_contents "from Basilisk.architecture.messaging.${_message} import *")
endforeach()

list(JOIN _file_contents "\n" _file_contents)

file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/messaging__init__.py"
  CONTENT "${_file_contents}"
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/messaging__init__.py"
  DESTINATION Basilisk/architecture/messaging/
  RENAME __init__.py
)
