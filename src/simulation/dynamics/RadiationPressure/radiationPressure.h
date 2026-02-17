// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef RADIATION_PRESSURE_H
#define RADIATION_PRESSURE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/EclipseMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

typedef enum { SRP_CANNONBALL_MODEL, SRP_FACETED_CPU_MODEL } srpModel_t;

//  SRP effects on body
/*! @brief solar radiation pressure dynamic effector */
class RadiationPressure : public SysModel, public DynamicEffector {
   public:
    RadiationPressure();
    ~RadiationPressure();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void linkInStates(DynParamManager& statesIn);
    void readInputMessages();
    void computeForceTorque(double integTime, double timeStep);
    void setUseCannonballModel();
    void setUseFacetedCPUModel();
    void addForceLookupBEntry(Eigen::Vector3d vec);
    void addTorqueLookupBEntry(Eigen::Vector3d vec);
    void addSHatLookupBEntry(Eigen::Vector3d vec);

   private:
    void computeCannonballModel(Eigen::Vector3d rSunB_B);
    void computeLookupModel(Eigen::Vector3d rSunB_B);

   public:
    double area;                                           //!< m^2 Body surface area
    double coefficientReflection;                          //!< -- Factor grouping surface optical properties
    ReadFunctor<SpicePlanetStateMsgPayload> sunEphmInMsg;  //!< -- sun state input message
    ReadFunctor<EclipseMsgPayload> sunEclipseInMsg;        //!< -- (optional) sun eclipse input message
    std::vector<Eigen::Vector3d> lookupForce_B;            //!< -- Force on S/C at 1 AU from sun
    std::vector<Eigen::Vector3d> lookupTorque_B;           //!< -- Torque on S/C
    std::vector<Eigen::Vector3d> lookupSHat_B;             //!< -- S/C to sun unit vector defined in the body frame.
    BSKLogger bskLogger;                                   //!< -- BSK Logging

   private:
    srpModel_t srpModel;                         //!< -- specifies which SRP model to use
    SpicePlanetStateMsgPayload sunEphmInBuffer;  //!< -- Buffer for incoming ephemeris message data
    bool stateRead;                              //!< -- Indicates a succesful read of incoming SC state message data
    EclipseMsgPayload sunVisibilityFactor;       //!< [-] scaling parameter from 0 (fully obscured) to 1 (fully visible)
    StateData* hubR_N;                           //!< -- State data accesss to inertial position for the hub
    StateData* hubSigma;                         //!< -- Hub/Inertial attitude represented by MRP
};

#endif
