// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef DRAG_DYNAMIC_EFFECTOR_H
#define DRAG_DYNAMIC_EFFECTOR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
#include <Eigen/Dense>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AtmoPropsMsgPayload.h>

#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>

//! @brief Container for basic drag parameters - the spacecraft's atmosphere-relative velocity, its projected area, and
//! its drag coefficient.
typedef struct {
    double projectedArea;       //!< m^2   Area of spacecraft projected in velocity direction
    double dragCoeff;           //!< --  Nondimensional drag coefficient
    Eigen::Vector3d comOffset;  //!< m distance from center of mass to center of projected area
} DragBaseData;

/*! @brief drag dynamic effector */
class DragDynamicEffector : public SysModel, public DynamicEffector {
   public:
    void linkInStates(DynParamManager& states) override;
    void computeForceTorque(double integTime, double timeStep) override;
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;
    void readMessages();
    void cannonballDrag();
    void updateDragDir();

    DragBaseData coreParams{};                       //!< -- Struct used to hold drag parameters
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;  //!< -- message used to read density inputs
    std::string modelType = "cannonball";            //!< -- String used to set the type of model used to compute drag
    StateData* hubSigma;                             //!< -- Hub/Inertial attitude represented by MRP
    StateData* hubVelocity;                          //!< m/s Hub inertial velocity vector
    Eigen::Vector3d v_B = Eigen::Vector3d::Zero();   //!< m/s local variable to hold the inertial velocity
    Eigen::Vector3d v_hat_B = Eigen::Vector3d::Zero();  //!< -- Drag force direction in the inertial frame

   private:
    AtmoPropsMsgPayload atmoInData;
};

#endif /* THRUSTER_DYNAMIC_EFFECTOR_H */
