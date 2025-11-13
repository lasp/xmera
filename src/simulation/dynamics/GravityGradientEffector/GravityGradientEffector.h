#ifndef GRAVITY_GRADIENT_DYNAMIC_EFFECTOR_H
#define GRAVITY_GRADIENT_DYNAMIC_EFFECTOR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h>
#include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
#include <Eigen/Dense>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/GravityGradientMsgPayload.h>

/*! @brief gravity gradient gradient module */
class GravityGradientEffector : public SysModel, public DynamicEffector {
   public:
    GravityGradientEffector();
    ~GravityGradientEffector();
    void linkInStates(DynParamManager& states);
    void computeForceTorque(double integTime, double timeStep);
    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    void addPlanetName(std::string planetName);

   public:
    Message<GravityGradientMsgPayload> gravityGradientOutMsg;  //!< output message containing the gravity gradient
    StateData* hubSigma;                                       //!< Hub/Inertial attitude represented by MRP
    StateData* r_BN_N;                       //!< Hub/Inertial position vector in inertial frame components
    Eigen::MatrixXd* ISCPntB_B;              //!< [kg m^2] current spacecraft inertia about point B, B-frame components
    Eigen::MatrixXd* c_B;                    //!< [m] Vector from point B to CoM of s/c in B frame components
    Eigen::MatrixXd* m_SC;                   //!< [kg] mass of spacecraft
    std::vector<Eigen::MatrixXd*> r_PN_N;    //!< [m] vector of inertial planet positions
    std::vector<Eigen::MatrixXd*> muPlanet;  //!< [m^3/s^-2] gravitational constant of planet

    BSKLogger bskLogger;  //!< BSK Logging

   private:
    std::vector<std::string> planetPropertyNames;  //!< Names of planets we want to track
};

#endif /* GRAVITY_GRADIENT_DYNAMIC_EFFECTOR_H */
