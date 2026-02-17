// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef LAMBERTPLANNER_H
#define LAMBERTPLANNER_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/LambertProblemMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/eigenSupport.h>
#include <vector>

/*! @brief This module creates the LambertProblemMsg to be used for the LambertSolver module
 */
class LambertPlanner : public SysModel {
   public:
    LambertPlanner();
    ~LambertPlanner();

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void useSolverIzzoMethod();
    void useSolverGoodingMethod();

    ReadFunctor<NavTransMsgPayload> navTransInMsg;           //!< translational navigation input message
    Message<LambertProblemMsgPayload> lambertProblemOutMsg;  //!< lambert problem output message

    BSKLogger bskLogger;  //!< -- BSK Logging

    Eigen::Vector3d r_TN_N;  //!< [m] targeted position vector with respect to celestial body at finalTime, in N frame
    double finalTime{};      //!< [s] time at which target position should be reached
    double maneuverTime{};   //!< [s] time at which maneuver should be executed
    double mu{};             //!< [m^3 s^-2] gravitational parameter
    int numRevolutions = 0;  //!< [-] number of revolutions to be completed (completed orbits)

   private:
    void readMessages();
    void writeMessages(uint64_t currentSimNanos);
    std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> propagate(
        const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& EOM,
        std::array<double, 2> interval,
        const Eigen::VectorXd& X0,
        double dt);
    Eigen::VectorXd RK4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
                        const Eigen::VectorXd& X0,
                        double t0,
                        double dt);

    SolverMethod solverMethod;  //!< lambert solver algorithm (GOODING or IZZO)
    double time{};              //!< [s] Current vehicle time-tag associated with measurements
    Eigen::Vector3d r_N;        //!< [m] Current inertial spacecraft position vector in inertial frame N components
    Eigen::Vector3d v_N;        //!< [m/s] Current inertial velocity of the spacecraft in inertial frame N components
    //!< [m] Expected inertial spacecraft position vector at maneuver time tm in inertial frame N components
    Eigen::Vector3d rm_N;
};

#endif
