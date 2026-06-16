// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef PLANETNAV_H
#define PLANETNAV_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <architecture/utilities/gauss_markov.h>

#include <Eigen/Dense>

//! 12x12 process-noise / propagation matrix type for the Gauss-Markov error model.
//! Named so SWIG can match it by name (see swig_eigen.i EIGEN_MAT_WRAP) and expose it to Python.
typedef Eigen::Matrix<double, 12, 12> PlanetNavCovar;
//! 12-element error / walk-bounds vector type for the Gauss-Markov error model.
typedef Eigen::Matrix<double, 12, 1> PlanetNavErrorVector;

/*! @brief This is an auto-created sample C++ module.  The description is included with the module class definition
 */
class PlanetNav : public SysModel {
public:
    PlanetNav();
    ~PlanetNav();

    void reset(uint64_t currentSimNanos);          //!< -- Reset function
    void updateState(uint64_t currentSimNanos);    //!< -- updateState
    void computeErrors(uint64_t currentSimNanos);  //!< -- Compute the errors to add to the truth
    void applyErrors();                            //!< -- Add the errors to the truth
    void readInputMessages();                      //!> -- Read the input messages
    void writeOutputMessages(uint64_t Clock);      //!> -- Write the output messages

public:
    PlanetNavCovar
        PMatrix;  //!< -- Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    PlanetNavErrorVector walkBounds;  //!< -- "3-sigma" errors to permit for states
    PlanetNavErrorVector navErrors;   //!< -- Current navigation errors applied to truth
    bool crossTrans;                  //!< -- Have position error depend on velocity
    bool crossAtt;                    //!< -- Have attitude depend on attitude rate

    EphemerisMsgPayload truePlanetState;              //!< planet ephemeris msg without noise
    EphemerisMsgPayload noisePlanetState;             //!< planet ephemeris msg with noise
    ReadFunctor<EphemerisMsgPayload> ephemerisInMsg;  //!< planet ephemeris input msg
    Message<EphemerisMsgPayload> ephemerisOutMsg;     //!< planet ephemeris output msg

    BSKLogger bskLogger;  //!< -- BSK Logging

private:
    PlanetNavCovar AMatrix;  //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;  //!< -- Gauss-markov error states
    uint64_t prevTime;       //!< -- Previous simulation time observed
};

#endif
