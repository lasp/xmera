/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "fswAlgorithms/attGuidance/mrpRotation/mrpRotation.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include <stdexcept>


/*! @brief This resets the module to original states.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpRotation::reset(uint64_t callTime)
{
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("mrpRotation.attRefInMsg wasn't connected.");
    }

    this->priorTime = 0;
    this->priorCmdSet = Eigen::Vector3d::Zero();
    this->priorCmdRates = Eigen::Vector3d::Zero();
}

/*! @brief This method takes the input attitude reference frame, and and superimposes the dynamics MRP
 scanning motion on top of this.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void MrpRotation::updateState(uint64_t callTime)
{
    /* - Read input messages */
    AttRefMsgPayload inputRef;
    AttRefMsgPayload attRefOut = {};

    /*!- read in input reference frame message */
    inputRef = this->attRefInMsg();

    /*! - Check if a desired attitude configuration message exists. This allows for dynamic changes to the desired MRP rotation */
    if (this->desiredAttInMsg.isLinked())
    {
        AttStateMsgPayload attStates;

        /* - Read Raster Manager messages */
        attStates = this->desiredAttInMsg();

        /* - Save commanded MRP set and body rates */
        this->cmdSet = Eigen::Map<const Eigen::Vector3d>(attStates.state);
        this->cmdRates = Eigen::Map<const Eigen::Vector3d>(attStates.rate);
        /* - Check the command is new */
        this->checkRasterCommands();
    }

    /*! - Compute time step to use in the integration downstream */
    this->computeTimeStep(callTime);

    Eigen::Vector3d sigma_RN = Eigen::Map<const Eigen::Vector3d>(inputRef.sigma_RN);
    Eigen::Vector3d omega_RN_N = Eigen::Map<const Eigen::Vector3d>(inputRef.omega_RN_N);
    Eigen::Vector3d domega_RN_N = Eigen::Map<const Eigen::Vector3d>(inputRef.domega_RN_N);

    /*! - Compute output reference frame */
    this->computeMRPRotationReference(sigma_RN,
                                      omega_RN_N,
                                      domega_RN_N,
                                      &attRefOut);


    /*! - write attitude guidance reference output */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);

    /*! - Update last time the module was called to current call time */
    this->priorTime = callTime;
    return;
}

/*! @brief This function checks if there is a new commanded raster maneuver message available
 @return void
 */
void MrpRotation::checkRasterCommands()
{
    bool prevCmdActive = ((this->cmdSet - this->priorCmdSet).array().abs() < 1E-12).all()
                         && ((this->cmdRates - this->priorCmdRates).array().abs() < 1E-12).all();

    /*! - check if a new attitude reference command message content is availble */
    if (!prevCmdActive)
    {
        /*! - copy over the commanded initial MRP and rate information */
        this->sigma_RR0 = this->cmdSet;
        this->omega_RR0_R = this->cmdRates;

        /*! - reset the prior commanded attitude state variables */
        this->priorCmdSet = this->cmdSet;
        this->priorCmdRates = this->cmdRates;
    }
}

/*! @brief This function computes control update time
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MrpRotation::computeTimeStep(uint64_t callTime)
{
    if (this->priorTime == 0)
    {
        this->dt = 0.0;
    } else {
        this->dt = (callTime - this->priorTime)*NANO2SEC;
    }
}


/*! @brief This function computes the reference (MRP attitude Set, angular velocity and angular acceleration)
 associated with a rotation defined in terms of an initial MRP set and a constant angular velocity vector
 @return void
 @param sigma_R0N The input reference attitude using MRPs
 @param omega_R0N_N The input reference frame angular rate vector
 @param domega_R0N_N The input reference frame angular acceleration vector
 @param attRefOut The output message copy
 */
void MrpRotation::computeMRPRotationReference(Eigen::Vector3d sigma_R0N,
                                              Eigen::Vector3d omega_R0N_N,
                                              Eigen::Vector3d domega_R0N_N,
                                              AttRefMsgPayload   *attRefOut)
{
    /*! - Compute attitude reference frame R/N information */
    Eigen::Matrix3d B = bmatMrp(this->sigma_RR0);
    Eigen::Vector3d sigmaDot_RR0 = 0.25 * B * this->omega_RR0_R;
    Eigen::Vector3d mrpSetNew = this->sigma_RR0 + sigmaDot_RR0 * this->dt;
    this->sigma_RR0 = mrpSwitch(mrpSetNew, 1.0);
    Eigen::Matrix3d dcm_RR0 = mrpToDcm(this->sigma_RR0);
    Eigen::Matrix3d dcm_R0N = mrpToDcm(sigma_R0N);
    Eigen::Matrix3d dcm_RN = dcm_RR0 * dcm_R0N;

    Eigen::Vector3d sigma_RN = dcmToMrp(dcm_RN);

    Eigen::Vector3d omega_RR0_N = dcm_RN.transpose() * this->omega_RR0_R;
    Eigen::Vector3d omega_RN_N = omega_RR0_N + omega_R0N_N;

    Eigen::Vector3d domega_RR0_N = omega_R0N_N.cross(omega_RR0_N);
    Eigen::Vector3d domega_RN_N = domega_RR0_N + domega_R0N_N;

    eigenVector3d2CArray(sigma_RN, attRefOut->sigma_RN);
    eigenVector3d2CArray(omega_RN_N, attRefOut->omega_RN_N);
    eigenVector3d2CArray(domega_RN_N, attRefOut->domega_RN_N);
}

/*! Setter method for the current MRP attitude coordinate set with respect to the input reference
 @return void
 @param sigma [-] current MRP attitude coordinate set with respect to the input reference
*/
void MrpRotation::setSigmaRR0(const Eigen::Vector3d &sigma) { this->sigma_RR0 = sigma; }

/*! Getter method for the current MRP attitude coordinate set with respect to the input reference
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d& MrpRotation::getSigmaRR0() const {return this->sigma_RR0; }

/*! Setter method for the angular velocity vector relative to input reference
 @return void
 @param omega [rad/s] angular velocity vector relative to input reference
*/
void MrpRotation::setOmegaRR0(const Eigen::Vector3d &omega) { this->omega_RR0_R = omega; }

/*! Getter method for the angular velocity vector relative to input reference
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d& MrpRotation::getOmegaRR0() const { return this->omega_RR0_R; }
