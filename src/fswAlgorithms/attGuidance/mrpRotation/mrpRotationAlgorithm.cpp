#include "mrpRotationAlgorithm.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

/*! @brief This resets the module to original states.
 @return void
 */
void MrpRotationAlgorithm::reset() {
    this->priorTime = 0;
    this->priorCmdSet = Eigen::Vector3d::Zero();
    this->priorCmdRates = Eigen::Vector3d::Zero();
}

/*! @brief This method takes the input attitude reference frame, and superimposes the dynamics MRP
 scanning motion on top of this.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param inputRef Guidance reference input message
 @param attStates Incoming message containing the desired attitude set
 */
AttRefMsgPayload MrpRotationAlgorithm::update(uint64_t callTime,
                                              AttRefMsgPayload inputRef,
                                              AttStateMsgPayload attStates) {
    /*! - Check if a desired attitude configuration message exists. This allows for dynamic changes to the desired MRP
     * rotation */
    if (this->dynamicReferenceEnabled) {
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
    AttRefMsgPayload attRefOut = this->computeMRPRotationReference(sigma_RN, omega_RN_N, domega_RN_N);

    /*! - Update last time the module was called to current call time */
    this->priorTime = callTime;

    return attRefOut;
}

/*! @brief This function checks if there is a new commanded raster maneuver message available
 @return void
 */
void MrpRotationAlgorithm::checkRasterCommands() {
    bool prevCmdActive = ((this->cmdSet - this->priorCmdSet).array().abs() < 1E-12).all() &&
                         ((this->cmdRates - this->priorCmdRates).array().abs() < 1E-12).all();

    /*! - check if a new attitude reference command message content is availble */
    if (!prevCmdActive) {
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
void MrpRotationAlgorithm::computeTimeStep(uint64_t callTime) {
    if (this->priorTime == 0) {
        this->dt = 0.0;
    } else {
        this->dt = (callTime - this->priorTime) * NANO2SEC;
    }
}

/*! @brief This function computes the reference (MRP attitude Set, angular velocity and angular acceleration)
 associated with a rotation defined in terms of an initial MRP set and a constant angular velocity vector
 @return AttRefMsgPayload The output message copy
 @param sigma_R0N The input reference attitude using MRPs
 @param omega_R0N_N The input reference frame angular rate vector
 @param domega_R0N_N The input reference frame angular acceleration vector
 */
AttRefMsgPayload MrpRotationAlgorithm::computeMRPRotationReference(Eigen::Vector3d sigma_R0N,
                                                                   Eigen::Vector3d omega_R0N_N,
                                                                   Eigen::Vector3d domega_R0N_N) {
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

    AttRefMsgPayload attRefOut{};

    eigenVectorToCArray(sigma_RN, attRefOut.sigma_RN);
    eigenVectorToCArray(omega_RN_N, attRefOut.omega_RN_N);
    eigenVectorToCArray(domega_RN_N, attRefOut.domega_RN_N);

    return attRefOut;
}

/*! Setter method for the current MRP attitude coordinate set with respect to the input reference
 @return void
 @param sigma [-] current MRP attitude coordinate set with respect to the input reference
*/
void MrpRotationAlgorithm::setSigmaRR0(const Eigen::Vector3d& sigma) { this->sigma_RR0 = sigma; }

/*! Getter method for the current MRP attitude coordinate set with respect to the input reference
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d MrpRotationAlgorithm::getSigmaRR0() const { return this->sigma_RR0; }

/*! Setter method for the angular velocity vector relative to input reference
 @return void
 @param omega [rad/s] angular velocity vector relative to input reference
*/
void MrpRotationAlgorithm::setOmegaRR0(const Eigen::Vector3d& omega) { this->omega_RR0_R = omega; }

/*! Getter method for the angular velocity vector relative to input reference
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d MrpRotationAlgorithm::getOmegaRR0() const { return this->omega_RR0_R; }

/*! Enable dynamic reference input
 @return void
*/
void MrpRotationAlgorithm::enableDynamicReference() { this->dynamicReferenceEnabled = true; }

/*! Get whether dynamic reference input is enabled
 @return const bool
*/
const bool MrpRotationAlgorithm::isDynamicReferenceEnabled() const { return this->dynamicReferenceEnabled; }
