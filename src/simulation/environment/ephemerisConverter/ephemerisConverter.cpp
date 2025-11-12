#include "ephemerisConverter.h"
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

EphemerisConverter::EphemerisConverter() {}

EphemerisConverter::~EphemerisConverter() {
    for (long unsigned int c = 0; c < this->ephemOutMsgs.size(); c++) {
        free(this->ephemOutMsgs.at(c));
    }
}

/*! Reset the module to origina configuration values.
 @return void
 */
void EphemerisConverter::reset(uint64_t CurrenSimNanos) {
    // check if the spiceInMsgs is empty or not
    if (this->spiceInMsgs.size() == 0) {
        bskLogger.bskLog(BSK_ERROR, "ephemerisConverter.spiceInMsgs is empty.");
    }
}

/*!
 add a planet spice input message
 */
void EphemerisConverter::addSpiceInputMsg(Message<SpicePlanetStateMsgPayload>* tmpMsg) {
    this->spiceInMsgs.push_back(tmpMsg->addSubscriber());

    /* setup output corresponding message */
    Message<EphemerisMsgPayload>* msg;
    msg = new Message<EphemerisMsgPayload>;
    this->ephemOutMsgs.push_back(msg);

    /* update input and output buffers*/
    SpicePlanetStateMsgPayload tmpSpice = {};
    this->spiceInBuffers.push_back(tmpSpice);

    EphemerisMsgPayload tmpEphem = {};
    this->ephemOutBuffers.push_back(tmpEphem);
}

/*!
    convert ephemeris data
    @param clockNow
 */
void EphemerisConverter::convertEphemData(uint64_t clockNow) {
    Eigen::Matrix3d dcm_BN;
    Eigen::Vector3d sigma_BN;
    Eigen::Matrix3d dcm_BN_dot;
    Eigen::Matrix3d omega_tilde_BN_B_eigen;
    double omega_tilde_BN_B[3][3];
    double omega_tilde_BN_B_array[9];

    for (long unsigned int c = 0; c < this->spiceInMsgs.size(); c++) {
        v3Copy(this->spiceInBuffers.at(c).PositionVector, this->ephemOutBuffers.at(c).r_BdyZero_N);

        v3Copy(this->spiceInBuffers.at(c).VelocityVector, this->ephemOutBuffers.at(c).v_BdyZero_N);

        this->ephemOutBuffers.at(c).timeTag = this->spiceInMsgs.at(c).timeWritten() * 1.0E-9;

        /* Compute sigma_BN */
        dcm_BN = cArrayAsEigenMatrix3(*this->spiceInBuffers.at(c).J20002Pfix);
        sigma_BN = dcmToMrp(dcm_BN);
        eigenVectorToCArray(sigma_BN, this->ephemOutBuffers.at(c).sigma_BN);  // sigma_BN

        /* Compute omega_BN_B */
        dcm_BN_dot = cArrayAsEigenMatrix3(*this->spiceInBuffers.at(c).J20002Pfix_dot);
        omega_tilde_BN_B_eigen = -dcm_BN_dot * dcm_BN.transpose();
        eigenMatrixToCArray(omega_tilde_BN_B_eigen, omega_tilde_BN_B_array);
        m33Copy(RECAST3X3 omega_tilde_BN_B_array, omega_tilde_BN_B);
        this->ephemOutBuffers.at(c).omega_BN_B[0] = omega_tilde_BN_B[2][1];
        this->ephemOutBuffers.at(c).omega_BN_B[1] = omega_tilde_BN_B[0][2];
        this->ephemOutBuffers.at(c).omega_BN_B[2] = omega_tilde_BN_B[1][0];
    }
}

void EphemerisConverter::readInputMessages() {
    for (long unsigned int c = 0; c < this->spiceInMsgs.size(); c++) {
        this->spiceInBuffers.at(c) = this->spiceInMsgs.at(c)();
    }
}

/*!
    write output message
    @param currentSimNanos time in nano-seconds
 */
void EphemerisConverter::writeOutputMessages(uint64_t currentSimNanos) {
    for (long unsigned int c = 0; c < this->ephemOutMsgs.size(); c++) {
        this->ephemOutMsgs.at(c)->write(&this->ephemOutBuffers.at(c), this->moduleID, currentSimNanos);
    }
}

/*!
    update module states
    @param currentSimNanos time in nano-seconds
 */
void EphemerisConverter::updateState(uint64_t currentSimNanos) {
    readInputMessages();
    convertEphemData(currentSimNanos);
    writeOutputMessages(currentSimNanos);
}
