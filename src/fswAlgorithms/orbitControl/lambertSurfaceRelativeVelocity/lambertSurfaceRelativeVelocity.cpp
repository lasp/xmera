#include "lambertSurfaceRelativeVelocity.h"
#include <architecture/utilities/linearAlgebra.h>
#include <cmath>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertSurfaceRelativeVelocity::LambertSurfaceRelativeVelocity() = default;

/*! Module Destructor */
LambertSurfaceRelativeVelocity::~LambertSurfaceRelativeVelocity() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSurfaceRelativeVelocity::reset(uint64_t currentSimNanos) {
    // check that required input messages are connected
    if (!this->lambertProblemInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSurfaceRelativeVelocity.lambertProblemInMsg was not linked.");
    }
    if (!this->ephemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSurfaceRelativeVelocity.ephemerisInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    It computes the solution of Lambert's problem.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSurfaceRelativeVelocity::updateState(uint64_t currentSimNanos) {
    // read messages
    this->readMessages();

    // surface frame: s1 in east direction, s2 in north direction, s3 in up direction
    Eigen::Vector3d s1Hat_N = (this->omega_PN_N.cross(this->r_BN_N)).normalized();
    Eigen::Vector3d s3Hat_N = this->r_BN_N.normalized();
    Eigen::Vector3d s2Hat_N = (s3Hat_N.cross(s1Hat_N)).normalized();
    // DCM from inertial frame N to surface frame S
    Eigen::Matrix3d dcm_SN;
    dcm_SN << s1Hat_N.transpose(), s2Hat_N.transpose(), s3Hat_N.transpose();

    this->v_BN_N = this->omega_PN_N.cross(this->r_BN_N) + dcm_SN.transpose() * this->vRelativeDesired_S;

    // write messages
    this->writeMessages(currentSimNanos);
}

/*! This method reads the input messages each call of updateState.
    @return void
*/
void LambertSurfaceRelativeVelocity::readMessages() {
    LambertProblemMsgPayload lambertProblemInMsgBuffer = this->lambertProblemInMsg();
    EphemerisMsgPayload ephemerisInMsgBuffer = this->ephemerisInMsg();

    this->r_BN_N = cArrayAsEigenVector(lambertProblemInMsgBuffer.r2vec);

    Eigen::MRPd sigma_PN = cArrayAsEigenMrp(ephemerisInMsgBuffer.sigma_BN);
    this->dcm_PN = sigma_PN.toRotationMatrix().transpose();
    Eigen::Vector3d omega_PN_P = cArrayAsEigenVector(ephemerisInMsgBuffer.omega_BN_B);
    this->omega_PN_N = this->dcm_PN.transpose() * omega_PN_P;
}

/*! This method writes the output messages each call of updateState
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSurfaceRelativeVelocity::writeMessages(uint64_t currentSimNanos) {
    DesiredVelocityMsgPayload desiredVelocityOutMsgBuffer{};

    eigenVectorToCArray(this->v_BN_N, desiredVelocityOutMsgBuffer.vDesired_N);
    desiredVelocityOutMsgBuffer.maneuverTime = this->time;

    // Write to the output messages
    this->desiredVelocityOutMsg.write(&desiredVelocityOutMsgBuffer, this->moduleID, currentSimNanos);
}
