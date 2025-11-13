#include "MtbEffector.h"
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
MtbEffector::MtbEffector() {}

/*! Module Destructor */
MtbEffector::~MtbEffector() {}

/*! This method is used to reset the module and checks that required input messages are connect.
    @return void
*/
void MtbEffector::reset(uint64_t currentSimNanos) {
    /*
     * Check that required input messages are connected.
     */
    if (!this->mtbCmdInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "MtbEffector.mtbCmdInMsg was not linked.");
    }
    if (!this->magInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "MtbEffector.magInMsg was not linked.");
    }
    if (!this->mtbParamsInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "MtbEffector.mtbParamsInMsg was not linked.");
    }

    /*
     * Zero the effector output forces and torques.
     */
    this->forceExternal_B.fill(0.0);
    this->torqueExternalPntB_B.fill(0.0);
    this->forceExternal_N.fill(0.0);

    return;
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
*/
void MtbEffector::updateState(uint64_t currentSimNanos) {
    /*
     * Write to the output message.
     */
    this->WriteOutputMessages(currentSimNanos);

    return;
}

/*! This method is used to link the magnetic torque bar effector to the hub attitude.
 @return void
 */
void MtbEffector::linkInStates(DynParamManager& states) {
    /*
     * Link the Body relative to Inertial frame modified modriguez parameter.
     */
    this->hubSigma = states.getStateObject("hubSigma");

    return;
}

/*! This method computes the body torque contribution from all magnetic torque bars.
 @return void
*/
void MtbEffector::computeForceTorque(double integTime, double timeStep) {
    /*
     * Create local variables.
     */
    Eigen::MRPd sigmaBN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Vector3d magField_B;
    Eigen::Matrix3d bTilde;
    Eigen::MatrixXd GtMatrix_B;
    Eigen::VectorXd muCmd_T;
    Eigen::Vector3d mtbTorque_B;
    Eigen::Vector3d magField_N;

    /*
     * Assign input messages to private class attributes.
     */
    this->mtbCmdInMsgBuffer = this->mtbCmdInMsg();
    this->magInMsgBuffer = this->magInMsg();
    this->mtbConfigParams = this->mtbParamsInMsg();

    /*
     * Zero out the external torque in the body frame.
     */
    this->torqueExternalPntB_B.setZero();

    /*
     * Construct bTilde matrix.
     */
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    dcm_BN = sigmaBN.toRotationMatrix().transpose();
    magField_N = cArrayAsEigenVector(this->magInMsgBuffer.magField_N);
    magField_B = dcm_BN * magField_N;
    bTilde = eigenTilde(magField_B);

    /*
     * Compute torque produced by magnetic torque bars in body frame components.
     * Since cArrayAsEigenMatrix expects a column major input, we need to
     * transpose GtMatrix_B.
     */
    double GtColMajor[3 * MAX_EFF_CNT];
    mSetZero(GtColMajor, 3, this->mtbConfigParams.numMTB);
    mTranspose(this->mtbConfigParams.GtMatrix_B, 3, this->mtbConfigParams.numMTB, GtColMajor);
    GtMatrix_B = cArrayAsEigenMatrixX(GtColMajor, 3, this->mtbConfigParams.numMTB);

    /* check if dipole commands are saturating the effector */
    for (int i = 0; i < this->mtbConfigParams.numMTB; i++) {
        if (this->mtbCmdInMsgBuffer.mtbDipoleCmds[i] > this->mtbConfigParams.maxMtbDipoles[i]) {
            this->mtbCmdInMsgBuffer.mtbDipoleCmds[i] = this->mtbConfigParams.maxMtbDipoles[i];
        } else if (this->mtbCmdInMsgBuffer.mtbDipoleCmds[i] < -this->mtbConfigParams.maxMtbDipoles[i]) {
            this->mtbCmdInMsgBuffer.mtbDipoleCmds[i] = -this->mtbConfigParams.maxMtbDipoles[i];
        }
    }

    muCmd_T = Eigen::Map<Eigen::VectorXd>(this->mtbCmdInMsgBuffer.mtbDipoleCmds, this->mtbConfigParams.numMTB, 1);
    mtbTorque_B = -bTilde * GtMatrix_B * muCmd_T;
    this->torqueExternalPntB_B = mtbTorque_B;

    return;
}

/*! Write the magnetic torque bar output message.
@return void
 */
void MtbEffector::WriteOutputMessages(uint64_t CurrentClock) {
    /*
     * Initialize output message buffer.
     */
    MTBMsgPayload mtbOutMsgBuffer;
    mtbOutMsgBuffer = MTBMsgPayload{};

    /*
     * Write output message
     */
    eigenVectorToCArray(this->torqueExternalPntB_B, mtbOutMsgBuffer.mtbNetTorque_B);
    this->mtbOutMsg.write(&mtbOutMsgBuffer, this->moduleID, CurrentClock);

    return;
}
