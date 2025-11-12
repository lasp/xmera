#include "mtbFeedforward.h"
#include <string.h>
#include <architecture/utilities/linearAlgebra.h>

/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.
 @return void
 @param callTime [ns] time the method is called
*/
void MtbFeedforward::reset(uint64_t callTime) {
    /*
     * Check if the required input messages are connected.
     */
    if (!this->dipoleRequestMtbInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbFeedForward.dipoleRequestMtbInMsg is not connected.");
    }
    if (!this->vehControlInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbFeedForward.vehControlInMsg is not connected.");
    }
    if (!this->tamSensorBodyInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbFeedForward.tamSensorBodyInMsg is not connected.");
    }
    if (!this->mtbArrayConfigParamsInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: mtbFeedForward.mtbArrayConfigParamsInMsg is not connected.");
    }

    /*! - Read in the torque rod input configuration message. This gives us the transformation from the
         torque rod space the the Body frame.*/
    this->mtbArrayConfigParams = this->mtbArrayConfigParamsInMsg();
}

/*! Computes the feedforward torque rod torque.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
*/
void MtbFeedforward::updateState(uint64_t callTime) {
    /*
     * Initialize local variables.
     */
    double mtbDipoleCmd_B[3] = {0.0, 0.0, 0.0};  // the commanded dipole in the Body frame
    double tauMtbFF_B[3] = {0.0, 0.0, 0.0};      // the torque rod feedforward term in the Body frame

    /*
     * Read the input messages and initialize output message.
     */
    MTBCmdMsgPayload dipoleRequestMtbInMsgBuffer = this->dipoleRequestMtbInMsg();
    TAMSensorBodyMsgPayload tamSensorBodyInMsgBuffer = this->tamSensorBodyInMsg();
    CmdTorqueBodyMsgPayload vehControlOutMsgBuffer = this->vehControlInMsg();

    /*! -  Compute net torque produced on the vehicle from the torque bars.*/
    mMultV(this->mtbArrayConfigParams.GtMatrix_B,
           3,
           this->mtbArrayConfigParams.numMTB,
           dipoleRequestMtbInMsgBuffer.mtbDipoleCmds,
           mtbDipoleCmd_B);
    v3Cross(mtbDipoleCmd_B, tamSensorBodyInMsgBuffer.tam_B, tauMtbFF_B);

    /*! -  Negate the net rod torque to spin wheels in appropriate direction. */
    v3Subtract(vehControlOutMsgBuffer.torqueRequestBody, tauMtbFF_B, vehControlOutMsgBuffer.torqueRequestBody);

    /*! - Write output message. This used as a feedforward term to the attiude controller.*/
    this->vehControlOutMsg.write(&vehControlOutMsgBuffer, moduleID, callTime);
}
