#include "lambertSecondDV.h"
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertSecondDV::LambertSecondDV() = default;

/*! Module Destructor */
LambertSecondDV::~LambertSecondDV() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSecondDV::reset(uint64_t currentSimNanos) {
    // check that required input messages are connected
    if (!this->lambertSolutionInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSecondDV.lambertSolutionInMsg was not linked.");
    }
    if (!this->desiredVelocityInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSecondDV.desiredVelocityInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSecondDV::updateState(uint64_t currentSimNanos) {
    // read messages
    this->readMessages();

    // compute Delta-V
    this->dv_N = this->vDesired_N - this->vExpected_N;

    // write messages
    this->writeMessages(currentSimNanos);
}

/*! This method reads the input messages each call of updateState.
    It also checks if the message contents are valid for this module.
    @return void
*/
void LambertSecondDV::readMessages() {
    LambertSolutionMsgPayload lambertSolutionInMsgBuffer = this->lambertSolutionInMsg();
    DesiredVelocityMsgPayload desiredVelocityInMsgBuffer = this->desiredVelocityInMsg();

    // lambert solution content
    if (this->lambertSolutionSpecifier == 1) {
        this->vExpected_N = cArrayToEigenVector(lambertSolutionInMsgBuffer.v2);
        this->validLambert = lambertSolutionInMsgBuffer.valid;
    } else if (this->lambertSolutionSpecifier == 2) {
        this->vExpected_N = cArrayToEigenVector(lambertSolutionInMsgBuffer.v2Sol2);
        this->validLambert = lambertSolutionInMsgBuffer.validSol2;
    } else {
        bskLogger.bskLog(BSK_ERROR,
                         "lambertValidator: the parameter lambertSolutionSpecifier that specifies which "
                         "Lambert solution should be used must be either 1 or 2.");
    }

    // desired velocity
    this->vDesired_N = cArrayToEigenVector(desiredVelocityInMsgBuffer.vDesired_N);
    this->maneuverTime = desiredVelocityInMsgBuffer.maneuverTime;
}

/*! This method writes the output messages each call of updateState
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSecondDV::writeMessages(uint64_t currentSimNanos) {
    DvBurnCmdMsgPayload dvBurnCmdOutMsgBuffer{};

    // DV Rotation vector and rotation magnitude not used by this module. Set to arbitrary unit vector and zero.
    Eigen::Vector3d dvRotVecUnit;
    dvRotVecUnit << 1., 0., 0.;
    double dvRotVecMag = 0.;

    auto burnStartTime = static_cast<uint64_t>(this->maneuverTime * SEC2NANO);

    // Write Delta-V message content only if all checks on performance and violations were passed
    if (this->validLambert) {
        eigenVectorToCArray(this->dv_N, dvBurnCmdOutMsgBuffer.dvInrtlCmd);
        eigenVectorToCArray(dvRotVecUnit, dvBurnCmdOutMsgBuffer.dvRotVecUnit);
        dvBurnCmdOutMsgBuffer.dvRotVecMag = dvRotVecMag;
        dvBurnCmdOutMsgBuffer.burnStartTime = burnStartTime;
    }

    // Write to the output messages
    this->dvBurnCmdOutMsg.write(&dvBurnCmdOutMsgBuffer, this->moduleID, currentSimNanos);
}
