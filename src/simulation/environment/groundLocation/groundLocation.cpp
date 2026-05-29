// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "groundLocation.h"

#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/geodeticConversion.h>
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/safeMath.h>

/*! @brief Creates an instance of the GroundLocation class with a minimum elevation of 10 degrees,
 @return void
 */
GroundLocation::GroundLocation() {
    //! - Set some default initial conditions:
    this->minimumElevation =
        10. *
        D2R;  // [rad] minimum elevation above the local horizon needed to see a spacecraft; defaults to 10 degrees
    this->maximumRange = -1;  // [m] Maximum range for the groundLocation to compute access.

    this->currentGroundStateBuffer = GroundStateMsgPayload{};

    this->planetRadius = REQ_EARTH * 1e3;

    this->r_LP_P.fill(0.0);
    this->r_LP_P_Init.fill(0.0);

    this->planetState = SpicePlanetStateMsgPayload{};
    this->planetState.J20002Pfix[0][0] = 1;
    this->planetState.J20002Pfix[1][1] = 1;
    this->planetState.J20002Pfix[2][2] = 1;

    this->r_North_N << 0, 0, 1;
}

/*! Empty destructor method.
 @return void
 */
GroundLocation::~GroundLocation() {
    for (long unsigned int c = 0; c < this->accessOutMsgs.size(); c++) {
        delete this->accessOutMsgs.at(c);
    }
    return;
}

/*! Resets the internal position to the specified initial position.*/
void GroundLocation::reset(uint64_t currentSimNanos) {
    this->r_LP_P = this->r_LP_P_Init;

    if (this->planetRadius < 0) {
        bskLogger.bskLog(BSK_ERROR, "GroundLocation module must have planetRadius set.");
    }
}

/*! Specifies the ground location from planet-centered latitude, longitude, altitude position.
 *
 * @param lat
 * @param longitude
 * @param alt
 * @return
 */
void GroundLocation::specifyLocation(double lat, double longitude, double alt) {
    Eigen::Vector3d tmpLLAPosition(lat, longitude, alt);
    this->r_LP_P_Init = LLA2PCPF(tmpLLAPosition, this->planetRadius);
    this->dcm_LP = C_PCPF2SEZ(lat, longitude);
}

/*! Specifies the ground location from planet-centered, planet-fixed coordinates
 * @param r_LP_P_Loc
 */
void GroundLocation::specifyLocationPCPF(Eigen::Vector3d& r_LP_P_Loc) {
    /* Assign to r_LP_P_Init */
    this->r_LP_P_Init = r_LP_P_Loc;

    /* Convert to LLA */
    Eigen::Vector3d tmpLLAPosition = PCPF2LLA(this->r_LP_P_Init, this->planetRadius);

    /* Compute dcm_LP */
    this->dcm_LP = C_PCPF2SEZ(tmpLLAPosition[0], tmpLLAPosition[1]);
}

/*! Adds a scState message name to the vector of names to be subscribed to. Also creates a corresponding access message
 * output name.
 */
void GroundLocation::addSpacecraftToModel(Message<SCStatesMsgPayload>* tmpScMsg) {
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    /* create output message */
    Message<AccessMsgPayload>* msg;
    msg = new Message<AccessMsgPayload>;
    this->accessOutMsgs.push_back(msg);

    /* expand the buffer vector */
    AccessMsgPayload accMsg;
    this->accessMsgBuffer.push_back(accMsg);
}

/*! Read module messages
 */
bool GroundLocation::ReadMessages() {
    SCStatesMsgPayload scMsg;

    /* clear out the vector of spacecraft states.  This is created freshly below. */
    this->scStatesBuffer.clear();

    //! - read in the spacecraft state messages
    bool scRead;
    if (!this->scStateInMsgs.empty()) {
        scRead = true;
        for (long unsigned int c = 0; c < this->scStateInMsgs.size(); c++) {
            scMsg = this->scStateInMsgs.at(c)();
            scRead = scRead && this->scStateInMsgs.at(c).isWritten();
            this->scStatesBuffer.push_back(scMsg);
        }
    } else {
        bskLogger.bskLog(BSK_ERROR, "Ground location has no spacecraft to track.");
        scRead = false;
    }
    //! - Read in the optional planet message.  if no planet message is set, then a zero planet position, velocity and
    //! orientation is assumed
    bool planetRead = true;
    if (this->planetInMsg.isLinked()) {
        planetRead = this->planetInMsg.isWritten();
        this->planetState = this->planetInMsg();
    }

    return (planetRead && scRead);
}

/*! write module messages
 */
void GroundLocation::WriteMessages(uint64_t CurrentClock) {
    //! - write access message for each spacecraft
    for (long unsigned int c = 0; c < this->accessMsgBuffer.size(); c++) {
        this->accessOutMsgs.at(c)->write(this->accessMsgBuffer.at(c), this->moduleID, CurrentClock);
    }
    this->currentGroundStateOutMsg.write(this->currentGroundStateBuffer, this->moduleID, CurrentClock);
}

void GroundLocation::updateInertialPositions() {
    // First, get the rotation matrix from the inertial to planet frame from SPICE:
    this->dcm_PN = cArrayToEigenMatrix3(*this->planetState.J20002Pfix);
    this->dcm_PN_dot = cArrayToEigenMatrix3(*this->planetState.J20002Pfix_dot);
    this->r_PN_N = cArrayToEigenVector(this->planetState.PositionVector);
    // Then, transpose it to get the planet to inertial frame
    this->r_LP_N = this->dcm_PN.transpose() * this->r_LP_P_Init;
    this->rhat_LP_N = this->r_LP_N / this->r_LP_N.norm();
    this->r_LN_N = this->r_PN_N + this->r_LP_N;
    // Get planet frame angular velocity vector
    Eigen::Matrix3d w_tilde_PN = -this->dcm_PN_dot * this->dcm_PN.transpose();
    this->w_PN << w_tilde_PN(2, 1), w_tilde_PN(0, 2), w_tilde_PN(1, 0);
    //  Stash updated position in the groundState message
    eigenVectorToCArray(this->r_LN_N, this->currentGroundStateBuffer.r_LN_N);
    eigenVectorToCArray(this->r_LP_N, this->currentGroundStateBuffer.r_LP_N);
}

void GroundLocation::computeAccess() {
    // Update the groundLocation's inertial position
    this->updateInertialPositions();

    // Iterate over spacecraft position messages and compute the access for each one
    std::vector<AccessMsgPayload>::iterator accessMsgIt;
    std::vector<SCStatesMsgPayload>::iterator scStatesMsgIt;
    for (scStatesMsgIt = this->scStatesBuffer.begin(), accessMsgIt = accessMsgBuffer.begin();
         scStatesMsgIt != scStatesBuffer.end();
         scStatesMsgIt++, accessMsgIt++) {
        //! Compute the relative position of each spacecraft to the site in the planet-centered inertial frame
        Eigen::Vector3d r_BP_N = cArrayToEigenVector(scStatesMsgIt->r_BN_N) - this->r_PN_N;
        Eigen::Vector3d r_BL_N = r_BP_N - this->r_LP_N;
        auto r_BL_mag = r_BL_N.norm();
        Eigen::Vector3d relativeHeading_N = r_BL_N / r_BL_mag;

        double viewAngle = (M_PI_2 - safeAcos(this->rhat_LP_N.dot(relativeHeading_N)));

        accessMsgIt->slantRange = r_BL_mag;
        accessMsgIt->elevation = viewAngle;
        Eigen::Vector3d r_BL_L = this->dcm_LP * this->dcm_PN * r_BL_N;
        eigenVectorToCArray(r_BL_L, accessMsgIt->r_BL_L);
        double cos_az = -r_BL_L[0] / (sqrt(pow(r_BL_L[0], 2) + pow(r_BL_L[1], 2)));
        double sin_az = r_BL_L[1] / (sqrt(pow(r_BL_L[0], 2) + pow(r_BL_L[1], 2)));
        accessMsgIt->azimuth = atan2(sin_az, cos_az);

        Eigen::Vector3d v_BL_L =
            this->dcm_LP * this->dcm_PN *
            (cArrayToEigenVector(scStatesMsgIt->v_BN_N) -
             this->w_PN.cross(r_BP_N));  // V observed from gL wrt P frame, expressed in L frame coords (SEZ)
        eigenVectorToCArray(v_BL_L, accessMsgIt->v_BL_L);
        accessMsgIt->range_dot = v_BL_L.dot(r_BL_L) / r_BL_mag;
        double xy_norm = sqrt(pow(r_BL_L[0], 2) + pow(r_BL_L[1], 2));
        accessMsgIt->az_dot = (-r_BL_L[0] * v_BL_L[1] + r_BL_L[1] * v_BL_L[0]) / pow(xy_norm, 2);
        accessMsgIt->el_dot =
            (v_BL_L[2] / xy_norm - r_BL_L[2] * (r_BL_L[0] * v_BL_L[0] + r_BL_L[1] * v_BL_L[1]) / pow(xy_norm, 3)) /
            (1 + pow(r_BL_L[2] / xy_norm, 2));

        if ((viewAngle > this->minimumElevation) && (r_BL_mag <= this->maximumRange || this->maximumRange < 0)) {
            accessMsgIt->hasAccess = 1;
        } else {
            accessMsgIt->hasAccess = 0;
        }
    }
}

/*!
 update module
 @param currentSimNanos
 */
void GroundLocation::updateState(uint64_t currentSimNanos) {
    this->ReadMessages();
    this->computeAccess();
    this->WriteMessages(currentSimNanos);
}
