// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "hillPointCpp.h"

#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>

/*! This method performs the module reset capability. */
void HillPointCpp::reset(uint64_t currentSimNanos) {
    // Check if the required input messages are linked
    if (!this->transNavInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR, "hillPoint.transNavInMsg wasn't connected.");
    }
    this->planetMsgIsLinked = this->celBodyInMsg.isLinked();
}

/*! This method creates a orbit hill frame reference message.  The desired orientation is
 defined within the module.
 */
void HillPointCpp::updateState(uint64_t currentSimNanos) {
    /*! - zero the output message */
    AttRefMsgPayload AttRefOutBuffer = AttRefMsgPayload();

    /* zero the local planet ephemeris message */
    EphemerisMsgPayload primPlanet = EphemerisMsgPayload(); /* zero'd as default, even if not connected */
    if (this->planetMsgIsLinked) { primPlanet = this->celBodyInMsg(); }
    NavTransMsgPayload navData = this->transNavInMsg();

    /*! - Compute and store output message */
    computeHillPointingReference(
        (Eigen::Vector3d) navData.r_BN_N,
        (Eigen::Vector3d) navData.v_BN_N,
        (Eigen::Vector3d) primPlanet.r_BdyZero_N,
        (Eigen::Vector3d) primPlanet.v_BdyZero_N,
        &AttRefOutBuffer
    );

    this->attRefOutMsg.write(AttRefOutBuffer, moduleID, currentSimNanos);
}

void HillPointCpp::computeHillPointingReference(
    Eigen::Vector3d r_BN_N,
    Eigen::Vector3d v_BN_N,
    Eigen::Vector3d celBdyPositonVector,
    Eigen::Vector3d celBdyVelocityVector,
    AttRefMsgPayload* attRefOut
) {
    /*! - Compute relative position and velocity of the spacecraft with respect to the main celestial body */
    Eigen::Vector3d relPosVector = r_BN_N - celBdyPositonVector;
    Eigen::Vector3d relVelVector = v_BN_N - celBdyVelocityVector;

    /*! - Compute RN */
    Eigen::Matrix3d dcm_RN; /* DCM from inertial to reference frame */
    /*! - First row of RN is i_r */
    dcm_RN.row(0) = relPosVector.normalized();

    /*! - Third row of RN is i_h */
    Eigen::Vector3d orbitAngMomentum = relPosVector.cross(relVelVector); /* orbit angular momentum vector */
    dcm_RN.row(2) = orbitAngMomentum.normalized();

    /*! - Second row of RN is i_theta */
    dcm_RN.row(1) = dcm_RN.row(2).cross(dcm_RN.row(0));

    /*! - Compute R-frame orientation */
    Eigen::Vector3d sigma_RN = dcmToMrp(dcm_RN);
    eigenVectorToCArray(sigma_RN, attRefOut->sigma_RN);

    /*! - Compute R-frame inertial rate and acceleration */
    double orbitRadius = relPosVector.norm(); /* orbit radius */

    /*! - determine orbit angular rates and accelerations */
    double dfdt;                                                              /* rotational rate of the orbit frame */
    double ddfdt2;                                                            /* rotational acceleration of the frame */
    if (orbitRadius > 1.0) {                                                  /* Robustness check */
        dfdt = orbitAngMomentum.norm() / (orbitRadius * orbitRadius);         /* true anomaly rate */
        ddfdt2 = -2.0 * relVelVector.dot(dcm_RN.row(0)) / orbitRadius * dfdt; /* derivative of true anomaly rate */
    } else {
        /* an error has occurred, radius shouldn't be less than 1km  */
        dfdt = 0.0;
        ddfdt2 = 0.0;
    }

    Eigen::Vector3d omega_RN_R = {
        0.0,
        0.0,
        dfdt
    }; /* reference angular velocity vector in Reference frame R components */
    Eigen::Vector3d domega_RN_R = {
        0.0,
        0.0,
        ddfdt2
    }; /* reference angular acceleration vector in Reference frame R components */

    Eigen::Vector3d temp = dcm_RN.transpose() * omega_RN_R;
    eigenVectorToCArray(temp, attRefOut->omega_RN_N);
    temp = dcm_RN.transpose() * domega_RN_R;
    eigenVectorToCArray(temp, attRefOut->domega_RN_N);
}
