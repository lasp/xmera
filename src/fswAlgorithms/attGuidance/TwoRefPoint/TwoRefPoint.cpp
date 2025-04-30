/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include <math.h>
#include "fswAlgorithms/attGuidance/twoRefPoint/twoRefPoint.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"



void TwoRefPoint::reset(uint64_t callTime)
{
    this->secCelBodyIsLinked = this->secCelBodyInMsg.isLinked();

    // check if required input messages have been included
    if (!this->transNavInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: TwoRefPoint.transNavInMsg wasn't connected.");
    }
    if (!this->celBodyInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: TwoRefPoint.celBodyInMsg wasn't connected.");
    }

    return;
}

/*! This method takes the spacecraft and points a specified axis at a named
 celestial body specified in the configuration data.  It generates the
 commanded attitude and assumes that the control errors are computed
 downstream.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TwoRefPoint::updateState(uint64_t callTime)
{
    /*! - Parse the input messages */
    this->parseInputMessages();
    /*! - Compute the pointing requirements */
    this->computeTwoRefPoint(callTime);
    /*! - Write the output message */
    this->attRefOutMsg.write(&this->attRefOut, this->moduleID, callTime);
}

/*! This method takes the navigation translational info as well as the spice data of the
 primary celestial body and, if applicable, the second one, and computes the relative state vectors
 necessary to create the restricted 2-body pointing reference frame.
 @return void
 */
void TwoRefPoint::parseInputMessages()
{
    NavTransMsgPayload navData;
    EphemerisMsgPayload primPlanet;
    EphemerisMsgPayload secPlanet;

    double R_P1B_N_hat[3];          /* Unit vector in the direction of r_P1 */
    double R_P2B_N_hat[3];          /* Unit vector in the direction of r_P2 */

    double platAngDiff{};             /* Angle between r_P1 and r_P2 */
    double dotProduct;              /* Temporary scalar variable */

    // read input messages
    navData = this->transNavInMsg();
    primPlanet = this->celBodyInMsg();


    v3Subtract(primPlanet.r_BdyZero_N, navData.r_BN_N, this->R_P1B_N);


    if(this->secCelBodyIsLinked)
    {

        secPlanet = this->secCelBodyInMsg();
        /*! - Compute R_P2 and v_P2 */
        v3Subtract(secPlanet.r_BdyZero_N, navData.r_BN_N, this->R_P2B_N);
        v3Normalize(this->R_P1B_N, R_P1B_N_hat);
        v3Normalize(this->R_P2B_N, R_P2B_N_hat);
        dotProduct = v3Dot(R_P2B_N_hat, R_P1B_N_hat);
        platAngDiff = safeAcos(dotProduct);
    }

    /*! - Cross the P1 states to get R_P2, v_p2 and a_P2 */
    if(!this->secCelBodyIsLinked ||
       fabs(platAngDiff) < this->singularityThresh ||
       fabs(platAngDiff) > M_PI - this->singularityThresh)
    {

    }
}



/*! This method takes the spacecraft and points a specified axis at a named
 celestial body specified in the configuration data.  It generates the
 commanded attitude and assumes that the control errors are computed
 downstream.
 @return void
 @param this The configuration data associated with the celestial body guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void TwoRefPoint::computeTwoRefPoint(uint64_t callTime)
{
    double temp3[3];        /* Temporary vector */
    double temp3_1[3];      /* Temporary vector 1 */
    double temp3_2[3];      /* Temporary vector 2 */
    double temp3_3[3];      /* Temporary vector 3 */
    double temp33[3][3];    /* Temporary 3x3 matrix */
    double temp33_1[3][3];  /* Temporary 3x3 matrix 1 */
    double temp33_2[3][3];  /* Temporary 3x3 matrix 2 */

    double R_N[3];          /* Normal vector of the plane defined by R_P1 and R_P2 */


    double dcm_RN[3][3];    /* DCM that maps from Reference frame to the inertial */
    double r1_hat[3];       /* 1st row vector of RN */
    double r2_hat[3];       /* 2nd row vector of RN */
    double r3_hat[3];       /* 3rd row vector of RN */


    double I_33[3][3];      /* Identity 3x3 matrix */


    this->attRefOut = {};

    /* - Initial computations: R_n, v_n, a_n */
    v3Cross(this->R_P1B_N, this->R_P2B_N, R_N);

    /* - Reference Frame computation */
    v3Normalize(this->R_P1B_N, r1_hat); /* Eq 9a*/
    v3Normalize(R_N, r3_hat); /* Eq 9c */
    v3Cross(r3_hat, r1_hat, r2_hat); /* Eq 9b */
    v3Normalize(r2_hat, r2_hat);
    v3Copy(r1_hat, dcm_RN[0]);
    v3Copy(r2_hat, dcm_RN[1]);
    v3Copy(r3_hat, dcm_RN[2]);
    C2MRP(dcm_RN, this->attRefOut.sigma_RN);




    return;
}
