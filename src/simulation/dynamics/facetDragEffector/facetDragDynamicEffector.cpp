/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "facetDragDynamicEffector.h"

#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/eigenMRP.h>
#include <architecture/utilities/eigenSupport.h>
#include <architecture/utilities/linearAlgebra.h>

FacetDragDynamicEffector::FacetDragDynamicEffector() {
    this->forceExternal_B.fill(0.0);
    this->torqueExternalPntB_B.fill(0.0);
    this->v_B.fill(0.0);
    this->v_hat_B.fill(0.0);
}

void FacetDragDynamicEffector::reset(uint64_t currentSimNanos) {
    // check if input message has not been included
    if (!this->atmoDensInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "facetDragDynamicEffector.atmoDensInMsg was not linked.");
    }
}

/*! This method is used to read the incoming density message and update the internal density/
atmospheric data.
 @return void
 */
bool FacetDragDynamicEffector::readInputs() {
    bool dataGood;
    this->atmoInData = this->atmoDensInMsg();
    dataGood = this->atmoDensInMsg.isWritten();
    return dataGood;
}

/*!
    add a facet
    @param area
    @param dragCoeff
    @param B_normal_hat
    @param B_location
 */
void FacetDragDynamicEffector::addFacet(double area,
                                        double dragCoeff,
                                        Eigen::Vector3d B_normal_hat,
                                        Eigen::Vector3d B_location) {
    this->scGeometry.facetAreas.push_back(area);
    this->scGeometry.facetCoeffs.push_back(dragCoeff);
    this->scGeometry.facetNormals_B.push_back(B_normal_hat);
    this->scGeometry.facetLocations_B.push_back(B_location);
    this->numFacets = this->numFacets + 1;
}

/*! This method is used to link the dragEffector to the hub attitude and velocity,
which are required for calculating drag forces and torques.
 @return void
 @param states dynamic parameter states
 */

void FacetDragDynamicEffector::linkInStates(DynParamManager& states) {
    this->hubSigma = states.getStateObject("hubSigma");
    this->hubVelocity = states.getStateObject("hubVelocity");
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
 */
void FacetDragDynamicEffector::updateDragDir() {
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();

    this->v_B = dcm_BN * this->hubVelocity->getState();  // [m/s] sc velocity
    this->v_hat_B = this->v_B / this->v_B.norm();
}

/*! This method WILL implement a more complex flat-plate aerodynamics model with attitude
dependence and lift forces.
*/
void FacetDragDynamicEffector::plateDrag() {
    Eigen::Vector3d facetDragForce;
    Eigen::Vector3d facetDragTorque;
    Eigen::Vector3d totalDragForce;
    Eigen::Vector3d totalDragTorque;

    //! - Zero out the structure force/torque for the drag set
    double projectedArea = 0.0;
    double projectionTerm = 0.0;
    totalDragForce.setZero();
    totalDragTorque.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();

    for (size_t i = 0; i < this->numFacets; i++) {
        projectionTerm = this->scGeometry.facetNormals_B[i].dot(this->v_hat_B);
        projectedArea = this->scGeometry.facetAreas[i] * projectionTerm;
        if (projectedArea > 0.0) {
            facetDragForce = 0.5 * pow(this->v_B.norm(), 2.0) * this->scGeometry.facetCoeffs[i] * projectedArea *
                             this->atmoInData.neutralDensity * (-1.0) * this->v_hat_B;
            facetDragTorque = (-1) * facetDragForce.cross(this->scGeometry.facetLocations_B[i]);
            totalDragForce = totalDragForce + facetDragForce;
            totalDragTorque = totalDragTorque + facetDragTorque;
        }
    }
    this->forceExternal_B = totalDragForce;
    this->torqueExternalPntB_B = totalDragTorque;

    return;
}

/*! This method computes the body forces and torques for the dragEffector in a simulation loop,
selecting the model type based on the settable attribute "modelType."
*/
void FacetDragDynamicEffector::computeForceTorque(double integTime, double timeStep) {
    this->updateDragDir();
    this->plateDrag();
}

/*! This method is called to update the local atmospheric conditions at each timestep.
Naturally, this means that conditions are held piecewise-constant over an integration step.
 @return void
 @param currentSimNanos The current simulation time in nanoseconds
 */
void FacetDragDynamicEffector::updateState(uint64_t currentSimNanos) { this->readInputs(); }
