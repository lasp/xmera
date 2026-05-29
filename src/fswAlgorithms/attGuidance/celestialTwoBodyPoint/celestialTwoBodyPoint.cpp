// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "celestialTwoBodyPoint.h"

void CelestialTwoBodyPoint::reset(uint64_t callTime) {
    this->secCelBodyIsLinked = this->secCelBodyInMsg.isLinked();

    // check if required input messages have been included
    if (!this->transNavInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.transNavInMsg was not linked.");
    }
    if (!this->celBodyInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.celBodyInMsg was not linked.");
    }

    this->algorithm.reset(this->secCelBodyIsLinked);
}

/*! This method takes the spacecraft and points a specified axis at a named
 celestial body specified in the configuration data.  It generates the
 commanded attitude and assumes that the control errors are computed
 downstream.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CelestialTwoBodyPoint::updateState(uint64_t callTime) {
    NavTransMsgPayload transNavIn = this->transNavInMsg();
    EphemerisMsgPayload celBodyIn = this->celBodyInMsg();
    EphemerisMsgPayload secCelBodyIn{};
    if (this->secCelBodyIsLinked) {
        secCelBodyIn = this->secCelBodyInMsg();
    }

    AttRefMsgPayload attRefOut = this->algorithm.update(celBodyIn, secCelBodyIn, transNavIn);

    /*! - Write the output message */
    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}

/**
 * @brief Set the singularity threshold
 * @param thresh singularity threshold
 */
void CelestialTwoBodyPoint::setSingularityThresh(double thresh) { this->algorithm.setSingularityThresh(thresh); }

/**
 * @brief Get the singularity threshold
 * @return double singularity threshold
 */
double CelestialTwoBodyPoint::getSingularityThresh() const { return this->algorithm.getSingularityThresh(); }
