/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
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
