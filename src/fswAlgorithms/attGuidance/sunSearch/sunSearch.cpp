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

#include "sunSearch.h"

/*! This method is used to reset the module.
 @return void
 */
void SunSearch::reset(uint64_t currentSimNanos) {
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("SunSearch.attNavInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("SunSearch.vehConfigInMsg wasn't connected.");
    }

    this->algorithm.reset(currentSimNanos, this->vehConfigInMsg());
}

/*! This method is the main carrier for the computation of the guidance message
 @return void
 @param currentSimNanos The current simulation time for system
 */
void SunSearch::updateState(uint64_t currentSimNanos) {
    NavAttMsgPayload navAttIn = this->attNavInMsg();
    AttGuidMsgPayload attGuidOut = this->algorithm.update(currentSimNanos, navAttIn);

    this->attGuidOutMsg.write(&attGuidOut, this->moduleID, currentSimNanos);
}

/**
 * @brief Set the properties of a slew maneuver
 * @param slewPropertiesInput the properties of the slew maneuver
 */
void SunSearch::setSlewProperties(SlewProperties slewPropertiesInput) {
    this->algorithm.setSlewProperties(slewPropertiesInput);
}

/**
 * @brief Modify the properties of a slew maneuver
 * @param slewPropertiesInput the properties of the slew maneuver
 * @param index index of the slew maneuver
 */
void SunSearch::modifySlewProperties(SlewProperties slewPropertiesInput, uint32_t index) {
    this->algorithm.modifySlewProperties(slewPropertiesInput, index);
}

/**
 * @brief Get the properties of a slew maneuver
 * @param index index of the slew maneuver
 * @return SlewProperties the properties of the slew maneuver
 */
SlewProperties SunSearch::getSlewProperties(uint32_t index) const { return this->algorithm.getSlewProperties(index); }
