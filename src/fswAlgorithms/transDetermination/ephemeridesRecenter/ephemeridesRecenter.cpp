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

#include "ephemeridesRecenter.h"

/*! @brief This method resets the module.
 @return void
 @param callTime : The clock time at which the function was called (nanoseconds)
 */
void EphemeridesRecenter::reset(uint64_t callTime) {
    for (auto i = 0; i < this->ephemeridesNumber; ++i) {
        if (!this->ephemerides[i].inputEphemerisMsg.isLinked()) {
            throw std::invalid_argument("Input ephemeris message was not connected for " +
                                        this->ephemerides[i].bodySpiceName);
        }
    }
    this->ephemeridesNumber = this->getNumberOfBodies();
    this->algorithm.reset();
}

/*! @brief This method recomputes the body positions and velocities relative to
    the base body ephemeris and writes out updated ephemeris position and velocity
    for each body.
 @return void
 @param callTime : The clock time at which the function was called (nanoseconds)
 */
void EphemeridesRecenter::updateState(uint64_t callTime) {
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> bodyPayloads{};
    for (auto i = 0; i < this->ephemeridesNumber; ++i) {
        BodyEphemerisPayload newBodyPayload{};
        newBodyPayload.bodySpiceName = this->ephemerides[i].bodySpiceName;
        newBodyPayload.originalCentralBodyName = this->ephemerides[i].originalCentralBodyName;
        newBodyPayload.inputEphemerisPayload = this->ephemerides[i].inputEphemerisMsg();
        bodyPayloads[i] = newBodyPayload;
    }

    auto outputPayloads = this->algorithm.updateState(bodyPayloads);

    for (auto i = 0; i < this->ephemeridesNumber; ++i) {
        this->recenteredEphemerisOutputMsgs[i]->write(
            &outputPayloads[i].outputEphemerisPayload, this->moduleID, callTime);
    }
}

/*! @brief Add a body to be re-centered.
 @return void
 @param ephemerisBody BodyEphemeris : A new celestial body instance
 */
void EphemeridesRecenter::addBodyEphemerisToRecenter(const BodyEphemeris& ephemerisBody) {
    if (this->ephemeridesNumber + 1 > MAX_NUM_CHANGE_BODIES) {
        throw std::invalid_argument("Adding one body too many to the list");
    }
    this->recenteredEphemerisOutputMsgs.push_back(new Message<EphemerisMsgPayload>);
    this->ephemerides[this->ephemeridesNumber] = ephemerisBody;
    this->ephemeridesNumber += 1;
    this->algorithm.addBodyEphemerisToRecenter(ephemerisBody.bodySpiceName);
}

/*! @brief Set a new celestial body center by name
 @return void
 @param bodyName std::string : the new zero base
 */
void EphemeridesRecenter::setNewZeroBase(const std::string& bodyName) { this->algorithm.setNewZeroBaseName(bodyName); }

/*! @brief Get the new celestial body center by name
 @return std::string : the new zero base
 */
std::string EphemeridesRecenter::getNewZeroBase() const { return this->algorithm.getNewZeroBase(); }

/*! @brief Set the previous common zero base of all the celestial bodies entered
 @param bodyName std::string : the new zero base
 */
void EphemeridesRecenter::setPreviousCommonZeroBase(const std::string& bodyName) {
    this->algorithm.setPreviousCommonZeroBase(bodyName);
}

/*! @brief Get the previous common zero base of all the celestial bodies entered
 @return std::string : the new zero base
 */
std::string EphemeridesRecenter::getPreviousCommonZeroBase() const {
    return this->algorithm.getPreviousCommonZeroBase();
}

/*! @brief Get the number of bodies that were entered into the module
 @return size_t : the number of bodies
 */
size_t EphemeridesRecenter::getNumberOfBodies() const { return this->algorithm.getNumberOfBodies(); }

/*! @brief Get the index of a body
 @param celestialBodyName std::string : celestial body name
 @return size_t : whether or not the index was found
 */
size_t EphemeridesRecenter::getBodyIndexFromName(const std::string& celestialBodyName) const {
    return this->algorithm.getBodyIndexFromName(celestialBodyName);
}

/*! @brief Get all the names of the bodies entered
 @return std::array<std::string, MAX_NUM_CHANGE_BODIES> : an array of names
 */
std::array<std::string, MAX_NUM_CHANGE_BODIES> EphemeridesRecenter::getAllNames() const {
    return this->algorithm.getAllNames();
}

/*! @brief Clear all the bodies from the current list
 */
void EphemeridesRecenter::clearAllBodies() {
    BodyEphemeris emptyBody{};
    std::fill(this->ephemerides.begin(), this->ephemerides.end(), emptyBody);
    this->ephemeridesNumber = 0;
    this->algorithm.clearAllBodies();
}
