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

#include "ephemeridesRecenterAlgorithm.h"

/*! @brief Subtract two C-array vectors
 @param v1 double[3] : vector 1
 @param v2 double[3] : vector 2
 @param result double[3] : subtracted vectors
 */
static void vectorSubtraction(const double v1[3], const double v2[3], double result[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = v1[i] - v2[i];
    }
}

/*! @brief Add two C-array vectors
 @param v1 double[3] : vector 1
 @param v2 double[3] : vector 2
 @param result double[3] : added vectors
 */
static void vectorAddition(const double v1[3], const double v2[3], double result[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = v1[i] + v2[i];
    }
}

void EphemeridesRecenterAlgorithm::reset() {
    this->newCentralIndex = this->findNewZeroBaseIndex(this->newCentralBodyName);
}

/*! @brief Recenter the ephemerides
 @param newBodies std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : input bodies
 @return std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> : re-centered bodies
 */
std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> EphemeridesRecenterAlgorithm::updateState(
    const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBodies) {
    this->celestialBodies = newBodies;

    auto newCentralBody = this->celestialBodies[this->newCentralIndex];
    EphemerisMsgPayload newCentralBodyPayload = newCentralBody.inputEphemerisPayload;
    /* - If the new central body is a moon (its original central body is not the common central body but another body in
     * the list) first re-center the moon around the common central body so that every body is relative to the common
     * center*/
    if (newCentralBody.originalCentralBodyName != this->previousCentralBodyName) {
        auto moonCentralBodyIndex = this->getBodyIndexFromName(newCentralBody.originalCentralBodyName);
        auto moonCentralBodyInput = this->celestialBodies[moonCentralBodyIndex].inputEphemerisPayload;
        vectorAddition(
            newCentralBodyPayload.r_BdyZero_N, moonCentralBodyInput.r_BdyZero_N, newCentralBodyPayload.r_BdyZero_N);
        vectorAddition(
            newCentralBodyPayload.v_BdyZero_N, moonCentralBodyInput.v_BdyZero_N, newCentralBodyPayload.v_BdyZero_N);
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> recenteredBodies{};
    for (size_t i = 0; i < this->celestialBodyCount; ++i) {
        /* Moons get re-centered along with their central body and shouldn't be re-centered in this main loop */
        if (!recenteredBodies[i].isMoon) {
            recenteredBodies[i] = BodyEphemerisPayload{};
            EphemerisMsgPayload newEphemerisToRecenterPayload = newBodies[i].inputEphemerisPayload;
            if (this->celestialBodies[i].originalCentralBodyName != newCentralBody.bodySpiceName &&
                this->celestialBodies[i].originalCentralBodyName == this->previousCentralBodyName) {
                vectorSubtraction(newEphemerisToRecenterPayload.r_BdyZero_N,
                                  newCentralBodyPayload.r_BdyZero_N,
                                  newEphemerisToRecenterPayload.r_BdyZero_N);
                vectorSubtraction(newEphemerisToRecenterPayload.v_BdyZero_N,
                                  newCentralBodyPayload.v_BdyZero_N,
                                  newEphemerisToRecenterPayload.v_BdyZero_N);

                if (size_t moonIndex{}; this->findMoonOfBody(this->celestialBodies[i], &moonIndex) &&
                                        this->celestialBodies[i].bodySpiceName != this->previousCentralBodyName) {
                    EphemerisMsgPayload moonOfBodyPayload = this->celestialBodies[moonIndex].inputEphemerisPayload;
                    vectorAddition(newEphemerisToRecenterPayload.r_BdyZero_N,
                                   moonOfBodyPayload.r_BdyZero_N,
                                   moonOfBodyPayload.r_BdyZero_N);
                    vectorAddition(newEphemerisToRecenterPayload.v_BdyZero_N,
                                   moonOfBodyPayload.v_BdyZero_N,
                                   moonOfBodyPayload.v_BdyZero_N);

                    recenteredBodies[moonIndex].bodySpiceName = this->celestialBodies[moonIndex].bodySpiceName;
                    recenteredBodies[moonIndex].isMoon = true;
                    recenteredBodies[moonIndex].originalCentralBodyName =
                        this->celestialBodies[moonIndex].originalCentralBodyName;
                    recenteredBodies[moonIndex].inputEphemerisPayload =
                        this->celestialBodies[moonIndex].inputEphemerisPayload;
                    recenteredBodies[moonIndex].outputEphemerisPayload = moonOfBodyPayload;
                }
            }
            recenteredBodies[i] = newBodies[i];
            recenteredBodies[i].outputEphemerisPayload = newEphemerisToRecenterPayload;
        }
    }
    return recenteredBodies;
}

/*! @brief Find the moon index of a given body
 @param celestialBody std::string : celestial body name
 @param index size_t* : index
 @return bool : whether the index was found
 */
bool EphemeridesRecenterAlgorithm::findMoonOfBody(const BodyEphemerisPayload& celestialBody, size_t* index) const {
    if (this->celestialBodyCount == 0) {
        throw std::invalid_argument("Requesting a body index but the current celestial body count is 0");
    }
    for (size_t i = 0; i < this->celestialBodyCount; ++i) {
        if (this->celestialBodies[i].originalCentralBodyName == celestialBody.bodySpiceName) {
            *index = i;
            return true;
        }
    }
    return false;
}

/*! @brief Get the index of a body
 @param celestialBodyName std::string : celestial body name
 @return size_t : index
 */
size_t EphemeridesRecenterAlgorithm::getBodyIndexFromName(const std::string& celestialBodyName) const {
    if (this->celestialBodyCount == 0) {
        throw std::invalid_argument("Requesting a body index but the current celestial body count is 0");
    }
    for (size_t i = 0; i < this->celestialBodyCount; ++i) {
        if (this->bodyNames[i] == celestialBodyName) {
            return i;
        }
    }
    throw std::invalid_argument("Requesting a body index but the current celestial body count is 0");
}

/*! @brief Set the new zero base body type by name
 @param bodyName std::string : the new zero base
 */
void EphemeridesRecenterAlgorithm::setNewZeroBaseName(const std::string& bodyName) {
    this->newCentralBodyName = bodyName;
}

/*! @brief Find the new zero base body type by name
 @param bodyName std::string : the new zero base
 */
size_t EphemeridesRecenterAlgorithm::findNewZeroBaseIndex(const std::string& bodyName) {
    if (auto indexOfNewZeroBase = std::find(this->bodyNames.begin(), this->bodyNames.end(), bodyName);
        indexOfNewZeroBase == this->bodyNames.end()) {
        throw std::invalid_argument("New zero base body was not in the list of existing bodies");
    } else {
        return std::distance(this->bodyNames.begin(), indexOfNewZeroBase);
    }
}

/*! @brief Get the new celestial body center by name
 @return std::string : the new zero base
 */
std::string EphemeridesRecenterAlgorithm::getNewZeroBase() const { return this->newCentralBodyName; }

/*! @brief Set the previous common zero base of all the celestial bodies entered
 @param bodyName std::string : the new zero base
 */
void EphemeridesRecenterAlgorithm::setPreviousCommonZeroBase(const std::string& bodyName) {
    if (auto indexOfPreviousZeroBase = std::find(this->bodyNames.begin(), this->bodyNames.end(), bodyName);
        indexOfPreviousZeroBase == this->bodyNames.end()) {
        throw std::invalid_argument("Previous zero base body was not in the list of existing bodies");
    }

    this->previousCentralBodyName = bodyName;
}

/*! @brief Get the previous common zero base of all the celestial bodies entered
 @return std::string : the new zero base
 */
std::string EphemeridesRecenterAlgorithm::getPreviousCommonZeroBase() const { return this->previousCentralBodyName; }

/*! @brief Get the number of bodies that were entered into the module
 @return size_t : the number of bodies
 */
size_t EphemeridesRecenterAlgorithm::getNumberOfBodies() const { return this->celestialBodyCount; }

/*! @brief Get all the names of the bodies entered
 @return std::array<std::string, MAX_NUM_CHANGE_BODIES> : an array of names
 */
std::array<std::string, MAX_NUM_CHANGE_BODIES> EphemeridesRecenterAlgorithm::getAllNames() const {
    if (this->celestialBodyCount == 0) {
        throw std::invalid_argument("Requesting all body names but the current celestial body count is 0");
    }
    std::array<std::string, MAX_NUM_CHANGE_BODIES> names{};
    for (size_t i = 0; i < this->celestialBodyCount; ++i) {
        if (!this->bodyNames[i].empty()) {
            names[i] = this->bodyNames[i];
        }
    }
    return names;
}

/*! @brief Add celestial body by name
 @param bodyName std::string : the body name to add
 */
void EphemeridesRecenterAlgorithm::addBodyEphemerisToRecenter(const std::string& bodyName) {
    if (this->celestialBodyCount + 1 > MAX_NUM_CHANGE_BODIES) {
        throw std::invalid_argument("Adding one body too many to the list");
    }
    if (auto indexInList = std::find(this->bodyNames.begin(), this->bodyNames.end(), bodyName);
        indexInList != this->bodyNames.end()) {
        throw std::invalid_argument("Body already added to list");
    }
    this->bodyNames[this->celestialBodyCount] = bodyName;
    this->celestialBodyCount += 1;
}

/*! @brief Clear all the bodies from the current list
 */
void EphemeridesRecenterAlgorithm::clearAllBodies() {
    BodyEphemerisPayload emptyBody{};
    const std::string emptyString{};
    std::fill(this->celestialBodies.begin(), this->celestialBodies.end(), emptyBody);
    std::fill(this->bodyNames.begin(), this->bodyNames.end(), emptyString);
    this->celestialBodyCount = 0;
}
