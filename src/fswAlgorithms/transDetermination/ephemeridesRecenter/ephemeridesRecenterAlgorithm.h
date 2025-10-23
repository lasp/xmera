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

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <Eigen/Core>
#include <array>

inline constexpr int MAX_NUM_CHANGE_BODIES = 20;

/**
 * @brief Container for input/output ephemeris messages used in recentering.
 */
class BodyEphemerisPayload {
   public:
    std::string bodySpiceName;                                          //!< SPICE name of the body
    std::string originalCentralBodyName;                                //!< Original reference body for ephemeris data
    bool isMoon{false};                                                 //!< Body is moon of another body in the list
    EphemerisMsgPayload inputEphemerisPayload{EphemerisMsgPayload{}};   //!< Input ephemeris message
    EphemerisMsgPayload outputEphemerisPayload{EphemerisMsgPayload{}};  //!< Output ephemeris message after recentering
};

/**
 * @brief Basilisk model to recenter ephemeris data from one central body to another.
 *
 * This class processes a set of body ephemerides and recomputes their
 * positions and velocities relative to a new central body.
 */
class EphemeridesRecenterAlgorithm {
   public:
    void reset();
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> updateState(
        const std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES>& newBody);
    size_t getBodyIndexFromName(const std::string& celestialBodyName) const;
    void setNewZeroBaseName(const std::string& bodyName);
    size_t findNewZeroBaseIndex(const std::string& bodyName);
    std::string getNewZeroBase() const;
    void setPreviousCommonZeroBase(const std::string& bodyName);
    std::string getPreviousCommonZeroBase() const;
    size_t getNumberOfBodies() const;
    std::array<std::string, MAX_NUM_CHANGE_BODIES> getAllNames() const;
    void addBodyEphemerisToRecenter(const std::string& bodyName);
    void clearAllBodies();

    ReadFunctor<EphemerisMsgPayload> ephBaseInMsg;  //!< Base ephemeris input message

   private:
    bool findMoonOfBody(const BodyEphemerisPayload& celestialBody, size_t* index) const;

    std::string newCentralBodyName{};
    std::array<std::string, MAX_NUM_CHANGE_BODIES> bodyNames{};
    size_t celestialBodyCount{};  //!< Number of primary bodies
    size_t newCentralIndex{};
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> celestialBodies{};  //!< All celestial bodies involved
    BodyEphemerisPayload previousCentralBody{};                                 //!< Previous reference body
    std::string previousCentralBodyName{};
};
