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

#ifndef _EPHEM_RECENTER_H_
#define _EPHEM_RECENTER_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include "ephemeridesRecenterAlgorithm.h"
#include <assert.h>
#include <Eigen/Core>
#include <array>

/*! @brief Container class for the input and output messages that will be re-centered on the base ephemeris. */
class BodyEphemeris {
   public:
    std::string bodySpiceName;
    std::string originalCentralBodyName;
    ReadFunctor<EphemerisMsgPayload> inputEphemerisMsg{};
    Message<EphemerisMsgPayload> outputEphemerisMsg{};
};

/*! @brief Ephemerides Recenter Basilisk Model */
class EphemeridesRecenter : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    void addBodyEphemerisToRecenter(const BodyEphemeris& ephemerisBody);
    void setNewZeroBase(const std::string& bodyName);
    std::string getNewZeroBase() const;
    void setPreviousCommonZeroBase(const std::string& bodyName);
    std::string getPreviousCommonZeroBase() const;
    std::array<std::string, MAX_NUM_CHANGE_BODIES> getAllNames() const;
    size_t getBodyIndexFromName(const std::string& celestialBodyName) const;
    size_t getNumberOfBodies() const;
    void clearAllBodies();
    std::vector<Message<EphemerisMsgPayload>*> recenteredEphemerisOutputMsgs{};

   private:
    size_t ephemeridesNumber{};
    std::array<BodyEphemeris, MAX_NUM_CHANGE_BODIES> ephemerides{};
    EphemeridesRecenterAlgorithm algorithm{};
};

#endif
