// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
