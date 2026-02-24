// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "navAggregateAlgorithm.h"
#include <architecture/utilities/linearAlgebra.h>

/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param attMsgsPayloads Aggregated attitude navigation messages
 @param transMsgsPayloads Aggregated translational navigation messages
 */
AggregateOutput NavAggregateAlgorithm::update(std::array<NavAttMsgPayload, MAX_AGG_NAV_MSG> attMsgsPayloads,
                                              std::array<NavTransMsgPayload, MAX_AGG_NAV_MSG> transMsgsPayloads) {
    NavAttMsgPayload navAttOutMsgPayload{}; /* [-] local storage of the outgoing attitude navigation message data*/
    NavTransMsgPayload
        navTransOutMsgPayload{}; /* [-] local storage of the outgoing translation navigation message data*/
    AggregateOutput navAggregateOut{};

    /*! - check that attitude navigation messages are present */
    if (this->attMsgCount) {
        /*! - Copy out each part of the attitude source message into the target output message*/
        navAttOutMsgPayload.timeTag = attMsgsPayloads[this->attTimeIdx].timeTag;
        v3Copy(attMsgsPayloads[this->attIdx].sigma_BN, navAttOutMsgPayload.sigma_BN);
        v3Copy(attMsgsPayloads[this->rateIdx].omega_BN_B, navAttOutMsgPayload.omega_BN_B);
        v3Copy(attMsgsPayloads[this->sunIdx].vehSunPntBdy, navAttOutMsgPayload.vehSunPntBdy);
    }

    /*! - check that translation navigation messages are present */
    if (this->transMsgCount) {
        /*! - Copy out each part of the translation source message into the target output message*/
        navTransOutMsgPayload.timeTag = transMsgsPayloads[this->transTimeIdx].timeTag;
        v3Copy(transMsgsPayloads[this->posIdx].r_BN_N, navTransOutMsgPayload.r_BN_N);
        v3Copy(transMsgsPayloads[this->velIdx].v_BN_N, navTransOutMsgPayload.v_BN_N);
        v3Copy(transMsgsPayloads[this->dvIdx].vehAccumDV, navTransOutMsgPayload.vehAccumDV);
    }

    navAggregateOut.navAttOut = navAttOutMsgPayload;
    navAggregateOut.navTransOut = navTransOutMsgPayload;

    return navAggregateOut;
}

/**
 * @brief Set the attitude time index.
 * @param idx The new attitude time index to set.
 */
void NavAggregateAlgorithm::setAttTimeIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        std::string errorMsg = "attTimeIdx (" + std::to_string(idx) +
                               ") must be less than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->attTimeIdx = idx;
}

/**
 * @brief Get the attitude time index.
 * @return uint32_t The current attitude time index.
 */
uint32_t NavAggregateAlgorithm::getAttTimeIdx() const { return this->attTimeIdx; }

/**
 * @brief Set the translation time index.
 * @param idx The new translation time index to set.
 */
void NavAggregateAlgorithm::setTransTimeIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        std::string errorMsg = "transTimeIdx (" + std::to_string(idx) +
                               ") must be less than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->transTimeIdx = idx;
}

/**
 * @brief Get the translation time index.
 * @return uint32_t The current translation time index.
 */
uint32_t NavAggregateAlgorithm::getTransTimeIdx() const { return this->transTimeIdx; }

/**
 * @brief Set the attitude index.
 * @param idx The new attitude index to set.
 */
void NavAggregateAlgorithm::setAttIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        std::string errorMsg = "attIdx (" + std::to_string(idx) +
                               ") must be less than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->attIdx = idx;
}

/**
 * @brief Get the attitude index.
 * @return uint32_t The current attitude index.
 */
uint32_t NavAggregateAlgorithm::getAttIdx() const { return this->attIdx; }

/**
 * @brief Set the rate index.
 * @param idx The new rate index to set.
 */
void NavAggregateAlgorithm::setRateIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        std::string errorMsg = "rateIdx (" + std::to_string(idx) +
                               ") must be less than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->rateIdx = idx;
}

/**
 * @brief Get the rate index.
 * @return uint32_t The current rate index.
 */
uint32_t NavAggregateAlgorithm::getRateIdx() const { return this->rateIdx; }

/**
 * @brief Set the position index.
 * @param idx The new position index to set.
 */
void NavAggregateAlgorithm::setPosIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        std::string errorMsg = "posIdx (" + std::to_string(idx) +
                               ") must be less than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->posIdx = idx;
}

/**
 * @brief Get the position index.
 * @return uint32_t The current position index.
 */
uint32_t NavAggregateAlgorithm::getPosIdx() const { return this->posIdx; }

/**
 * @brief Set the velocity index.
 * @param idx The new velocity index to set.
 */
void NavAggregateAlgorithm::setVelIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        std::string errorMsg = "velIdx (" + std::to_string(idx) +
                               ") must be less than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->velIdx = idx;
}

/**
 * @brief Get the velocity index.
 * @return uint32_t The current velocity index.
 */
uint32_t NavAggregateAlgorithm::getVelIdx() const { return this->velIdx; }

/**
 * @brief Set the accumulated DV index.
 * @param idx The new accumulated DV index to set.
 */
void NavAggregateAlgorithm::setDvIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        std::string errorMsg = "dvIdx (" + std::to_string(idx) +
                               ") must be less than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->dvIdx = idx;
}

/**
 * @brief Get the accumulated DV index.
 * @return uint32_t The current accumulated DV index.
 */
uint32_t NavAggregateAlgorithm::getDvIdx() const { return this->dvIdx; }

/**
 * @brief Set the sun index.
 * @param idx The new sun index to set.
 */
void NavAggregateAlgorithm::setSunIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        std::string errorMsg = "sunIdx (" + std::to_string(idx) +
                               ") must be less than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->sunIdx = idx;
}

/**
 * @brief Get the sun index.
 * @return uint32_t The current sun index.
 */
uint32_t NavAggregateAlgorithm::getSunIdx() const { return this->sunIdx; }

/**
 * @brief Set the attitude message count.
 * @param msgCount The new attitude message count to set.
 */
void NavAggregateAlgorithm::setAttMsgCount(uint32_t msgCount) {
    if (msgCount > MAX_AGG_NAV_MSG) {
        std::string errorMsg = "attMsgCount (" + std::to_string(msgCount) +
                               ") must not be greater than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->attMsgCount = msgCount;
}

/**
 * @brief Get the attitude message count.
 * @return uint32_t The current attitude message count.
 */
uint32_t NavAggregateAlgorithm::getAttMsgCount() const { return this->attMsgCount; }

/**
 * @brief Set the translational message count.
 * @param msgCount The new translational message count to set.
 */
void NavAggregateAlgorithm::setTransMsgCount(uint32_t msgCount) {
    if (msgCount > MAX_AGG_NAV_MSG) {
        std::string errorMsg = "transMsgCount (" + std::to_string(msgCount) +
                               ") must not be greater than maximum navAggregate message size (" +
                               std::to_string(MAX_AGG_NAV_MSG) + ").";
        throw std::invalid_argument(errorMsg);
    }
    this->transMsgCount = msgCount;
}

/**
 * @brief Get the translational message count.
 * @return uint32_t The current translational message count.
 */
uint32_t NavAggregateAlgorithm::getTransMsgCount() const { return this->transMsgCount; }
