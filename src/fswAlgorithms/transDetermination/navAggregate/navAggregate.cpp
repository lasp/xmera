/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

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

#include "navAggregate.h"
#include "architecture/utilities/linearAlgebra.h"
#include <cstdio>


/*! This resets the module to original states.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::reset(uint64_t callTime)
{
    /*! - loop over the number of attitude input messages and make sure they are linked */
    for(uint32_t i=0; i<this->attMsgCount; i=i+1)
    {
        if (!this->attMsgs[i].navAttInMsg.isLinked()) {
            throw std::invalid_argument(
                "An attitude input message name was not linked. "
                "Be sure that the number of linked messages corresponds to attMsgCount.");
        }
    }
    /*! - loop over the number of translational input messages and make sure they are linked */
    for(uint32_t i=0; i<this->transMsgCount; i=i+1)
    {
        if (!this->transMsgs[i].navTransInMsg.isLinked()) {
            throw std::invalid_argument(
                "A translation input message name was not linked. "
                "Be sure that the number of linked messages corresponds to transMsgCount.");
        }
    }

    //! - zero the arrays of input messages
    for (uint32_t i=0; i< MAX_AGG_NAV_MSG; i++) {
        this->attMsgs[i].msgStorage = NavAttMsgPayload();
        this->transMsgs[i].msgStorage = NavTransMsgPayload();
    }
}


/*! This method takes the navigation message snippets created by the various
    navigation components in the FSW and aggregates them into a single complete
    navigation message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::updateState(uint64_t callTime)
{
    uint32_t i;
    NavAttMsgPayload navAttOutMsgBuffer = NavAttMsgPayload();     /* [-] The local storage of the outgoing attitude navibation message data*/
    NavTransMsgPayload navTransOutMsgBuffer = NavTransMsgPayload(); /* [-] The local storage of the outgoing message data*/

    /*! - check that attitude navigation messages are present */
    if (this->attMsgCount) {
        /*! - Iterate through all of the attitude input messages, clear local Msg buffer and archive the new nav data */
        for(i=0; i<this->attMsgCount; i=i+1)
        {
            this->attMsgs[i].msgStorage = this->attMsgs[i].navAttInMsg();
        }

        /*! - Copy out each part of the attitude source message into the target output message*/
        navAttOutMsgBuffer.timeTag = this->attMsgs[this->attTimeIdx].msgStorage.timeTag;
        v3Copy(this->attMsgs[this->attIdx].msgStorage.sigma_BN, navAttOutMsgBuffer.sigma_BN);
        v3Copy(this->attMsgs[this->rateIdx].msgStorage.omega_BN_B, navAttOutMsgBuffer.omega_BN_B);
        v3Copy(this->attMsgs[this->sunIdx].msgStorage.vehSunPntBdy, navAttOutMsgBuffer.vehSunPntBdy);

    }

    /*! - check that translation navigation messages are present */
    if (this->transMsgCount) {
        /*! - Iterate through all of the translation input messages, clear local Msg buffer and archive the new nav data */
        for(i=0; i<this->transMsgCount; i=i+1)
        {
            this->transMsgs[i].msgStorage = this->transMsgs[i].navTransInMsg();
        }

        /*! - Copy out each part of the translation source message into the target output message*/
        navTransOutMsgBuffer.timeTag = this->transMsgs[this->transTimeIdx].msgStorage.timeTag;
        v3Copy(this->transMsgs[this->posIdx].msgStorage.r_BN_N, navTransOutMsgBuffer.r_BN_N);
        v3Copy(this->transMsgs[this->velIdx].msgStorage.v_BN_N, navTransOutMsgBuffer.v_BN_N);
        v3Copy(this->transMsgs[this->dvIdx].msgStorage.vehAccumDV, navTransOutMsgBuffer.vehAccumDV);
    }

    /*! - Write the total message out for everyone else to pick up */
    this->navAttOutMsg.write(&navAttOutMsgBuffer, this->moduleID, callTime);
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, callTime);
}

/**
 * @brief Set the attitude time index.
 * @param idx The new attitude time index to set.
 */
void NavAggregate::setAttTimeIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "attTimeIdx (%i) must be less than maximum navAggregate message size (%i).",
                idx, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->attTimeIdx = idx;
}

/**
 * @brief Get the attitude time index.
 * @return uint32_t The current attitude time index.
 */
uint32_t NavAggregate::getAttTimeIdx() const { return this->attTimeIdx; }

/**
 * @brief Set the translation time index.
 * @param idx The new translation time index to set.
 */
void NavAggregate::setTransTimeIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "transTimeIdx (%i) must be less than maximum navAggregate message size (%i).",
                idx, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->transTimeIdx = idx;
}

/**
 * @brief Get the translation time index.
 * @return uint32_t The current translation time index.
 */
uint32_t NavAggregate::getTransTimeIdx() const { return this->transTimeIdx; }

/**
 * @brief Set the attitude index.
 * @param idx The new attitude index to set.
 */
void NavAggregate::setAttIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "attIdx (%i) must be less than maximum navAggregate message size (%i).",
                idx, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->attIdx = idx;
}

/**
 * @brief Get the attitude index.
 * @return uint32_t The current attitude index.
 */
uint32_t NavAggregate::getAttIdx() const { return this->attIdx; }

/**
 * @brief Set the rate index.
 * @param idx The new rate index to set.
 */
void NavAggregate::setRateIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "rateIdx (%i) must be less than maximum navAggregate message size (%i).",
                idx, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->rateIdx = idx;
}

/**
 * @brief Get the rate index.
 * @return uint32_t The current rate index.
 */
uint32_t NavAggregate::getRateIdx() const { return this->rateIdx; }

/**
 * @brief Set the position index.
 * @param idx The new position index to set.
 */
void NavAggregate::setPosIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "posIdx (%i) must be less than maximum navAggregate message size (%i).",
                idx, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->posIdx = idx;
}

/**
 * @brief Get the position index.
 * @return uint32_t The current position index.
 */
uint32_t NavAggregate::getPosIdx() const { return this->posIdx; }

/**
 * @brief Set the velocity index.
 * @param idx The new velocity index to set.
 */
void NavAggregate::setVelIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "velIdx (%i) must be less than maximum navAggregate message size (%i).",
                idx, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->velIdx = idx;
}

/**
 * @brief Get the velocity index.
 * @return uint32_t The current velocity index.
 */
uint32_t NavAggregate::getVelIdx() const { return this->velIdx; }

/**
 * @brief Set the accumulated DV index.
 * @param idx The new accumulated DV index to set.
 */
void NavAggregate::setDvIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "dvIdx (%i) must be less than maximum navAggregate message size (%i).",
                idx, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->dvIdx = idx;
}

/**
 * @brief Get the accumulated DV index.
 * @return uint32_t The current accumulated DV index.
 */
uint32_t NavAggregate::getDvIdx() const { return this->dvIdx; }

/**
 * @brief Set the sun index.
 * @param idx The new sun index to set.
 */
void NavAggregate::setSunIdx(uint32_t idx) {
    if (idx >= MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "sunIdx (%i) must be less than maximum navAggregate message size (%i).",
                idx, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->sunIdx = idx;
}

/**
 * @brief Get the sun index.
 * @return uint32_t The current sun index.
 */
uint32_t NavAggregate::getSunIdx() const { return this->sunIdx; }

/**
 * @brief Set the attitude message count.
 * @param msgCount The new attitude message count to set.
 */
void NavAggregate::setAttMsgCount(uint32_t msgCount) {
    if (msgCount > MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "attMsgCount (%i) must not be greater than maximum navAggregate message size (%i).",
                msgCount, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->attMsgCount = msgCount;
}

/**
 * @brief Get the attitude message count.
 * @return uint32_t The current attitude message count.
 */
uint32_t NavAggregate::getAttMsgCount() const { return this->attMsgCount; }

/**
 * @brief Set the translational message count.
 * @param msgCount The new translational message count to set.
 */
void NavAggregate::setTransMsgCount(uint32_t msgCount) {
    if (msgCount > MAX_AGG_NAV_MSG) {
        char errorMsg[MAX_LOGGING_LENGTH];
        snprintf(errorMsg, sizeof(errorMsg), "transMsgCount (%i) must not be greater than maximum navAggregate message size (%i).",
                msgCount, MAX_AGG_NAV_MSG);
        throw std::invalid_argument(errorMsg);
    }
    this->transMsgCount = msgCount;
}

/**
 * @brief Get the translational message count.
 * @return uint32_t The current translational message count.
 */
uint32_t NavAggregate::getTransMsgCount() const { return this->transMsgCount; }
