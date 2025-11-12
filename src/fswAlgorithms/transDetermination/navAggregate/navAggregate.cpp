#include "navAggregate.h"

/*! This resets the module to original states.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void NavAggregate::reset(uint64_t callTime) {
    /*! - loop over the number of attitude input messages and make sure they are linked */
    for (uint32_t i = 0; i < this->getAttMsgCount(); i = i + 1) {
        if (!this->attMsgs[i].navAttInMsg.isLinked()) {
            throw std::invalid_argument(
                "An attitude input message name was not linked. "
                "Be sure that the number of linked messages corresponds to attMsgCount.");
        }
    }
    /*! - loop over the number of translational input messages and make sure they are linked */
    for (uint32_t i = 0; i < this->getTransMsgCount(); i = i + 1) {
        if (!this->transMsgs[i].navTransInMsg.isLinked()) {
            throw std::invalid_argument(
                "A translation input message name was not linked. "
                "Be sure that the number of linked messages corresponds to transMsgCount.");
        }
    }

    //! - zero the arrays of input messages
    for (uint32_t i = 0; i < MAX_AGG_NAV_MSG; i++) {
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
void NavAggregate::updateState(uint64_t callTime) {
    uint32_t i;
    std::array<NavAttMsgPayload, MAX_AGG_NAV_MSG> attMsgsPayloads{};
    std::array<NavTransMsgPayload, MAX_AGG_NAV_MSG> transMsgsPayloads{};

    /*! - check that attitude navigation messages are present */
    if (this->getAttMsgCount()) {
        /*! - Iterate through all of the attitude input messages, clear local Msg buffer and archive the new nav data */
        for (i = 0; i < this->getAttMsgCount(); i = i + 1) {
            this->attMsgs[i].msgStorage = this->attMsgs[i].navAttInMsg();
            attMsgsPayloads[i] = this->attMsgs[i].navAttInMsg();
        }
    }

    /*! - check that translation navigation messages are present */
    if (this->getTransMsgCount()) {
        /*! - Iterate through all of the translation input messages, clear local Msg buffer and archive the new nav data
         */
        for (i = 0; i < this->getTransMsgCount(); i = i + 1) {
            this->transMsgs[i].msgStorage = this->transMsgs[i].navTransInMsg();
            transMsgsPayloads[i] = this->transMsgs[i].navTransInMsg();
        }
    }

    AggregateOutput navAggregateOut = this->algorithm.update(attMsgsPayloads, transMsgsPayloads);

    this->navAttOutMsg.write(&navAggregateOut.navAttOut, this->moduleID, callTime);
    this->navTransOutMsg.write(&navAggregateOut.navTransOut, this->moduleID, callTime);
}

/**
 * @brief Set the attitude time index.
 * @param idx The new attitude time index to set.
 */
void NavAggregate::setAttTimeIdx(uint32_t idx) { this->algorithm.setAttTimeIdx(idx); }

/**
 * @brief Get the attitude time index.
 * @return uint32_t The current attitude time index.
 */
uint32_t NavAggregate::getAttTimeIdx() const { return this->algorithm.getAttTimeIdx(); }

/**
 * @brief Set the translation time index.
 * @param idx The new translation time index to set.
 */
void NavAggregate::setTransTimeIdx(uint32_t idx) { this->algorithm.setTransTimeIdx(idx); }

/**
 * @brief Get the translation time index.
 * @return uint32_t The current translation time index.
 */
uint32_t NavAggregate::getTransTimeIdx() const { return this->algorithm.getTransTimeIdx(); }

/**
 * @brief Set the attitude index.
 * @param idx The new attitude index to set.
 */
void NavAggregate::setAttIdx(uint32_t idx) { this->algorithm.setAttIdx(idx); }

/**
 * @brief Get the attitude index.
 * @return uint32_t The current attitude index.
 */
uint32_t NavAggregate::getAttIdx() const { return this->algorithm.getAttIdx(); }

/**
 * @brief Set the rate index.
 * @param idx The new rate index to set.
 */
void NavAggregate::setRateIdx(uint32_t idx) { this->algorithm.setRateIdx(idx); }

/**
 * @brief Get the rate index.
 * @return uint32_t The current rate index.
 */
uint32_t NavAggregate::getRateIdx() const { return this->algorithm.getRateIdx(); }

/**
 * @brief Set the position index.
 * @param idx The new position index to set.
 */
void NavAggregate::setPosIdx(uint32_t idx) { this->algorithm.setPosIdx(idx); }

/**
 * @brief Get the position index.
 * @return uint32_t The current position index.
 */
uint32_t NavAggregate::getPosIdx() const { return this->algorithm.getPosIdx(); }

/**
 * @brief Set the velocity index.
 * @param idx The new velocity index to set.
 */
void NavAggregate::setVelIdx(uint32_t idx) { this->algorithm.setVelIdx(idx); }

/**
 * @brief Get the velocity index.
 * @return uint32_t The current velocity index.
 */
uint32_t NavAggregate::getVelIdx() const { return this->algorithm.getVelIdx(); }

/**
 * @brief Set the accumulated DV index.
 * @param idx The new accumulated DV index to set.
 */
void NavAggregate::setDvIdx(uint32_t idx) { this->algorithm.setDvIdx(idx); }

/**
 * @brief Get the accumulated DV index.
 * @return uint32_t The current accumulated DV index.
 */
uint32_t NavAggregate::getDvIdx() const { return this->algorithm.getDvIdx(); }

/**
 * @brief Set the sun index.
 * @param idx The new sun index to set.
 */
void NavAggregate::setSunIdx(uint32_t idx) { this->algorithm.setSunIdx(idx); }

/**
 * @brief Get the sun index.
 * @return uint32_t The current sun index.
 */
uint32_t NavAggregate::getSunIdx() const { return this->algorithm.getSunIdx(); }

/**
 * @brief Set the attitude message count.
 * @param msgCount The new attitude message count to set.
 */
void NavAggregate::setAttMsgCount(uint32_t msgCount) { this->algorithm.setAttMsgCount(msgCount); }

/**
 * @brief Get the attitude message count.
 * @return uint32_t The current attitude message count.
 */
uint32_t NavAggregate::getAttMsgCount() const { return this->algorithm.getAttMsgCount(); }

/**
 * @brief Set the translational message count.
 * @param msgCount The new translational message count to set.
 */
void NavAggregate::setTransMsgCount(uint32_t msgCount) { this->algorithm.setTransMsgCount(msgCount); }

/**
 * @brief Get the translational message count.
 * @return uint32_t The current translational message count.
 */
uint32_t NavAggregate::getTransMsgCount() const { return this->algorithm.getTransMsgCount(); }
