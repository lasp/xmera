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

#ifndef NAV_AGGREGATE_H
#define NAV_AGGREGATE_H

#include <stdint.h>

#include <array>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/NavAttMsgPayload.h"
#include "architecture/msgPayloadDef/NavTransMsgPayload.h"
#include "fswAlgorithms/transDetermination/navAggregate/navAggregateAlgorithm.h"

/*! structure containing the attitude navigation message name, ID and local buffer*/
typedef struct {
    ReadFunctor<NavAttMsgPayload> navAttInMsg; /*!< attitude navigation input message*/
    NavAttMsgPayload msgStorage;               /*!< [-] Local buffer to store nav message*/
} AggregateAttInput;

/*! structure containing the translational navigation message name, ID and local buffer*/
typedef struct {
    ReadFunctor<NavTransMsgPayload> navTransInMsg; /*!< translation navigation input message*/
    NavTransMsgPayload msgStorage;                 /*!< [-] Local buffer to store nav message*/
} AggregateTransInput;

class NavAggregate : public SysModel {
   public:
    NavAggregate() = default;
    ~NavAggregate() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setAttTimeIdx(uint32_t idx);
    uint32_t getAttTimeIdx() const;
    void setTransTimeIdx(uint32_t idx);
    uint32_t getTransTimeIdx() const;
    void setAttIdx(uint32_t idx);
    uint32_t getAttIdx() const;
    void setRateIdx(uint32_t idx);
    uint32_t getRateIdx() const;
    void setPosIdx(uint32_t idx);
    uint32_t getPosIdx() const;
    void setVelIdx(uint32_t idx);
    uint32_t getVelIdx() const;
    void setDvIdx(uint32_t idx);
    uint32_t getDvIdx() const;
    void setSunIdx(uint32_t idx);
    uint32_t getSunIdx() const;
    void setAttMsgCount(uint32_t msgCount);
    uint32_t getAttMsgCount() const;
    void setTransMsgCount(uint32_t msgCount);
    uint32_t getTransMsgCount() const;

    AggregateAttInput attMsgs[MAX_AGG_NAV_MSG];     /*!< [-] The incoming nav message buffer */
    AggregateTransInput transMsgs[MAX_AGG_NAV_MSG]; /*!< [-] The incoming nav message buffer */
    Message<NavAttMsgPayload> navAttOutMsg;         /*!< blended attitude navigation output message */
    Message<NavTransMsgPayload> navTransOutMsg;     /*!< blended translation navigation output message */

   private:
    NavAggregateAlgorithm algorithm{};
};

#endif
