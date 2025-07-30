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

#ifndef NAV_AGGREGATE_ALGORITHM_H
#define NAV_AGGREGATE_ALGORITHM_H

#include <stdint.h>
#include <stdexcept>

#include <array>

#include "architecture/msgPayloadDef/NavAttMsgPayload.h"
#include "architecture/msgPayloadDef/NavTransMsgPayload.h"

#define MAX_AGG_NAV_MSG 10

/*! structure containing the attitude and translational navigation out messages */
typedef struct {
    NavAttMsgPayload navAttOut; /*!< attitude navigation out message payload */
    NavTransMsgPayload navTransOut; /*!< translation navigation out message payload */
}AggregateOutput;

class NavAggregateAlgorithm {
   public:
    AggregateOutput update(std::array<NavAttMsgPayload, MAX_AGG_NAV_MSG> attMsgsPayloads,
                           std::array<NavTransMsgPayload, MAX_AGG_NAV_MSG> transMsgsPayloads);
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

   private:
    uint32_t attTimeIdx{};        /*!< [-] The index of the message to use for attitude message time */
    uint32_t transTimeIdx{};      /*!< [-] The index of the message to use for translation message time */
    uint32_t attIdx{};        /*!< [-] The index of the message to use for inertial MRP*/
    uint32_t rateIdx{};       /*!< [-] The index of the message to use for attitude rate*/
    uint32_t posIdx{};        /*!< [-] The index of the message to use for inertial position*/
    uint32_t velIdx{};        /*!< [-] The index of the message to use for inertial velocity*/
    uint32_t dvIdx{};         /*!< [-] The index of the message to use for accumulated DV */
    uint32_t sunIdx{};        /*!< [-] The index of the message to use for sun pointing*/
    uint32_t attMsgCount{};   /*!< [-] The total number of messages available as inputs */
    uint32_t transMsgCount{}; /*!< [-] The total number of messages available as inputs */
};

#endif
