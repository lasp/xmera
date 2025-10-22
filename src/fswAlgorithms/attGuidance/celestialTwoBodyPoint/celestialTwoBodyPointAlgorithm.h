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

#ifndef CELESTIAL_BODY_POINT_ALGORITHM_H
#define CELESTIAL_BODY_POINT_ALGORITHM_H

#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <Eigen/Core>

/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
class CelestialTwoBodyPointAlgorithm {
   public:
    void reset(bool secCelBodyIsLinked);
    AttRefMsgPayload update(EphemerisMsgPayload& celBodyIn,
                            EphemerisMsgPayload& secCelBodyIn,
                            NavTransMsgPayload& transNavIn);
    void setSingularityThresh(double thresh);
    double getSingularityThresh() const;

   private:
    double singularityThresh;  //!< [rad] Threshold for when to fix constraint axis*/
    bool secCelBodyIsLinked;   //!< flag to indicate if the optional 2nd celestial body message is linked
};

#endif
