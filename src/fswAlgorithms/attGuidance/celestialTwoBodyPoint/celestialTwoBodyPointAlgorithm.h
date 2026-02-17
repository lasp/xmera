// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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
