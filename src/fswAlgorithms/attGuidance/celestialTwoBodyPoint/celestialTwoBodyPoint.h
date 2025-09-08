/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _CELESTIAL_BODY_POINT_H_
#define _CELESTIAL_BODY_POINT_H_

#include <stdint.h>
#include <stdexcept>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/AttRefMsgPayload.h"
#include "architecture/msgPayloadDef/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDef/NavTransMsgPayload.h"
#include <Eigen/Core>

/*!@brief Data structure for module to compute the two-body celestial pointing navigation solution.
 */
class CelestialTwoBodyPoint : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void parseInputMessages();
    void computeCelestialTwoBodyPoint(uint64_t callTime);
    void setSingularityThresh(double thresh);
    double getSingularityThresh() const;

    Message<AttRefMsgPayload> attRefOutMsg;            //!< The name of the output message*/
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;     //!< The name of the celestial body message*/
    ReadFunctor<EphemerisMsgPayload> secCelBodyInMsg;  //!< The name of the secondary body to constrain point*/
    ReadFunctor<NavTransMsgPayload> transNavInMsg;     //!< The name of the incoming attitude command*/

   private:
    double singularityThresh;  //!< [rad] Threshold for when to fix constraint axis*/
    int secCelBodyIsLinked;  //!< flag to indicate if the optional 2nd celestial body message is linked
    Eigen::Vector3d R_P1B_N{};   //!< [m] planet 1 position vector relative to inertial frame, in N-frame components
    Eigen::Vector3d R_P2B_N{};   //!< [m] planet 2 position vector relative to inertial frame, in N-frame components
    Eigen::Vector3d v_P1B_N{};   //!< [m/s] planet 1 velocity vector relative to inertial frame, in N-frame components
    Eigen::Vector3d v_P2B_N{};   //!< [m/s] planet 2 velocity vector relative to inertial frame, in N-frame components
    Eigen::Vector3d a_P1B_N{};   //!< [m/s^2] planet 1 acceleration vector relative to inertial frame, in N-frame components
    Eigen::Vector3d a_P2B_N{};   //!< [m/s^2] planet 2 acceleration vector relative to inertial frame, in N-frame components
    AttRefMsgPayload attRefOut;  //!< (-) copy of output reference frame message
};

#endif
