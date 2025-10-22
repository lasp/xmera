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

#ifndef SUN_SEARCH_ALGORITHM_H
#define SUN_SEARCH_ALGORITHM_H

#include <stdexcept>

#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <Eigen/Dense>

#define NUM_SLEWS 3

struct SlewProperties {
    double slewTime;       //!< [s] total time for the three-axes maneuver
    double slewAngle;      //!< [rad] total angle sweep around one axis
    double slewMaxRate;    //!< [rad/s] maximum spacecraft body rate norm
    double slewMaxTorque;  //!< [Nm] maximum torque for slew
    int slewRotAxis;       //!< [-] axes about which to perform the Sun search
};

struct KinematicProperties {
    int slewRotAxis;        //!< [-] axes about which to perform the Sun search
    double slewAngAcc;      //!< [rad/s^2] angular accelerations about each rotation axis
    double slewOmegaMax;    //!< [rad/s] highes angular rate about each rotation axis
    double slewThrustTime;  //!< [s] control time of each rotation
    double slewTotalTime;   //!< [s] total slew time of each rotation
};

struct ReferenceMotionOutput {
    Eigen::Vector3d omega_RN_B{Eigen::Vector3d::Zero()};  /*!< reference angular velocity */
    Eigen::Vector3d domega_RN_B{Eigen::Vector3d::Zero()}; /*!< reference angular acceleration */
};

class SunSearchAlgorithm {
   public:
    SunSearchAlgorithm() = default;
    ~SunSearchAlgorithm() = default;

    void reset(uint64_t currentSimNanos, VehicleConfigMsgPayload const& vehicleConfigIn);
    AttGuidMsgPayload update(uint64_t currentSimNanos, NavAttMsgPayload& navAttIn);
    void setSlewProperties(SlewProperties slewPropertiesInput);
    void modifySlewProperties(SlewProperties slewPropertiesInput, uint32_t index);
    SlewProperties getSlewProperties(uint32_t index) const;

   private:
    void computeKinematicProperties(uint32_t const index);
    ReferenceMotionOutput computeReferenceMotion(uint64_t const currentSimNanos, uint32_t const index);

    SlewProperties slewProperties[NUM_SLEWS];
    KinematicProperties kinematicProperties[NUM_SLEWS];
    uint32_t numberOfSlews{};             //!< [-] number of slew maneuvers set
    Eigen::Vector3d principleInertias{};  //!< [kg m^2] inertias about the three principal axes
    uint64_t resetTime;                   //!< time at which reset is called
};

#endif
