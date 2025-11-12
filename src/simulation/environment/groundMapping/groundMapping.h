#ifndef GROUNDMAPPING_H
#define GROUNDMAPPING_H

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AccessMsgPayload.h>
#include <architecture/msgPayloadDef/GroundStateMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>
#include <architecture/utilities/bskLogging.h>

#include <architecture/utilities/astroConstants.h>
#include <architecture/utilities/geodeticConversion.h>

/*! @brief This module checks that a vector of mapping points are visible to a spacecraft's imager, outputting a vector
 * of accessMessages for each mapping point
 */
class GroundMapping : public SysModel {
   public:
    GroundMapping();
    ~GroundMapping();

    void addPointToModel(Eigen::Vector3d& r_LP_P_init);
    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

   private:
    void ReadMessages();
    void computeAccess(uint64_t c);
    void WriteMessages(uint64_t CurrentClock);
    void updateInertialPositions();
    uint64_t checkInstrumentFOV();

   public:
    double minimumElevation;      //!< [rad] (optional) minimum elevation above the local horizon needed to see a
                                  //!< spacecraft; defaults to 10 degrees equivalent.
    double maximumRange;          //!< [m] (optional) Maximum slant range to compute access for; defaults to -1, which
                                  //!< represents no maximum range.
    Eigen::Vector3d cameraPos_B;  //!< [m] (optional) Instrument position in body frame, defaults to (0,0,0)
    double halfFieldOfView;       //!< [r] Instrument half-fov, defaults to 10 degrees
    Eigen::Vector3d nHat_B;       //!< [-] Instrument unit direction vector in body frame components

    BSKLogger bskLogger;  //!< -- BSK Logging

    ReadFunctor<SpicePlanetStateMsgPayload> planetInMsg;  //!< Planet state input message
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;         //!< Spacecraft state input message
    std::vector<Message<GroundStateMsgPayload>*>
        currentGroundStateOutMsgs;                          //!< vector of ground location output message
    std::vector<Message<AccessMsgPayload>*> accessOutMsgs;  //!< vector of ground location access messages

   private:
    std::vector<AccessMsgPayload> accessMsgBuffer;                   //!< buffer of access output data
    std::vector<GroundStateMsgPayload> currentGroundStateMsgBuffer;  //!< buffer of access output data
    SCStatesMsgPayload scStateInMsgBuffer;                           //!< buffer of spacecraft states
    SpicePlanetStateMsgPayload planetInMsgBuffer;                    //!< buffer of planet data

    std::vector<Eigen::Vector3d> mappingPoints;  //!< Vector of mapping points
    Eigen::Matrix3d dcm_LP;  //!< Rotation matrix from planet-centered, planet-fixed frame P to site-local topographic
                             //!< (SEZ) frame L coordinates.
    Eigen::Matrix3d dcm_PN;  //!< Rotation matrix from inertial frame N to planet-centered to planet-fixed frame P.
    Eigen::Matrix3d
        dcm_PN_dot;  //!< Rotation matrix derivative from inertial frame N to planet-centered to planet-fixed frame P.
    Eigen::Vector3d w_PN;    //!< [rad/s] Angular velocity of planet-fixed frame P relative to inertial frame N.
    Eigen::Vector3d r_PN_N;  //!< [m] Planet position vector relative to inertial frame origin.
    Eigen::Vector3d r_LP_P;  //!< [m] Ground Location position vector relative to to planet origin vector in planet
                             //!< frame coordinates.
    Eigen::Vector3d
        r_LP_N;  //!< [m] Gound Location position vector relative to planet origin vector in inertial coordinates.
    Eigen::Vector3d rhat_LP_N;  //!< [-] Surface normal vector from the target location in inertial coordinates.
    Eigen::Vector3d
        r_LN_N;  //!< [m] Ground Location position vector relative to inertial frame origin in inertial coordinates.
    Eigen::Vector3d r_North_N;  //!<[-] Inertial 3rd axis, defined internally as "North".
    Eigen::Vector3d r_BP_N;     //!< [m] Inertial position of the body frame wrt the planet
    Eigen::Matrix3d dcm_NB;     //!< DCM from the body frame to the inertial frame
};

#endif
