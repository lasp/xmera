#ifndef WAYPOINTREFERENCE_H
#define WAYPOINTREFERENCE_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <fstream>
#include <iostream>

/*! @brief waypoint reference module class */
class WaypointReference : public SysModel {
   public:
    WaypointReference();
    ~WaypointReference();
    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

   public:
    std::string dataFileName;  //!< Name of the attitude waypoint data file
    std::string delimiter;     //!< delimiter string that separates data on a line, defaulted to comma ","
    int headerLines;           //!< Number of header lines in the file, defaulted to 0
    int attitudeType;  //!< 0 - MRP, 1 - EP or quaternions (q0, q1, q2, q3), 2 - EP or quaternions (q1, q2, q3, qs)
    bool useReferenceFrame;  //!< if true: angular rates and accelerations in the file are expressed in the reference
                             //!< frame; defaulted to false
    Message<AttRefMsgPayload> attRefOutMsg;  //!< attitude reference output msg

    BSKLogger bskLogger;  //!< -- BSK Logging

   private:
    std::ifstream* fileHandle;  //!< pointer to the file that is to be read
    bool endOfFile;             //!< boolean that indicates if the last line of file has been reached
    double pullScalar(std::istringstream* iss);
    void pullVector(std::istringstream* iss, double*);
    void pullVector4(std::istringstream* iss, double*);
    void pullDataLine(uint64_t* t, AttRefMsgPayload* attRefMsg_t);
    uint64_t t_a;                  //!< [ns] time t_a in the data file
    uint64_t t_b;                  //!< [ns] time t_b in the data file
    AttRefMsgPayload attRefMsg_a;  //!< attitude at time t_a
    AttRefMsgPayload attRefMsg_b;  //!< attitude at time t_b
    void linearInterpolation(uint64_t t_a, double v_a[3], uint64_t t_b, double v_b[3], uint64_t t, double* v);
};

#endif
