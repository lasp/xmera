#ifndef inertialHeadingSimMsg_h
#define inertialHeadingSimMsg_h

//!@brief Inertial heading message definition.
/*! This message contains the unit direction vector of a certain direction
  expressed in inertial frame coordinates.
 */
typedef struct {
    double rHat_XN_N[3];  //!< [] unit heading vector to any thing "X" in space, "N", inertial frame
} InertialHeadingMsgPayload;

#endif /* inertialHeadingSimMsg_h */
