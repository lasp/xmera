#ifndef bodyHeadingSimMsg_h
#define bodyHeadingSimMsg_h

//!@brief Planet heading message definition.
/*! Many modules in Basilisk utilize the spacecraft body frame heading to something.
  For instance, a spacecraft may want to point at a point on earth, the sun, another planet
  or another spacecraft. This message is unique from the interface message NavAttMsgPayload
  in being agnostic of the thing being pointed to while not including separate information
  about attitude and rates that are not necessarily desired
 */
typedef struct {
    double rHat_XB_B[3];  //!< [] unit heading vector to any thing "X" in the spacecraft, "B", body frame
} BodyHeadingMsgPayload;

#endif /* bodyHeadingSimMsg_h */
