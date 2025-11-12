#ifndef hingedRigidBodySimMsg_h
#define hingedRigidBodySimMsg_h

/*! @brief Structure used to define the individual Hinged Rigid Body  data message*/
typedef struct {
    double theta;     //!< [rad], panel angular displacement
    double thetaDot;  //!< [rad/s], panel angular displacement rate
} HingedRigidBodyMsgPayload;

#endif /* hingedRigidBodySimMsg_h */
