#ifndef linearTranslationRigidBodySimMsg_h
#define linearTranslationRigidBodySimMsg_h

/*! @brief Structure used to define the translating rigid body data message*/
typedef struct {
    double rho;     //!< [m], body linear displacement
    double rhoDot;  //!< [m/s], body linear displacement rate
} LinearTranslationRigidBodyMsgPayload;

#endif /* linearTranslationRigidBodySimMsg_h */
