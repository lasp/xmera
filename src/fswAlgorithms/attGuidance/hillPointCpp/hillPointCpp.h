#ifndef _HILL_POINT_CPP_H_
#define _HILL_POINT_CPP_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <Eigen/Core>

/*! @brief Hill Point attitude guidance class. */
class HillPointCpp : public SysModel {
   public:
    HillPointCpp() = default;            //!< Constructor
    ~HillPointCpp() override = default;  //!< Destructor

    void reset(uint64_t currentSimNanos) override;        //!< Reset function
    void updateState(uint64_t currentSimNanos) override;  //!< Update function

    ReadFunctor<NavTransMsgPayload> transNavInMsg;  //!< The name of the incoming attitude command
    ReadFunctor<EphemerisMsgPayload> celBodyInMsg;  //!< The name of the celestial body message
    Message<AttRefMsgPayload> attRefOutMsg;         //!< The name of the output message

    BSKLogger* bskLogger;  //!< BSK Logging

   private:
    int planetMsgIsLinked;  //!< flag if the planet message is linked

    static void computeHillPointingReference(Eigen::Vector3d r_BN_N,
                                             Eigen::Vector3d v_BN_N,
                                             Eigen::Vector3d celBdyPositionVector,
                                             Eigen::Vector3d celBdyVelocityVector,
                                             AttRefMsgPayload* attRefOut);
};

#endif
