#ifndef _EPHEM_DIFFERENCE_H_
#define _EPHEM_DIFFERENCE_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/utilities/bskLogging.h>
#include <array>
#include <cstdint>

inline constexpr int MAX_NUM_CHANGE_BODIES = 10;

/*! @brief Container with paired input/output message names and IDs */
typedef struct {
    ReadFunctor<EphemerisMsgPayload> ephInMsg;  //!< [-] Input name for the ephemeris message
    Message<EphemerisMsgPayload> ephOutMsg;     //!< [-] The name converted output message
} EphemChangeConfig;

/*! @brief Container holding ephemDifference module variables */
class EphemDifference : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    ReadFunctor<EphemerisMsgPayload> ephBaseInMsg;                        //!< base ephemeris input message name
    std::array<EphemChangeConfig, MAX_NUM_CHANGE_BODIES> changeBodies{};  //!< [-] The list of bodies to change out

    uint32_t ephBdyCount;  //!< [-] The number of ephemeris bodies we are changing

    BSKLogger bskLogger{};  //!< BSK Logging
};

#endif
