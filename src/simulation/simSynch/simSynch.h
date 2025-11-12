#ifndef CLOCK_SYNCH_H
#define CLOCK_SYNCH_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <chrono>
#include <string>
#include <vector>

#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/SynchClockMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief clock sync model class */
class ClockSynch : public SysModel {
   public:
    ClockSynch();
    ~ClockSynch();

    void reset(uint64_t currentSimNanos);
    void updateState(uint64_t currentSimNanos);

   public:
    double accelFactor;                         //!< [-] Factor used to accelerate sim-time relative to clock
    SynchClockMsgPayload outputData;            //!< [-] Output data for the synch module
    Message<SynchClockMsgPayload> clockOutMsg;  //!< [-] output message

    int64_t accuracyNanos;  //!< ns Level of accuracy that we want out of the timer, default is 10ms
    bool displayTime;     //!< [-] Flag indicating that we want to display the time elapsed in cmd line, default is off
    BSKLogger bskLogger;  //!< -- BSK Logging
   private:
    bool timeInitialized;                                      //!< [-] Flag that the module has been reset
    std::chrono::high_resolution_clock::time_point startTime;  //! [-] first time stamp of pass through data
    uint64_t startSimTimeNano;                                 //!< [ns] Previous simulation time observed
};

#endif
