#ifndef XMERA_SIMPLEBATTERY_H
#define XMERA_SIMPLEBATTERY_H

#include <simulation/power/_GeneralModuleFiles/powerStorageBase.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief simple battery class */
class SimpleBattery : public PowerStorageBase {
   public:
    SimpleBattery();
    ~SimpleBattery();

   private:
    void customreset(uint64_t CurrentClock);
    void evaluateBatteryModel(PowerStorageStatusMsgPayload* msg);

   public:
    double storageCapacity;  //!< [W-s] Battery capacity in Watt-seconds (Joules).
    BSKLogger bskLogger;     //!< -- BSK Logging
};

#endif  // XMERA_SIMPLEBATTERY_H
