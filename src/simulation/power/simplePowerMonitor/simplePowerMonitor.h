#ifndef XMERA_SIMPLEPOWERMONITOR_H
#define XMERA_SIMPLEPOWERMONITOR_H

#include <simulation/power/_GeneralModuleFiles/powerStorageBase.h>

/*! @brief simple power monitor class */
class SimplePowerMonitor : public PowerStorageBase {
   public:
    SimplePowerMonitor();
    ~SimplePowerMonitor();

   private:
    void evaluateBatteryModel(PowerStorageStatusMsgPayload* msg);
};

#endif  // XMERA_SIMPLEPOWERMONITOR_H
