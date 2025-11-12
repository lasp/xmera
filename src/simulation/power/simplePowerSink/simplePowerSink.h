#ifndef XMERA_SIMPLEPOWERSINK_H
#define XMERA_SIMPLEPOWERSINK_H

#include <simulation/power/_GeneralModuleFiles/powerNodeBase.h>

/*! @brief simple power sink class */
class SimplePowerSink : public PowerNodeBase {
   public:
    SimplePowerSink();
    ~SimplePowerSink();

   private:
    void evaluatePowerModel(PowerNodeUsageMsgPayload* powerUsageMsg);
};

#endif  // XMERA_SIMPLEPOWERSINK_H
