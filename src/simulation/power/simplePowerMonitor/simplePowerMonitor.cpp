#include "simplePowerMonitor.h"


/*! The constructor creates a SimplePowerMonitor instance with zero stored charge and a capacity of -1.*/
SimplePowerMonitor::SimplePowerMonitor(){

    this->storedCharge = 0;
    return;
}

SimplePowerMonitor::~SimplePowerMonitor(){

    return;
}

/*! This method integrates the net power across all the attached devices and stores it.
 @return void
 */
void SimplePowerMonitor::evaluateBatteryModel(PowerStorageStatusMsgPayload *msg) {

    this->storedCharge = this->storedCharge + this->currentPowerSum * (this->currentTimestep);
    msg->storageCapacity = -1.0;
    msg->currentNetPower = this->currentPowerSum;
    msg->storageLevel = this->storedCharge;
    return;
}
