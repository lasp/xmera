#include "architecture/utilities/signalCondition.h"


/*! This method applies the low-pass filter configuration to the newMeas that
    is passed in.  The state is maintained in the LowPassFilterData structure
 @return void
 @param lpData The configuration data and history of the LP filter
 @param newMeas The new measurement to take in to the filter
 */
void lowPassFilterSignal(double newMeas, LowPassFilterData *lpData)
{
    /*! See documentation of algorithm in documentation for LP torque filter module*/
    double hOmeg = lpData->hStep*lpData->omegCutoff;
    lpData->currentState = (1.0/(2.0+hOmeg)*
        (lpData->currentState*(2.0-hOmeg)+hOmeg*(newMeas+lpData->currentMeas)));
    lpData->currentMeas = newMeas;
    return;
}
