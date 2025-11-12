#include "exponentialAtmosphere.h"
#include <architecture/utilities/linearAlgebra.h>

/*! The constructor method initializes the dipole parameters to zero, resuling in a zero magnetic field result by
 default.
 @return void
 */
ExponentialAtmosphere::ExponentialAtmosphere() {
    //! - Set the default atmospheric properties to yield a zero response
    this->baseDensity = 0.0;   // [T]
    this->scaleHeight = 1.0;   // [m]
    this->planetRadius = 0.0;  // [m]
    this->localTemp = 1.0;     // [K]

    return;
}

/*! Empty destructor method.
 @return void
 */
ExponentialAtmosphere::~ExponentialAtmosphere() { return; }

/*! This method is evaluates the centered dipole magnetic field model.
 @param msg magnetic field message structure
 @param currentTime current time (s)
 @return void
 */
void ExponentialAtmosphere::evaluateAtmosphereModel(AtmoPropsMsgPayload* msg, double currentTime) {
    msg->neutralDensity = this->baseDensity * exp(-(this->orbitAltitude) / this->scaleHeight);
    msg->localTemp = this->localTemp;

    return;
}
