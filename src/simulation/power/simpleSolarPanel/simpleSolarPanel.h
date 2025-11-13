#ifndef XMERA_SIMPLESOLARPANEL_H
#define XMERA_SIMPLESOLARPANEL_H

#include <architecture/messaging/messaging.h>
#include <simulation/power/_GeneralModuleFiles/powerNodeBase.h>
#include <Eigen/Dense>
#include <vector>

#include <architecture/msgPayloadDef/EclipseMsgPayload.h>
#include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
#include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

/*! @brief simple solar panel class */
class SimpleSolarPanel : public PowerNodeBase {
   public:
    SimpleSolarPanel();
    ~SimpleSolarPanel();
    bool customReadMessages();
    void customreset(uint64_t CurrentClock);
    void setPanelParameters(Eigen::Vector3d nHat_B, double panelArea, double panelEfficiency);

   private:
    void evaluatePowerModel(PowerNodeUsageMsgPayload* powerUsageMsg);
    void computeSunData();

   public:
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;  //!< [-] sun data input message
    ReadFunctor<SCStatesMsgPayload> stateInMsg;        //!< [-] spacecraft state input message
    ReadFunctor<EclipseMsgPayload> sunEclipseInMsg;    //!< [-] Messun eclipse state input message
    double panelArea;                                  //!< [m^2] Panel area in meters squared.
    double panelEfficiency;  //!< [W/W] Panel efficiency in converting solar energy to electrical energy.
    Eigen::Vector3d nHat_B;  //!< [-] Panel normal unit vector relative to the spacecraft body frame.
    BSKLogger bskLogger;     //!< -- BSK Logging

   private:
    double projectedArea;      //!< [m^2] Area of the panel projected along the sun vector.
    double sunDistanceFactor;  //!< [-] Scale factor on the base solar power computed using the true s/c-sun distance.
    SpicePlanetStateMsgPayload sunData;  //!< [-] Unused for now, but including it for future
    SCStatesMsgPayload stateCurrent;     //!< [-] Current SSBI-relative state
    double shadowFactor;  //!< [-] solar eclipse shadow factor from 0 (fully obscured) to 1 (fully visible)
};

#endif  // XMERA_SIMPLESOLARPANEL_H
