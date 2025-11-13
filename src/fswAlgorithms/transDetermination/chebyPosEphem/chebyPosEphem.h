#ifndef _CHEBY_POS_EPHEM_H_
#define _CHEBY_POS_EPHEM_H_

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/TDBVehicleClockCorrelationMsgPayload.h>

#include <architecture/utilities/bskLogging.h>

#define MAX_CHEB_COEFF 40
#define MAX_CHEB_RECORDS 4

/*! @brief Structure that defines the layout of an Ephemeris "record."  This is
           basically the set of coefficients for the body x/y/z positions and
           the time factors associated with those coefficients
*/
typedef struct {
    uint32_t nChebCoeff;                      /*!< [-] Number chebyshev coefficients loaded into record*/
    double ephemTimeMid;                      /*!< [s] Ephemeris time (TDB) associated with the mid-point of the curve*/
    double ephemTimeRad;                      /*!< [s] "Radius" of time that curve is valid for (half of total range*/
    double posChebyCoeff[3 * MAX_CHEB_COEFF]; /*!< [-] Set of chebyshev coefficients for position */
    double velChebyCoeff[3 * MAX_CHEB_COEFF]; /*!< [-] Set of coefficients for the velocity estimate*/
} ChebyEphemRecord;

/*! @brief Top level structure for the Chebyshev position ephemeris
           fit system. e
*/
class ChebyPosEphem : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    Message<EphemerisMsgPayload> posFitOutMsg;                        /*!< [-] output navigation message for pos/vel*/
    ReadFunctor<TDBVehicleClockCorrelationMsgPayload> clockCorrInMsg; /*!< clock correlation input message*/
    ChebyEphemRecord ephArray[MAX_CHEB_RECORDS]; /*!< [-] Array of Chebyshev records for ephemeris*/

    uint32_t coeffSelector; /*!< [-] Index in the ephArray that we are currently using*/

    EphemerisMsgPayload outputState; /*!< [-] The local storage of the outgoing message data*/

    BSKLogger bskLogger{};  //!< BSK Logging
};

#endif
