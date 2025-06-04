/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef _OE_STATE_EPHEM_H_
#define _OE_STATE_EPHEM_H_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/TDBVehicleClockCorrelationMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "fswAlgorithms/transDetermination/_GeneralModuleFiles/ephemerisUtilities.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/orbitalMotion.h"
#include <cmath>
#include <assert.h>
#include <array>

#define MAX_OE_RECORDS 10
#define MAX_OE_COEFF 20

/*! @brief Structure that defines the layout of an Ephemeris "record."  This is
           basically the set of coefficients for the ephemeris elements and
           the time factors associated with those coefficients
*/
class ChebyshevFitArc{
public:
    signed int numberChebCoefficients{};                  //!< [-] Number chebyshev coefficients loaded into record
    double ephemerisTimeMiddle{};                  //!< [s] Ephemeris time (TDB) associated with the mid-point of the curve
    double ephemerisTimeRadius{};                  //!< [s] "Radius" of time that curve is valid for (half of total range)
    std::array<double, MAX_OE_COEFF> radiusPeriapsisCoefficients{};    //!< [-] Set of chebyshev coefficients for radius at periapses
    std::array<double, MAX_OE_COEFF> eccentricityCoefficients{};        //!< [-] Set of chebyshev coefficients for eccentricity
    std::array<double, MAX_OE_COEFF> inclinationCoefficients{};        //!< [-] Set of chebyshev coefficients for inclination
    std::array<double, MAX_OE_COEFF> argPeriapsisCoefficients{};     //!< [-] Set of chebyshev coefficients for argument of periapses
    std::array<double, MAX_OE_COEFF> raanCoefficients{};       //!< [-] Set of chebyshev coefficients for right ascention of the ascending node
    std::array<double, MAX_OE_COEFF> trueAnomalyCoefficients{};       //!< [-] Set of chebyshev coefficients for true anomaly angle
    signed int anomalyFlag{};                 //!< [-] Flag indicating if the anomaly angle is true (0), mean (1)
};

/*! @brief Top level structure for the Chebyshev position ephemeris
           fit system.  Allows the user to specify a set of chebyshev
           coefficients and then use the input time to determine where
           a given body is in space
*/
class OEStateEphem : public SysModel {
public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    Message<EphemerisMsgPayload> stateFitOutMsg; //!< [-] output navigation message for pos/vel
    ReadFunctor<TDBVehicleClockCorrelationMsgPayload> clockCorrInMsg; //!< clock correlation input message

    void setCentralBodyGravitationalParameter(double gravitationalParameter);
    double getCentralBodyGravitationalParameter() const;

    void setArcNumberOfCoefficients(signed int arcNumber, signed int numberOfCoefficients);
    signed int getArcNumberOfCoefficients(signed int arcNumber) const;

    void setArcMiddleTime(signed int arcNumber, double timeMiddle);
    double getArcMiddleTime(signed int arcNumber) const;

    void setArcRadiusTime(signed int arcNumber, double timeRadius);
    double getArcRadiusTime(signed int arcNumber) const;

    void setArcAnomalyFlag(signed int arcNumber, signed int anomalyFlag);
    signed int getArcAnomalyFlag(signed int arcNumber) const;

    void setArcRadiusPeriapsisCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &radiusPeriapsisCoefficients);
    std::array<double, MAX_OE_COEFF> getArcRadiusPeriapsisCoefficients(const signed int arcNumber);
    void setArcEccentricityCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &eccentricityCoefficients);
    std::array<double, MAX_OE_COEFF> getArcEccentricityCoefficients(const signed int arcNumber);
    void setArcInclinationCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &inclinationCoefficients);
    std::array<double, MAX_OE_COEFF> getArcInclinationCoefficients(const signed int arcNumber);
    void setArcArgPeriapsisCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &argPeriapsisCoefficients);
    std::array<double, MAX_OE_COEFF> getArcArgPeriapsisCoefficients(const signed int arcNumber);
    void setArcRaanCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &raanCoefficients);
    std::array<double, MAX_OE_COEFF> getArcRaanCoefficients(const signed int arcNumber);
    void setArcTrueAnomalyCoefficients(const signed int arcNumber, const std::array<double, MAX_OE_COEFF> &trueAnomalyCoefficients);
    std::array<double, MAX_OE_COEFF> getArcTrueAnomalyCoefficients(const signed int arcNumber);

private:
    double gravitationalParameter{};                             //!< [m3/s^2] Gravitational parameter for center of orbital elements
    std::array<ChebyshevFitCoefficients, MAX_OE_RECORDS> fitCoefficients{};       //!< [-] Array of Chebyshev records for ephemeris
};

#endif
