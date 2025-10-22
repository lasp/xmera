/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

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

#ifndef _OE_STATE_EPHEM_ALGORITHM_H_
#define _OE_STATE_EPHEM_ALGORITHM_H_

#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/TDBVehicleClockCorrelationMsgPayload.h>
#include <architecture/utilities/orbitalMotion.hpp>
#include <fswAlgorithms/transDetermination/_GeneralModuleFiles/ephemerisUtilities.h>
#include <array>

#define MAX_OE_RECORDS 10
#define MAX_OE_COEFF 20

/*! @brief Structure that defines the layout of an Ephemeris "record."  This is
           basically the set of coefficients for the ephemeris elements and
           the time factors associated with those coefficients
*/
class ChebyshevFitArc {
   public:
    unsigned int numberChebCoefficients{};  //!< [-] Number chebyshev coefficients loaded into record
    double ephemerisTimeMiddle{};           //!< [s] Ephemeris time (TDB) associated with the mid-point of the curve
    double ephemerisTimeRadius{};           //!< [s] "Radius" of time that curve is valid for (half of total range)
    std::array<double, MAX_OE_COEFF>
        radiusPeriapsisCoefficients{};  //!< [-] Set of chebyshev coefficients for radius at periapses
    std::array<double, MAX_OE_COEFF>
        eccentricityCoefficients{};                              //!< [-] Set of chebyshev coefficients for eccentricity
    std::array<double, MAX_OE_COEFF> inclinationCoefficients{};  //!< [-] Set of chebyshev coefficients for inclination
    std::array<double, MAX_OE_COEFF>
        argPeriapsisCoefficients{};  //!< [-] Set of chebyshev coefficients for argument of periapses
    std::array<double, MAX_OE_COEFF>
        raanCoefficients{};  //!< [-] Set of chebyshev coefficients for right ascention of the ascending node
    std::array<double, MAX_OE_COEFF>
        trueAnomalyCoefficients{};  //!< [-] Set of chebyshev coefficients for true anomaly angle
    unsigned int anomalyFlag{};     //!< [-] Flag indicating if the anomaly angle is true (0), mean (1)
};

/*! @brief Top level structure for the Chebyshev position ephemeris
           fit system.  Allows the user to specify a set of chebyshev
           coefficients and then use the input time to determine where
           a given body is in space
*/
class OEStateEphemAlgorithm {
   public:
    void reset(uint64_t callTime, const TDBVehicleClockCorrelationMsgPayload& vehicleTimePayload);
    EphemerisMsgPayload updateState(uint64_t callTime);

    void setCentralBodyGravitationalParameter(double gravitationalParameter);
    double getCentralBodyGravitationalParameter() const;

    void setArcNumberOfCoefficients(unsigned int arcNumber, unsigned int numberOfCoefficients);
    unsigned int getArcNumberOfCoefficients(unsigned int arcNumber) const;

    void setArcMiddleTime(unsigned int arcNumber, double timeMiddle);
    double getArcMiddleTime(unsigned int arcNumber) const;

    void setArcRadiusTime(unsigned int arcNumber, double timeRadius);
    double getArcRadiusTime(unsigned int arcNumber) const;

    void setArcAnomalyFlag(unsigned int arcNumber, unsigned int anomalyFlag);
    unsigned int getArcAnomalyFlag(unsigned int arcNumber) const;

    void setArcRadiusPeriapsisCoefficients(const unsigned int arcNumber,
                                           const std::array<double, MAX_OE_COEFF>& radiusPeriapsisCoefficients);
    std::array<double, MAX_OE_COEFF> getArcRadiusPeriapsisCoefficients(const unsigned int arcNumber);
    void setArcEccentricityCoefficients(const unsigned int arcNumber,
                                        const std::array<double, MAX_OE_COEFF>& eccentricityCoefficients);
    std::array<double, MAX_OE_COEFF> getArcEccentricityCoefficients(const unsigned int arcNumber);
    void setArcInclinationCoefficients(const unsigned int arcNumber,
                                       const std::array<double, MAX_OE_COEFF>& inclinationCoefficients);
    std::array<double, MAX_OE_COEFF> getArcInclinationCoefficients(const unsigned int arcNumber);
    void setArcArgPeriapsisCoefficients(const unsigned int arcNumber,
                                        const std::array<double, MAX_OE_COEFF>& argPeriapsisCoefficients);
    std::array<double, MAX_OE_COEFF> getArcArgPeriapsisCoefficients(const unsigned int arcNumber);
    void setArcRaanCoefficients(const unsigned int arcNumber, const std::array<double, MAX_OE_COEFF>& raanCoefficients);
    std::array<double, MAX_OE_COEFF> getArcRaanCoefficients(const unsigned int arcNumber);
    void setArcTrueAnomalyCoefficients(const unsigned int arcNumber,
                                       const std::array<double, MAX_OE_COEFF>& trueAnomalyCoefficients);
    std::array<double, MAX_OE_COEFF> getArcTrueAnomalyCoefficients(const unsigned int arcNumber);

   private:
    ChebyshevFitArc findCurrentArc(uint64_t callTime, const TDBVehicleClockCorrelationMsgPayload& localTime);
    double scaleEphemerisTime(const ChebyshevFitArc& arc) const;
    static ClassicalElements evaluateCoefficients(const double currentScaledValue, const ChebyshevFitArc& arc);
    double currentEphTime{};
    double gravitationalParameter{};  //!< [m3/s^2] Gravitational parameter for center of orbital elements
    std::array<ChebyshevFitArc, MAX_OE_RECORDS> fitCoefficients{};  //!< [-] Array of Chebyshev records for ephemeris
    TDBVehicleClockCorrelationMsgPayload spacecraftTime{};
};

#endif
