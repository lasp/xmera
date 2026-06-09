// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_ALGORITHMS_SUNLINE_SR_UKF_ALGORITHM_H
#define FILTERING_ALGORITHMS_SUNLINE_SR_UKF_ALGORITHM_H

#include "sunlineSRuKFSpecs.h"

#include <filteringCore/kalmanFilter.hpp>
#include <filteringCore/measurementQueue.h>
#include <filteringCore/srukf.hpp>

#include <Eigen/Core>

namespace filtering::sunlineSRuKF {

/*! Raw CSS array reading consumed by SunlineSRuKFAlgorithm::update(). The
 *  first `numberOfCss` slots of `cosValues` are meaningful; `valid` flags
 *  whether the adapter saw a fresh reading this cycle. */
struct CssData {
    double                        timeTag   = 0;
    Eigen::Vector<double, MaxCss> cosValues = Eigen::Vector<double, MaxCss>::Zero();
};

/*! Raw gyro reading consumed by SunlineSRuKFAlgorithm::update(). `valid`
 *  flags whether the adapter saw a fresh reading this cycle. */
struct RateData {
    double          timeTag = 0;
    Eigen::Vector3d rate    = Eigen::Vector3d::Zero();
};

/*! Snapshot returned by SunlineSRuKFAlgorithm::update(). Bundles the filter
 *  state with the per-kind residuals so the host adapter writes all output
 *  messages from one consistent post-update snapshot. */
struct SunlineSRuKFOutput {
    FilterStateOutput   filterState;
    CssResidualsOutput  cssResiduals;
    RateResidualsOutput rateResiduals;
};

/*! @brief Sunline square-root UKF. Estimates sun-heading direction, body rate,
 *  and a CSS bias from CSS array measurements and gyro rates on one timeline. */
class SunlineSRuKFAlgorithm {
   public:
    using State = SunlineState;
    static constexpr int N = State::size;

    SunlineSRuKFOutput update(double currentSeconds,
                              CssData const& cssData,
                              RateData const& rateData);

    void reset();
    void timeUpdate(double dt);
    void measurementUpdate(Measurement const& measurement);

    FilterStateOutput           getFilterOutput() const;
    CssResidualsOutput const&   getLastCssResiduals() const;
    RateResidualsOutput const&  getLastRateResiduals() const;
    State                       getState() const;
    Eigen::Matrix<double, N, N> getCovariance() const;

    void                        setProcessNoise(Eigen::Matrix<double, N, N> const& newProcessNoise);
    Eigen::Matrix<double, N, N> getProcessNoise() const;
    void                        setAlpha(double newAlpha);
    double                      getAlpha() const;
    void                        setBeta(double newBeta);
    double                      getBeta() const;
    void                        setInitialState(State const& newInitialState);
    State                       getInitialState() const;
    void                        setInitialCovariance(Eigen::Matrix<double, N, N> const& newInitialCovariance);
    Eigen::Matrix<double, N, N> getInitialCovariance() const;
    void                        setBiasLowerBound(double lowerBound);
    double                      getBiasLowerBound() const;
    void                        setBiasUpperBound(double upperBound);
    double                      getBiasUpperBound() const;

    void                             setCssNHat(Eigen::Matrix<double, MaxCss, 3> const& nHat);
    Eigen::Matrix<double, MaxCss, 3> getCssNHat() const;
    void                             setCssCBias(Eigen::Vector<double, MaxCss> const& cBias);
    Eigen::Vector<double, MaxCss>    getCssCBias() const;
    void                             setNumberOfCss(int count);
    int                              getNumberOfCss() const;
    void                             setSensorThreshold(double threshold);
    double                           getSensorThreshold() const;
    void                             setCssMeasurementNoiseStd(double noiseStd);
    double                           getCssMeasurementNoiseStd() const;
    void                             setGyroMeasurementNoiseStd(double noiseStd);
    double                           getGyroMeasurementNoiseStd() const;

   private:
    void applyMeasurement(CssMeasurement const& measurement);
    void applyMeasurement(RateMeasurement const& measurement);

    CssMeasurement  packCssMeasurement(CssData const& cssData);
    RateMeasurement packRateMeasurement(RateData const& rateData);

    State regularize(State const& state) const;

    SRuKF<State, SunlineDynamics>               srukf;
    filtering::measurement_queue<Measurement, BatchSize> measurements;

    double                      alpha             = 0;           //!< [-] sigma-point spread tunable
    double                      beta              = 0;           //!< [-] prior-knowledge tunable
    Eigen::Matrix<double, N, N> processNoise      = Eigen::Matrix<double, N, N>::Zero();      //!< [-] Q
    State                       initialState      = {};          //!< [-] seed for reset()
    Eigen::Matrix<double, N, N> initialCovariance = Eigen::Matrix<double, N, N>::Identity();  //!< [-] P0
    double                      biasLowerBound    = 0.5;         //!< [-] lower clamp on the bias state
    double                      biasUpperBound    = 1.5;         //!< [-] upper clamp on the bias state

    Eigen::Matrix<double, MaxCss, 3> cssNHat          = Eigen::Matrix<double, MaxCss, 3>::Zero();  //!< [-] per-sensor unit vectors (body)
    Eigen::Vector<double, MaxCss>    cssCBias         = Eigen::Vector<double, MaxCss>::Zero();     //!< [-] per-sensor calibration biases
    int                              numberOfCss      = 0;       //!< [-] configured CSS count (first N rows meaningful)
    double                           sensorUseThresh  = 0;       //!< [-] minimum cosValue to count a sensor as active
    double                           cssMeasNoiseStd  = 0;       //!< [-] CSS noise std (diag of CSS R)
    double                           gyroMeasNoiseStd = 0;       //!< [rad/s] gyro noise std (diag of rate R)

    CssResidualsOutput  lastCssResiduals{};   //!< latest CSS residuals; valid=true only on cycles a CSS measurement fired
    RateResidualsOutput lastRateResiduals{};  //!< latest rate residuals; valid=true only on cycles a rate measurement fired
};

}  // namespace filtering::sunlineSRuKF

#endif
