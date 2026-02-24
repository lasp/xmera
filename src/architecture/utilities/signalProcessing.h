// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SIGNAL_PROCESSING_H_
#define _SIGNAL_PROCESSING_H_

#include <Eigen/Core>

class LowPassFilter {
   public:
    void setFilterStep(double filterStepSeconds);
    double getFilterStep() const;
    void setFilterCutoff(double cutOffValue);
    double getFilterCutoff() const;

    void processMeasurement(const Eigen::Vector3d& measurement);
    Eigen::Vector3d getCurrentState() const;

   private:
    double filterStep = 0.5;                                      /*!< [s] filter time step (assumed to be fixed) */
    double filterCutOff = 0.1 * 22 / 7 * 2;                       /*!< [rad/s]  Cutoff frequency for the filter */
    Eigen::Vector3d currentState = Eigen::Vector3d::Zero();       /*!< [-] Current state of the filter */
    Eigen::Vector3d currentMeasurement = Eigen::Vector3d::Zero(); /*!< [-] Current measurement that we read */
};

#endif
