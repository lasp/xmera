// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#ifndef _SIGNAL_CONDITION_H_
#define _SIGNAL_CONDITION_H_

/*! struct definition */
typedef struct {
    double hStep;        /*!< [s]      filter time step (assumed to be fixed) */
    double omegCutoff;   /*!< [rad/s]  Cutoff frequency for the filter        */
    double currentState; /*!< [-] Current state of the filter                 */
    double currentMeas;  /*!< [-] Current measurement that we read            */
} LowPassFilterData;

#ifdef __cplusplus
extern "C" {
#endif

void lowPassFilterSignal(double newMeas, LowPassFilterData* lpData);

#ifdef __cplusplus
}
#endif

#endif
