#ifndef FILTER_DEFINITIONS_H
#define FILTER_DEFINITIONS_H

/*! @brief Maximum number of measurements that can be expected in a filter between two filter updates.
 * ie, if the filter updates at a 1Hz rate, no more than N measurements can be individually processed every second.
 * Measurements can also be averaged into a single measurement if the measurement frequency is very high */
#define MAX_MEASUREMENT_NUMBER 5
#define MAX_MEASUREMENT_VECTOR 5
#define MAX_STATES_VECTOR 9

#endif
