#ifndef MISSION_PARAMETERS_H
#define MISSION_PARAMETERS_H

// SWIG parses this header to expose the constants to Python, and its
// preprocessor reads the quote in a digit separator such as 5'000 as the start
// of a character literal, silently dropping the constants that follow. Turn
// clang-format off so IntegerLiteralSeparator does not reintroduce them.

// clang-format off
#define MAX_KEY_POINTS 5000
#define MAX_NUM_CSS_SENSORS 32
#define MAX_EFF_CNT 36
#define RW_EFF_CNT 36
#define MAX_N_CSS_MEAS 32

#define MAX_SICP_POINTS 5000
#define SICP_POINT_DIM 3
#define MAX_SICP_ITERATIONS 250

#define MAX_NUMBER_REGIONS 3
// clang-format on

#endif  // MISSION_PARAMETERS_H
