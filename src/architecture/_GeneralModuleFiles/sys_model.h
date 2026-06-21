// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sys_model
#define XMAheader_sys_model

#include <architecture/utilities/bskLogging.h>
#include <stdint.h>
#include <string>

/*! @brief Simulation System Model Class */
class SysModel {
   public:
    SysModel();
    SysModel(const SysModel& obj);

    virtual ~SysModel() {}

    /** Initializes the module, create messages */
    virtual void selfInit() {}

    /** Reads incoming messages, performs module actions, writes output messages */
    virtual void updateState(uint64_t currentSimNanos) {}

    /** Called at simulation initialization, resets module to specified time */
    virtual void reset(uint64_t currentSimNanos) {}

    std::string modelTag = "";      //!< -- name for the algorithm to base off of
    uint32_t RNGSeed = 0x1badcad1;  //!< -- Giving everyone a random seed for ease of MC
    int64_t moduleID;               //!< -- Dynamically generated unique ID for this module
};

#endif
