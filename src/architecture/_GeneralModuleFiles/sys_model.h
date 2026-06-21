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

// The following code helps users who defined their own module classes
// to transition to using the SWIG file for sys_model instead of the header file.
// After a period of 12 months from 2023/09/15, this message can be removed.
#ifdef SWIG
%extend SysModel
{
    %pythoncode %{
        def logger(self, *args, **kwargs):
            raise TypeError(
                f"The 'logger' function is not supported for this type ('{type(self).__qualname__}'). "
                "To fix this, update the SWIG file for this module. Change "
                """'%include "sys_model.h"' to '%include "sys_model.i"'"""
            )
    %}
}
#endif

#endif
