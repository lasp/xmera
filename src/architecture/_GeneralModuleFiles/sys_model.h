// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sys_model
#define XMAheader_sys_model

#include <architecture/utilities/bskLogging.h>
#include <stdint.h>
#include <string>

//! The atomic unit of behavior in a χmera simulation
/*!
 *  An instance of `SysModel` (a "module") plays two distinct roles: it acts as
 *  a *configuration interface* for simulation modelers, and it acts as a *behavioral
 *  component* for the χmera simulation system. The `SysModel` class itself describes
 *  the structure required in the latter context. An implementor of the class
 *  is responsible for satisfying any model configuration needs.
 *
 *  `SysModel` instances are allowed (and, largely, expected) to communicate
 *  amongst themselves at runtime. For this purpose, χmera includes a "messaging"
 *  subsystem. See the `Message` class for details. However, note that modules
 *  may use any other agreed-upon mechanism for inter-module communication; the
 *  χmera messaging subsystem is not mandatory.
 *
 *  To be included in a simulation, a module must be attached to an instance of
 *  `SysModelTask` (a "task"). This task will dictate the schedule on which a
 *  module's state is advanced.
 *
 *
 *  ## Lifecycle of a module
 *
 *  Implementors and users of a `SysModel` must abide by the following lifecycle
 *  contract.
 *
 *  - During construction, an instance must not assume that a containing simulation
 *    exists yet. The instance should put itself into a known state, dictated
 *    by its construction parameters (if any), from which it may subsequently
 *    be configured. This state need not be simulation-ready.
 *
 *  - Any implementor-defined configuration methods may be invoked before simulation
 *    according to whatever configuration protocol is dictated by the module.
 *    Modules must not be (re)configured *during* simulation; thus, the only
 *    method that may be invoked immediately after configuration is `reset`.
 *
 *  - The `reset()` method places the module into an **initial state** dictated
 *    by its configuration. A successful return from this method indicates that
 *    the module is ready to participate in simulation. If the initial state is
 *    not simulation-ready, the instance must throw an `std::invalid_argument`
 *    exception. The module may also communicate with other modules (i.e. those
 *    it has been *configured* to know about) to determine its initial state.
 *
 *    The argument to `reset()` indicates the instant within a fresh simulation
 *    associated with the initial state of the module. The specific value of this
 *    instant must not be depended on; only the relative distance between the
 *    reset instant and any provided via `updateState()` may bear on the module's
 *    behavior. Thus, assuming the same configuration, every call to `reset()`
 *    must always place the module back in the same initial state.
 *
 *  - The `updateState()` method is called after `reset()` has been called at
 *    least once. As with `reset`, during `updateState`, a module may (and is
 *    largely expected to) communicate with other modules to determine its new
 *    state.
 *
 *    The argument to `updateState()` indicates the instant within the active
 *    simulation associated with the state of the module on return. This instant
 *    must be no earlier than that provided to most recent `updateState()` or
 *    `reset()` call. Thus, within a single simulation, the arguments to consecutive
 *    `updateState()` calls will be monotonically increasing.
 */
class SysModel {
public:
    //! Construct a new module instance with a fresh `moduleId`
    SysModel();

    //! Create a copy of a module instance with a fresh `moduleId`
    SysModel(SysModel const &other);

    virtual ~SysModel() = default;

public:
    //! Reset the module to its initial state determined by its configuration.
    /*!
     *  The initial state of a module must be determined solely by its configuration
     *  at the time that `reset()` is called. The specific value of `initialSimNanos`
     *  must not be depended on; instead, the *relative* difference between it
     *  and a subsequent invocation of `updateState()` determines the latter's
     *  behavior.
     *
     *  @param[in] initialSimNanos
     *    The simulation instant associated with the module's initial state.
     */
    virtual void reset(uint64_t initialSimNanos) {}

    //! Advance the module's state to a given simulation time.
    /*!
     *  Advance the module to the state identified by `nextSimNanos`, dependent
     *  on the module's current state and the difference between `nextSimNanos`
     *  and the instant associated with the current state.
     *
     *  @param[in] nextSimNanos
     *    The simulation instant associated with the desired module state.
     */
    virtual void updateState(uint64_t nextSimNanos) {}

public:
    //! A configurable, human-readable tag
    std::string modelTag = "";

    //! A configurable seed to drive any random number generation needed by the module
    uint32_t RNGSeed = 0x1bad'cad1;

    //! A number uniquely identifying this module among all modules in the (system) process
    int64_t const moduleID;
};

#endif
