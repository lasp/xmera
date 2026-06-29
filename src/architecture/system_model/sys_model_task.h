// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sys_model_task
#define XMAheader_sys_model_task

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

#include <vector>

//! A module paired with its priority among modules within its containing task
struct ModelPriorityPair final {
    //! The priority of this module among others within its containing task
    int32_t CurrentModelPriority;

    //! A non-owning pointer to a module
    SysModel* ModelPtr;
};

//! An ordered sequence of modules to be updated on a periodic schedule
/*!
 *  An instance of `SysModelTask` (a "task") is a sequence of modules that should
 *  always be updated as a unit during simulation. Each module is inserted into
 *  the sequence by priority, with higher-priority modules being updated before
 *  lower-priority modules.
 *
 *  Every task also includes an update period, which dictates how frequently its
 *  modules will be updated, and an initial update time, which indicates the time
 *  at which the task should begin updating. The task will not update at any point
 *  prior to the first update time; however, once this time is reached, it will
 *  continue to update according to the update period.
 *
 *  A task may also be enabled or disabled. When disabled, the modules in this
 *  task will be inhibited from updating, though future updates will continue to
 *  occur on the same schedule. In other words, if a task is disabled and later
 *  re-enabled, the updates occurring afterwards will occur on schedule as if
 *  the task was never disabled, although updates while the task was disabled
 *  will not have occurred at all.
 *
 *  @todo \parblock
 *    This ought to be an opaque `SysModel`. It's doing two jobs: it represents
 *    a sequential composition of other `SysModel`s, and it manages its own update
 *    schedule. These jobs ought to be separated out, and arguably the latter
 *    should be the job of the containing `SysProcess`. Heck, `SysProcess` *already*
 *    keeps a copy of the `NextTaskStart` and `TaskUpdatePeriod` for each task.
 *
 *    On top of that, `resetModules()` and `reset()` are weirdly separated when
 *    neither should be invoked without the other also. If those can be combined,
 *    then `resetModules()` and `executeModules()` are exactly implementing
 *    `SysModel::reset()` and `SysModel::updateState()`.
 *  \endparblock
 */
class SysModelTask final {
public:
    //! Obtain an empty task with a given update period and start time
    /*!
     *  @param[in] updatePeriodNanos
     *    The interval of nanoseconds that should elapse between updates
     *  @param[in] firstUpdateNanos
     *    The time at which the task should be first updated
     */
    explicit SysModelTask(uint64_t updatePeriodNanos = 100, uint64_t firstUpdateNanos = 0)
        : nextUpdateNanos(firstUpdateNanos), updatePeriodNanos(updatePeriodNanos), firstUpdateNanos(firstUpdateNanos) {}

    //! Insert a module into the task's sequence of modules with a given priority
    /*!
     *  @param[in] module
     *    A non-owning pointer to the module to be added to the task
     *  @param[in] priority
     *    The priority of the given module (higher is earlier).
     */
    void addModel(SysModel* module, int32_t priority = -1);

    //! Update all modules in the task, in priority order
    /*!
     *  The argument to this method should always be equal to `getNextUpdateTime()`.
     *  On return, `getNextUpdateTime()` will indicate the next time at which this
     *  method should be called.
     *
     *  @param[in] nextSimNanos
     *    The simulation instant associated with the desired simulation state
     */
    void executeModels(uint64_t nextSimNanos);

    //! Reset all modules in the task, in priority order
    /*!
     *  @param[in] initialSimNanos
     *    The simulation instant associated with the initial simulation state
     */
    void resetModels(uint64_t initialSimNanos);

    //! Change the update period of the task
    /*!
     *  If the task has already been updated since its last reset, the next update
     *  time is recalculated against the new period. On return, the task will
     *  follow an update schedule as if its period were always `updatePeriodNanos`.
     *  The value of `getNextUpdateTime()` will be the greatest lower bound of
     *  its previous value on the new schedule; or if that wouldn't be later than
     *  the actual last time the module was updated, it will be the least upper
     *  bound instead.
     *
     *  This method may make the next time update earlier than it was before changing
     *  the period. Hence, it is safest to invoke this method at the same simulation
     *  time as (but causally after) an update occurs.
     *
     *  @important
     *    This method does not inform any containing `SysProcess` of the new
     *    next update time. If this task has been added to a running `SysProcess`,
     *    the `SysProcess::changeTaskPeriod()` method must be used instead.
     *
     *  @param[in] updatePeriodNanos
     *    The new interval of nanoseconds that should elapse between updates
     */
    void setPeriod(uint64_t updatePeriodNanos);

    //! Roll back the next update time to the *first* update time
    /*!
     *  This method must be used in conjunction with `resetModels()`, and must
     *  not be used while a simulation is still in progress.
     *
     *  @todo
     *    This method should be folded into `resetModels()`; it's silly that they're
     *    separate. Invoking one method without the other would decohere the
     *    simulation time understood by the task from the simulation time understood
     *    by its modules.
     */
    void reset() {
        this->nextUpdateNanos = this->firstUpdateNanos;
    }

    //! Get the next scheduled update time for this task
    /*!
     *  This will always be `getTaskPeriod()` plus some multiple of `getFirstTaskTime()`.
     */
    uint64_t getNextStartTime() const {
        return this->nextUpdateNanos;
    }

    //! Permit the task's modules to be updated at subsequent update times
    void enable() {
        this->taskActive = true;
    }

    //! Inhibit the task's modules from being updated at subsequent update times
    void disable() {
        this->taskActive = false;
    }

    //! Get the task's current update period in nanoseconds
    uint64_t getTaskPeriod() const {
        return this->updatePeriodNanos;
    }

    //! Get the task's initial update time in nanoseconds
    uint64_t getFirstTaskTime() const {
        return this->firstUpdateNanos;
    }

private:
    //! Project a time onto this task's lattice of scheduled updates
    /*!
     *  Let `p` be `TaskPeriod` and `k` be `FirstTaskTime`.
     *  Every update of this task is some multiple of `p` plus an offset `k`.
     *  In other words, every update `u` is `p*n + k` for some non-negative `n`.
     *
     *  This method takes an arbitrary time `t` and finds the latest `u` that is
     *  no later than `t`.
     */
    uint64_t projectToCurrentSchedule(uint64_t time);

public:
    //! The sequence of modules in this task, ordered by priority
    /*!
     *  @important
     *    This vector *must* remain in sorted priority order. It should never
     *    be modified by clients of `SysModelTask`.
     *
     *  @todo
     *    This should be `private`. Why is this not `private`??
     */
    std::vector<ModelPriorityPair> TaskModels{};

    //! A configurable, human-readable name for this task
    std::string TaskName = "";

    //! Whether the task's modules will be updated at subsequent update times
    /*!
     *  @todo
     *    Either this field should be made `private` and a getter method added,
     *    or this field should be made `public` and the `enable()` and `disable()`
     *    methods removed. (I vote for the latter.)
     */
    bool taskActive = true;

private:
    //! The next time (in nanoseconds) at which the task's modules will be updated
    uint64_t nextUpdateNanos;

    //! The interval of nanoseconds that will elapse between updates
    uint64_t updatePeriodNanos;

    //! The first time (in nanoseconds) at which the task's modules will be updated
    uint64_t const firstUpdateNanos;
};

#endif
