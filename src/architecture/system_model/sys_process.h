// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sys_process
#define XMAheader_sys_process

#include <architecture/system_model/sys_model_task.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

#include <memory>
#include <vector>

//! A collection of concurrent tasks to be updated on independent schedules
/*!
 *  An instance of `SysProcess` (a "process") is a collection of tasks, each
 *  with its own update cadence. If two tasks update at the same time, ties are
 *  broken by a task-level priority, where higher-priority tasks go first.
 *
 *  In principle, a process is a reasonably self-contained and internally consistent
 *  model of simulation. A χmera simulation with multiple processes may be thought
 *  of as an integrated multi-model simulation.
 */
class SysProcess final {
    friend class SimModel;

public:
    SysProcess(SysProcess const&) = delete;
    SysProcess& operator=(SysProcess const&) = delete;

    SysProcess(SysProcess&&) = delete;
    SysProcess& operator=(SysProcess&&) = delete;

    //! Obtain an empty simulation process
    /*!
     *  @param[in] name
     *    A human-readable name for this process
     */
    explicit SysProcess(std::string name = "", int64_t priority = -1) : processName{name}, processPriority{priority} {}

    //! Add a new task with the given update schedule and priority
    /*!
     *  @param[in] updatePeriodNanos
     *    The interval of nanoseconds that should elapse between updates
     *  @param[in] firstUpdateNanos
     *    The time at which the task should be first updated
     *  @param[in] priority
     *    The priority of the given task (higher is earlier).
     */
    SysModelTask &addTask(uint64_t updatePeriodNanos = 100, uint64_t firstUpdateNanos = 0, int32_t priority = -1);

    //! Change a named task's period and recompute its next update time
    /*!
     *  @param[in] taskName
     *    The name of the task to change the period of
     *  @param[in] newPeriod
     *    The new period between update times in nanoseconds
     *  @return
     *    Whether a task with the given name was found; if false, no period was
     *    changed
     */
    bool changeTaskPeriod(std::string const &taskName, uint64_t newPeriod);

    //! Allow this process to participate in simulation
    void enable() {
        this->enabled = true;
    }

    //! Prevent this process from participating in simulation
    void disable() {
        this->enabled = false;
    }

    //! Determine whether the process is currently participating in simulation
    /*! @todo The field is already public. Remove this getter. */
    bool isEnabled() const {
        return this->enabled;
    }

    //! Invoke `SysModelTask::disable` on every task in the module
    /*! @todo Remove this; clients should be able to iterate over tasks directly. */
    void disableTasks() {
        for (auto const &entry : this->processTasks) { entry->disable(); }
    }

    //! Invoke `SysModelTask::enable` on every task in the module
    /*! @todo Remove this; clients should be able to iterate over tasks directly. */
    void enableTasks() {
        for (auto const &entry : this->processTasks) { entry->enable(); }
    }

    //! Get an immutable view on the list of tasks in this process
    /*!
     *  Note that it is the list of tasks that is immutable here, not the tasks
     *  themselves. It is perfectly legal to mutate one of the tasks obtained
     *  from this method, so long as no other protocol of use is violated.
     */
    std::vector<SysModelTask*> const &getTasks() const {
        return this->processTasks;
    }

private:
    //! Insert a scheduled task entry into the ordered list of tasks in this process
    void scheduleTask(SysModelTask* task);

    //! Index the task with the soonest next update time
    std::vector<SysModelTask*>::iterator getNextTask();

public:
    //! A configurable, human-readable name for this process
    std::string processName = "";

    //! The priority of this process among others within its containing thread
    /*!
     *  Tasks in processes of higher priority will be updated before tasks in
     *  processes of lower priority. *Intra*-process priority is still dictated
     *  by the individual priority associated to each task in the same process.
     *  In contrast, processes across different threads are effectively unordered.
     *
     *  @todo
     *    Process priority should really be owned by whichever container tracks
     *    this process (`SimModel` in single-threaded simulations). Nothing in
     *    this class uses this field except a transparent setter (which is redundant
     *    for a *public* field).
     */
    int64_t const processPriority = -1;

private:
    //! Whether the process is currently participating in simulation at all
    bool enabled = true;

    //! The schedule of tasks being performed by this process
    std::vector<SysModelTask*> processTasks = {};

    //! The collection of tasks in this process, sans scheduling information
    std::vector<std::unique_ptr<SysModelTask>> allocatedTasks = {};
};

#endif
