// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sys_process
#define XMAheader_sys_process

#include <architecture/system_model/sys_model_task.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

#include <vector>

//! A task paired with its priority among tasks within its containing process
struct ModelScheduleEntry final {
    //! The cached value of `SysModelTask::getNextStartTime()`
    uint64_t NextTaskStart;

    //! The cached value of `SysModelTask::getTaskPeriod()`
    uint64_t TaskUpdatePeriod;

    //! The priority of this task among others within its containing process
    int32_t taskPriority;

    //! A non-owning pointer to a task
    SysModelTask* TaskPtr;
};

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
public:
    //! Obtain an empty simulation process
    /*!
     *  @param[in] name
     *    A human-readable name for this process
     */
    explicit SysProcess(std::string name = "") : processName{name} {}

    //! Insert a task with a given priority into the process's collection of tasks
    /*!
     *  @param[in] task
     *    A non-owning pointer to the module to be added to the task
     *  @param[in] priority
     *    The priority of the given module (higher is earlier).
     */
    void addTask(SysModelTask* task, int32_t priority = -1);

    //! Reset the next update time for all tasks in this process
    /*!
     *  This method also ensures that processes are correctly ordered by priority.
     *
     *  @todo
     *    Unless a client of `SysProcess` messes with `processTasks` directly,
     *    it shouldn't be possible for `processTasks` to get out of order to begin
     *    with. This should be confirmed, the redundant code reduced, and the
     *    documentation clarified.
     *
     *  @todo
     *    Remove this method. There doesn't seem to be any reason to ever invoke
     *    this without also invoking `reset()` and `selfInitialize()`, both of
     *    which also set the process-level next update time.
     */
    void reInitialize();

    //! Reset all tasks in the process, in priority order
    /*!
     *  @param[in] initialSimNanos
     *    The simulation instant associated with the initial simulation state
     */
    void reset(uint64_t initialSimNanos);

    //! Step the task with the earliest next update time at or before the argument
    /*!
     *  @param[in] nextSimNanos
     *    The simulation instant associated with the desired simulation state
     */
    void singleStepNextTask(uint64_t nextSimNanos);

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

    //! Reset the next update time for this process to zero nanoseconds
    /*!
     * @todo
     *   Remove this method. There doesn't seem to be any reason to ever invoke
     *   this without also invoking `reset()` and `reInitialize()`, and the former
     *   also sets the next update time (to something more appropriate than 0ns).
     */
    void selfInitialize() {
        this->nextTaskTime = 0;
    }

    //! Allow this process to participate in simulation
    void enable() {
        this->processActive = true;
    }

    //! Prevent this process from participating in simulation
    void disable() {
        this->processActive = false;
    }

    //! Determine whether the process is currently participating in simulation
    /*! @todo The field is already public. Remove this getter. */
    bool isEnabled() const {
        return this->processActive;
    }

    //! Change the human-readable name of this process
    /*! @todo The field is already public. Remove this setter. */
    void setName(std::string const &newName) {
        this->processName = newName;
    }

    //! Obtain the human-readable name of this process
    /*! @todo The field is already public. Remove this getter. */
    std::string getName() const {
        return this->processName;
    }

    //! Change the priority of this process
    /*!
     *  @warning
     *    If this process has already been added to a simulation container, the
     *    container will not be notified of the new priority. This may cause the
     *    process to be reset in an order that doesn't comport with its new priority.
     */
    void setPriority(int64_t newPriority) {
        this->processPriority = newPriority;
    }

    //! Invoke `SysModelTask::disable` on every task in the module
    /*! @todo Remove this; clients should be able to iterate over tasks directly. */
    void disableTasks() {
        for (auto const &entry : this->processTasks) { entry.TaskPtr->disable(); }
    }

    //! Invoke `SysModelTask::enable` on every task in the module
    /*! @todo Remove this; clients should be able to iterate over tasks directly. */
    void enableTasks() {
        for (auto const &entry : this->processTasks) { entry.TaskPtr->enable(); }
    }

    //! Check whether this process has been allocated to a simulation thread
    /*! @todo Remove this getter. The field is already public! */
    bool getProcessControlStatus() const {
        return this->processOnThread;
    }

    //! Indicate whether this process has been allocated to a simulation thread
    /*! @todo Remove this setter. The field is already public! */
    void setProcessControlStatus(bool processTaken) {
        processOnThread = processTaken;
    }

    //! Determine the next update time of the next-soonest task to update
    uint64_t getNextTaskTime() const {
        return this->nextTaskTime;
    }

private:
    //! Insert a scheduled task entry into the ordered list of tasks in this process
    /*!
     *  @param[in] scheduleEntry
     *    A descriptor containing a task to schedule and its start time and period.
     */
    void scheduleTask(ModelScheduleEntry const &scheduleEntry);

    //! Index the task with the soonest next update time
    std::vector<ModelScheduleEntry>::iterator getNextTask();

public:
    //! The schedule of tasks being performed by this process
    /*!
     *  @important
     *    This vector *must* remain in sorted priority order. It should never
     *    be modified by clients of `SysProcess`.
     *
     *  @todo
     *    This should be `private`. Why is this not `private`??
     */
    std::vector<ModelScheduleEntry> processTasks;

    //! A configurable, human-readable name for this process
    std::string processName;

    //! Whether the process is currently participating in simulation at all
    /*!
     *  @todo
     *    Why is this `false` by default? Currently, this is automatically set
     *    to `true` whenever a task is scheduled -- even if a client explicitly
     *    called `disable()` and intended for the process to be disabled at the
     *    start of simulation. If an enabled process with no tasks is updated,
     *    it will just return immediately without fault. So it's entirely unclear
     *    why we want or need a process to start out disabled.
     */
    bool processActive = false;

    //! Whether this process has been added to a thread for execution
    /*!
     *  @todo
     *    This is not the responsibility of `SysProcess` to keep track of.
     *    No code in this class reads or writes this field except a transparent
     *    getter/setter pair (which, mind, are redundant for a *public* field).
     *    This state should be moved to the class that actually cares about it,
     *    `SimModel`.
     */
    bool processOnThread = false;

    //! The priority of this process among others within its containing thread
    /*!
     *  Tasks in processes of higher priority will be updated before tasks in
     *  processes of lower priority. *Intra*-process priority is still dictated
     *  by the individual priority associated to each task in the same process.
     *  In contrast, processes across different threads are effectively unordered.
     *
     *  @todo
     *    Process priority should really be owned by whichever container tracks
     *    this process (`SimModel` in single-threaded simulations and `SimThreadExecution`
     *    in multi-thraded simulations). Nothing in this class uses this field
     *    except a transparent setter (which is redundant for a *public* field).
     */
    int64_t processPriority = -1;

private:
    //! The next soonest time (in nanoseconds) at which some task will be updated
    uint64_t nextTaskTime = 0;
};

#endif
