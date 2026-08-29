// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sim_model
#define XMAheader_sim_model

#include <architecture/_GeneralModuleFiles/sys_model.h>

#include <stdint.h>

#include <compare>
#include <memory>
#include <vector>

class SimModel;

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
 */
class SysModelTask final {
    friend class SimModel;
    friend class SysProcess;
    struct Passkey {};

public:
    SysModelTask(SysModelTask const&) = delete;
    SysModelTask& operator=(SysModelTask const&) = delete;

    SysModelTask(SysModelTask&&) = delete;
    SysModelTask& operator=(SysModelTask&&) = delete;

    //! Obtain an empty task with a given update period and start time
    /*!
     *  @param[in] updatePeriodNanos
     *    The interval of nanoseconds that should elapse between updates
     *  @param[in] firstUpdateNanos
     *    The time at which the task should be first updated
     */
    explicit SysModelTask(
        SysModelTask::Passkey _ignored,
        SimModel &owner,
        uint64_t updatePeriodNanos = 100,
        uint64_t firstUpdateNanos = 0,
        int64_t priority = -1
    ) : priority(priority)
      , owner(owner)
      , nextUpdateNanos(firstUpdateNanos)
      , updatePeriodNanos(updatePeriodNanos)
      , firstUpdateNanos(firstUpdateNanos)
    {}

    //! Insert a module into the task's sequence of modules with a given priority
    /*!
     *  @param[in] module
     *    A non-owning pointer to the module to be added to the task
     *  @param[in] priority
     *    The priority of the given module (higher is earlier).
     */
    void addModel(SysModel* module, int32_t priority = -1);

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

    //! Get an immutable view on the list of models in this task
    /*!
     *  Note that it is the list of models that is immutable here, not the models
     *  themselves. It is perfectly legal to mutate one of the models obtained
     *  from this method, so long as no other protocol of use is violated.
     */
    std::vector<ModelPriorityPair> const &getModels() const {
        return this->TaskModels;
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

    //! The priority of this task among others within its containing process
    int32_t const priority;

private:
    //! The `SimModel` that contains this task.
    SimModel &owner;

    //! The next time (in nanoseconds) at which the task's modules will be updated
    uint64_t nextUpdateNanos;

    //! The interval of nanoseconds that will elapse between updates
    uint64_t updatePeriodNanos;

    //! The first time (in nanoseconds) at which the task's modules will be updated
    uint64_t const firstUpdateNanos;

    //! The sequence of modules in this task, ordered by priority
    std::vector<ModelPriorityPair> TaskModels{};
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
    friend class SimModel;
    struct Passkey {};

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
    explicit SysProcess(
        SysProcess::Passkey _ignored,
        SimModel &owner,
        size_t processId,
        std::string name = "",
        int64_t priority = -1
    ) : processName{name}, processPriority{priority}, owner{owner}, processId{processId} {}

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
        for (auto &entry : this->processTasks) { entry->disable(); }
    }

    //! Invoke `SysModelTask::enable` on every task in the module
    /*! @todo Remove this; clients should be able to iterate over tasks directly. */
    void enableTasks() {
        for (auto &entry : this->processTasks) { entry->enable(); }
    }

    //! Get an immutable view on the list of tasks in this process
    /*!
     *  Note that it is the list of tasks that is immutable here, not the tasks
     *  themselves. It is perfectly legal to mutate one of the tasks obtained
     *  from this method, so long as no other protocol of use is violated.
     */
    std::vector<std::unique_ptr<SysModelTask>> const &getTasks() const {
        return this->processTasks;
    }

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
    //! The `SimModel` that contains this process.
    SimModel &owner;

    //! Whether the process is currently participating in simulation at all
    bool enabled = true;

    //! The creation-order ID of this process
    size_t const processId;

    //! The collection of tasks performed by this process
    std::vector<std::unique_ptr<SysModelTask>> processTasks = {};
};

//! The top-level container for an entire simulation
class SimModel final {
    //! A record of a job to be performed during simulation
    // A "job" is just a task paired with its containing process, made comparable for priority queueing.
    struct Job final {
        //! The process containing the task to be updated by this job
        SysProcess const* process;
        //! The creation-ordered ID of the process
        size_t process_id;
        //! The task to be updated by this job
        SysModelTask* task;
        //! The creation-ordered ID of the task
        size_t task_id;

        //! Determine whether this job is schedule for no later than the given threshold time.
        bool noLaterThan(uint64_t nanos, int64_t priority) const {
            return (this->task->getNextStartTime() < nanos) ? true
                 : (this->task->getNextStartTime() > nanos) ? false
                 : (this->process->processPriority >= priority);
        }

        //! Compare two jobs lexicographically by their next update time, their inter-process priority, and their inter-process priority
        std::weak_ordering operator<=>(Job const &other) const {
            return (other.task->getNextStartTime() < this->task->getNextStartTime()) ? std::weak_ordering::less
                 : (other.task->getNextStartTime() > this->task->getNextStartTime()) ? std::weak_ordering::greater
                 : (this->process->processPriority < other.process->processPriority) ? std::weak_ordering::less
                 : (this->process->processPriority > other.process->processPriority) ? std::weak_ordering::greater
                 : (other.process_id < this->process_id) ? std::weak_ordering::less
                 : (other.process_id > this->process_id) ? std::weak_ordering::greater
                 : (this->task->priority < other.task->priority) ? std::weak_ordering::less
                 : (this->task->priority > other.task->priority) ? std::weak_ordering::greater
                 : (other.task_id < this->task_id) ? std::weak_ordering::less
                 : (other.task_id > this->task_id) ? std::weak_ordering::greater
                 : std::weak_ordering::equivalent;
        }
    };

public:
    SimModel() = default;

    SimModel(SimModel const&) = delete;
    SimModel& operator=(SimModel const&) = delete;

    SimModel(SimModel&&) = delete;
    SimModel& operator=(SimModel&&) = delete;

    //! Add a new process to be simulated
    /*!
     *  The priority of the given process dictates the order in which it is reset,
     *  and the order in which it is updated if multiple processes update at the
     *  same time. Higher priorities go first; if two processes have the same
     *  priority, the one added earlier takes precedence.
     *
     *  @important
     *    This method must not be invoked during simulation; it must only be
     *    invoked before `resetSimulation` occurs for any given simulation run.
     */
    SysProcess &addNewProcess(std::string name = "", int64_t priority = -1);

    //! Reset all simulation elements to the initial state
    /*!
     *  This method recursively resets all processes, tasks, and models in the
     *  simulation, ensuring that everything is synchronized at time 0.
     */
    void resetSimulation();

    //! Step the processes sharing the earliest resumption time
    /*!
     *  Of all processes in the simulation, one or more will have the earliest
     *  resumption time. (If two processes resume at the same time, both are "earliest".)
     *  This method will allow those processes to resume, but potentially only
     *  partially: only the tasks within those processes whose priority is at least
     *  `stopPri` will be allowed to resume.
     *
     *  @important
     *    This method must only be invoked once `resetSimulation` has been invoked
     *    at least once.
     *
     *  @param[in] stopPri
     *    The least priority at which processes should be updated
     */
    void singleStepProcesses(int64_t const stopPriority = -1) {
        this->stepUntilStop(this->getNextTaskTime());
    }

    //! Step the simulation forward until the next update would occur after the given stop time
    /*!
     *  This helper method invokes `singleStepProcesses` repeatedly, stopping
     *  only once the given time has been reached.
     *
     *  @important
     *    This method must only be invoked once `resetSimulation` has been invoked
     *    at least once.
     *
     *  @param[in] stopNanos
     *    The latest time at which processes should be updated
     *  @param[in] stopPriority
     *    The least priority at which processes should be updated
     */
    void stepUntilStop(uint64_t stopNanos, int64_t stopPriority  = -1);

    //! Get the time at which the simulation was last stepped or reset
    uint64_t getCurrentNanos() const {
        return this->CurrentNanos;
    }

    //! Get the time at which the simulation will next be stepped
    /*!
     *  This method should only be used during a simulation. If called before
     *  a simulation begins, or after adding new processes, its value will be
     *  unreliable.
     */
    uint64_t getNextTaskTime() const {
        this->ensureHeap();

        return (!this->jobHeap.empty())
            ? jobHeap.front().task->getNextStartTime()
            : std::numeric_limits<uint64_t>::max();
    }

    //! Get the priority of the next process to be updated
    /*!
     *  This method should only be used during a simulation. If called before
     *  a simulation begins, or after adding new processes, its value will be
     *  unreliable.
     */
    int64_t getNextProcPriority() const {
        this->ensureHeap();

        return (!this->jobHeap.empty())
            ? jobHeap.front().process->processPriority
            : std::numeric_limits<int64_t>::min();
    }

    //! Get an immutable view on the list of processes in this simulation
    /*!
     *  Note that it is the list of processes that is immutable here, not the
     *  processes themselves. It is perfectly legal to mutate one of the processes
     *  obtained from this method, so long as no other protocol of use is violated.
     */
    std::vector<std::unique_ptr<SysProcess>> const &getProcesses() const {
        return this->processList;
    }

private:
    //! Re-heapify the `jobHeap` if `isHeap` is false
    void ensureHeap() const;

private:
    //! The time at which the simulation was last updated or reset
    uint64_t CurrentNanos = 0;

    //! The collection of processes to be simulated, in priority order
    std::vector<std::unique_ptr<SysProcess>> processList = {};

    //! A prioritized heap of simulation jobs
    // This is `mutable` so that the heap invariant can remain broken until the last moment,
    // allowing re-heapification to occur during logically const queries.
    mutable std::vector<Job> jobHeap = {};

    //! Whether the `jobHeap` field currently has the heap property
    mutable bool isHeap = false;
};

#endif
