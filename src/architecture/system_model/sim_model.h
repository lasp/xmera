// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sim_model
#define XMAheader_sim_model

#include <architecture/system_model/sim_instant.h>
#include <architecture/system_model/sys_process.h>

#include <stdint.h>

#include <compare>
#include <memory>
#include <vector>

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

        //! Get the time at which this job should be performed relative to those of other processes
        SimInstant nextProcessTime() const {
            return SimInstant::atNanos(this->task->getNextStartTime()).atPriority(this->process->processPriority);
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
        return (!this->jobHeap.empty()) ? jobHeap.front().task->getNextStartTime() : SimInstant::endOfTime().realNanos;
    }

    //! Get the priority of the next process to be updated
    /*!
     *  This method should only be used during a simulation. If called before
     *  a simulation begins, or after adding new processes, its value will be
     *  unreliable.
     */
    int64_t getNextProcPriority() const {
        return (!this->jobHeap.empty()) ? jobHeap.front().process->processPriority : SimInstant::endOfTime().causalPriority;
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
    //! The time at which the simulation was last updated or reset
    uint64_t CurrentNanos = 0;

    //! A prioritized heap of simulation jobs
    std::vector<Job> jobHeap = {};

    //! Whether the `jobHeap` field currently has the heap property.
    bool isHeap = false;

    //! The collection of processes to be simulated, in priority order
    std::vector<std::unique_ptr<SysProcess>> processList = {};
};

#endif
