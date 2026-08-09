// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_sim_model
#define XMAheader_sim_model

#include <architecture/system_model/sim_instant.h>
#include <architecture/system_model/sys_process.h>

#include <stdint.h>

#include <memory>
#include <vector>

//! The top-level container for an entire simulation
class SimModel final {
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
    void singleStepProcesses(int64_t stopPri = -1);

    //! Step the simulation forward until the next update would occur after the given stop time
    /*!
     *  This helper method invokes `singleStepProcesses` repeatedly, stopping
     *  only once the given time has been reached.
     *
     *  @important
     *    This method must only be invoked once `resetSimulation` has been invoked
     *    at least once.
     *
     *  @param[in] SimStopTime
     *    The latest time at which processes should be updated
     *  @param[in] stopPri
     *    The least priority at which processes should be updated
     */
    void stepUntilStop(uint64_t SimStopTime, int64_t stopPri = -1);

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
        return this->NextTaskTime;
    }

    //! Get the priority of the next process to be updated
    /*!
     *  This method should only be used during a simulation. If called before
     *  a simulation begins, or after adding new processes, its value will be
     *  unreliable.
     */
    int64_t getNextProcPriority() const {
        return this->nextProcPriority;
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
    //! Step a process until its next update time is after `stopTime`.
    /*!
     *  @param[in] process
     *    The process to step
     *  @param[in] stopTime
     *    The time up to which (and including which) we want to step tasks.
     *  @return
     *    The time at which the next task after `stopTime` will occur.
     */
    static SimInstant stepProcessUpTo(SysProcess &process, SimInstant stopTime);

private:
    //! The time at which the simulation was last updated or reset
    uint64_t CurrentNanos = 0;

    //! The time at which the simulation will next be updated
    uint64_t NextTaskTime = 0;

    //! The priority of the next process to be updated
    int64_t nextProcPriority = -1;

    //! The collection of processes to be simulated, in priority order
    std::vector<std::unique_ptr<SysProcess>> processList = {};
};

#endif
