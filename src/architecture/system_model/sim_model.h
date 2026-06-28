// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SimModel_HH_
#define _SimModel_HH_

#include <architecture/system_model/sys_process.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

#include <vector>

//! This class handles the management of a given "thread" of execution and provides the main mechanism for running
//! concurrent jobs inside BSK
class SimThreadExecution {
public:
    explicit SimThreadExecution(uint64_t currentSimNanos = 0) : currentThreadNanos(currentSimNanos) {}

    //! Update simulation stop time
    void updateNewStopTime(uint64_t newStopNanos) {
        this->stopThreadNanos = newStopNanos;
    }

    //! Clear the process list
    void clearProcessList() {
        this->processList.clear();
    }

    void selfInitProcesses() const;

    void resetProcesses();

    void addNewProcess(SysProcess* newProc);

    //< Step simulation until stop time reached
    void stepUntilStop();

    //! Step the processes sharing the earliest resumption time
    /*!
     *  Of all processes in the simulation, one or more will have the earliest
     *  resumption time. (If two processes resume at the same time, both are
     *  "earliest".) This method will allow those processes to resume, but potentially
     *  only partially: only the tasks within those processes whose priority is
     *  at least `stopPri` will be allowed to resume.
     *
     *  @param stopPri
     *    The least priority at which processes' tasks will be resumed
     */
    void singleStepProcesses(int64_t stopPri = -1);

    void moveProcessMessages() const;

    uint64_t getCurrentNanos() const {
        return this->CurrentNanos;
    }

    void setCurrentNanos(uint64_t currentNanos) {
        this->CurrentNanos = currentNanos;
    }

    uint64_t getNextTaskTime() const {
        return this->NextTaskTime;
    }

    void setNextTaskTime(uint64_t nextTaskTime) {
        this->NextTaskTime = nextTaskTime;
    }

    uint64_t getCurrentThreadNanos() const {
        return this->currentThreadNanos;
    }

    void setStopThreadNanos(uint64_t stopThreadNanos) {
        this->stopThreadNanos = stopThreadNanos;
    }

    uint64_t getStopThreadNanos() const {
        return this->stopThreadNanos;
    }

public:
    //! Current stop priority for thread
    int64_t stopThreadPriority = -1;

    //! Priority level for the next process
    int64_t nextProcPriority = -1;

private:
    //! Current simulation time available at thread
    uint64_t currentThreadNanos = 0;

    //! Current stop conditions for the thread
    uint64_t stopThreadNanos = 0;

    //! [ns] Current sim time
    uint64_t CurrentNanos = 0;

    //! [ns] time for the next Task
    uint64_t NextTaskTime = 0;

    //! Flag that will allow for easy concurrent locking
    bool threadRunning{};

    //! Flag that indicates that it is time to take thread down
    bool terminateThread{};

    //! List of processes associated with thread
    std::vector<SysProcess*> processList;
};

//! The top-level container for an entire simulation
class SimModel {
public:
    void selfInitSimulation();                                  //!< Method to initialize all added Tasks
    void resetInitSimulation();                                 //!< Method to reset all added tasks
    void stepUntilStop(uint64_t SimStopTime, int64_t stopPri);  //!< Step simulation until stop time uint64_t reached
    //! Step the processes sharing the earliest resumption time
    /*!
     *  Of all processes in the simulation, one or more will have the earliest
     *  resumption time. (If two processes resume at the same time, both are "earliest".)
     *  This method will allow those processes to resume, but potentially only
     *  partially: only the tasks within those processes whose priority is at least
     *  `stopPri` will be allowed to resume.
     *
     *  @param stopPri
     *    The least priority at which processes' tasks will be resumed
     */
    void singleStepProcesses(int64_t stopPri = -1);
    void addNewProcess(SysProcess* newProc);
    void resetSimulation();  //!< Reset simulation back to zero

    uint64_t getCurrentNanos() const;
    uint64_t getNextTaskTime() const;
    BSKLogger bskLogger;
    std::vector<SysProcess*> processList;  //!< -- List of processes we've created
    std::string SimulationName;            //!< -- Identifier for Sim
    int64_t nextProcPriority = -1;         //!< [-] Priority level for the next process

private:
    uint64_t CurrentNanos = 0;  //!< [ns] Current sim time
    uint64_t NextTaskTime = 0;  //!< [ns] time for the next Task

    SimThreadExecution workerThread{0};
};

#endif /* _SimModel_H_ */
