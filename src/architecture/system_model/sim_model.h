// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _SimModel_HH_
#define _SimModel_HH_

#include <architecture/system_model/sys_process.h>
#include <architecture/utilities/bskLogging.h>

#include <stdint.h>

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <semaphore>
#include <set>
#include <thread>
#include <vector>

//! This class handles the management of a given "thread" of execution and provides the main mechanism for running
//! concurrent jobs inside BSK
class SimThreadExecution {
public:
    explicit SimThreadExecution(uint64_t threadIdent = 0, uint64_t currentSimNanos = 0)
        : threadID(threadIdent), currentThreadNanos(currentSimNanos) {}

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

    //! Gets the current "thread-count" in the system
    uint64_t procCount() const {
        return this->processList.size();
    }

    //! Is the thread is currently allocated processes and is in execution
    bool threadActive() const {
        return this->threadRunning;
    }

    //! Put the thread into a running state
    void threadReady() {
        this->threadRunning = true;
    }

    void waitOnInit();
    void postInit();

    //! Is the thread currently usable or if it has been requested to shutdown
    bool threadValid() const {
        return !this->terminateThread;
    }

    //! Asks the thread to no longer be alive
    void killThread() {
        this->terminateThread = true;
    }

    /*!
     *  This method provides a synchronization mechanism for the "child" thread
     *  ensuring that it can be held at a fixed point after it finishes the
     *  execution of a given frame until it is released by the "parent" thread.
     */
    void lockThread() {
        this->selfThreadLock.acquire();
    }

    /*!
     *  This method provides an entry point for the "parent" thread to release the
     *  child thread for a single frame's execution.  It is intended to only be
     *  called from the parent thread.
     */
    void unlockThread() {
        this->selfThreadLock.release();
    }

    /*!
     *  This method provides a forced synchronization on the "parent" thread so that
     *  the parent and all other threads in the system can be forced to wait at a
     *  known time until this thread has finished its execution for that time.
     */
    void lockParent() {
        this->parentThreadLock.acquire();
    }

    /*!
     *  This method provides an entry point for the "child" thread to unlock the
     *  parent thread after it has finished its execution in a frame.  That way the
     *  parent and all of its other children have to wait for this child to finish
     *  its execution.
     */
    void unlockParent() {
        this->parentThreadLock.release();
    }

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

    //! Identifier for thread
    uint64_t threadID = 0;

    //! std::thread data for concurrent execution
    std::thread* threadContext = nullptr;

    //! Priority level for the next process
    int64_t nextProcPriority = -1;

    //! Flag requesting self init
    bool selfInitNow{};

    //! Flag requesting cross-init
    bool crossInitNow{};

    //! Flag requesting that the thread execute reset
    bool resetNow{};

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

    //! Lock that ensures parent thread won't proceed
    std::binary_semaphore parentThreadLock{0};

    //! Lock that ensures this thread only reaches allowed time
    std::binary_semaphore selfThreadLock{0};

    //! List of processes associated with thread
    std::vector<SysProcess*> processList;

    //! Lock function to ensure runtime locks are configured
    std::mutex initReadyLock;

    //! Conditional variable used to prevent race conditions
    std::condition_variable initHoldVar;
};

//! The top-level container for an entire simulation
class SimModel {
public:
    SimModel();
    ~SimModel();

    void selfInitSimulation();                                  //!< Method to initialize all added Tasks
    void resetInitSimulation() const;                           //!< Method to reset all added tasks
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
    void addProcessToThread(SysProcess* newProc, uint64_t threadSel);
    void resetSimulation();  //!< Reset simulation back to zero
    void clearProcsFromThreads() const;
    void resetThreads(uint64_t threadCount);
    void deleteThreads();
    void assignRemainingProcs();

    uint64_t getThreadCount() const {
        return threadList.size();
    }  //!< returns the number of threads used

    uint64_t getCurrentNanos() const;
    uint64_t getNextTaskTime() const;
    BSKLogger bskLogger;
    std::vector<SysProcess*> processList;           //!< -- List of processes we've created
    std::vector<SimThreadExecution*> threadList{};  //!< -- Array of threads that we're running on
    std::string SimulationName;                     //!< -- Identifier for Sim
    int64_t nextProcPriority = -1;                  //!< [-] Priority level for the next process

private:
    uint64_t CurrentNanos = 0;  //!< [ns] Current sim time
    uint64_t NextTaskTime = 0;  //!< [ns] time for the next Task
};

#endif /* _SimModel_H_ */
