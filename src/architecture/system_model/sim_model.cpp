// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sim_model.h"

#include <architecture/system_model/sim_instant.h>

#include <algorithm>
#include <iostream>

static void activateNewThread(SimThreadExecution &theThread) {
    theThread.postInit();

    while (theThread.threadValid()) {
        theThread.lockThread();

        if (theThread.selfInitNow) {
            theThread.selfInitProcesses();
            theThread.selfInitNow = false;
        } else if (theThread.crossInitNow) {
            theThread.crossInitNow = false;
        } else if (theThread.resetNow) {
            theThread.resetProcesses();
            theThread.resetNow = false;
        } else {
            theThread.stepUntilStop();
        }
        theThread.unlockParent();
    }
}

//! Step a process until its next update time is after `stopTime`.
/*!
 *  @param[in] process
 *    The process to step
 *  @param[in] stopTime
 *    The time up to which (and including which) we want to step tasks.
 *  @return
 *    The time at which the next task after `stopTime` will occur.
 */
static SimInstant stepProcessUpTo(SysProcess* process, SimInstant stopTime) {
    while (true) {
        auto nextTaskTime = SimInstant::atNanos(process->getNextTaskTime()).atPriority(process->processPriority);

        if (stopTime < nextTaskTime) { return nextTaskTime; }

        process->singleStepNextTask(stopTime.realNanos);
    }
}

void SimThreadExecution::singleStepProcesses(int64_t const stopPri) {
    // Advance the simulation clock to the time of the next task.
    this->CurrentNanos = this->NextTaskTime;

    // Step all processes up to the desired stop instant.
    // Keep track of the earliest resumption time *following* the stop instant.
    auto stopTime = SimInstant::atNanos(this->CurrentNanos).atPriority(stopPri);
    auto nextTaskTime = SimInstant::endOfTime();
    for (auto &localProc : this->processList) {
        if (!localProc->isEnabled()) { continue; }

        // Bail early if we're being asked to terminate. Yes, this leaves the
        // SimThreadExecution object in a bit of a weird state. However, if
        // we're being asked to terminate, it will be disposed of shortly anyway.
        if (!this->threadValid()) { return; }

        nextTaskTime = std::min(nextTaskTime, stepProcessUpTo(localProc, stopTime));
    }

    // Record the next earliest resumption time. Note that `nextTaskTime` should
    // only be `endOfTime` if no process was enabled.
    if (nextTaskTime != SimInstant::endOfTime()) {
        this->NextTaskTime = nextTaskTime.realNanos;
        this->nextProcPriority = nextTaskTime.causalPriority;
    }
}

/*! This method steps the simulation until the specified stop time and
 stop priority have been reached.
 @return void
 */
void SimThreadExecution::stepUntilStop() {
    /*! - Note that we have to step until both the time is greater and the next
     Task's start time is in the future. If the NextTaskTime is less than
     SimStopTime, then the inPri shouldn't come into effect, so set it to -1
     (that's less than all process priorities, so it will run through the next
     process)*/
    int64_t inPri = stopThreadNanos == this->NextTaskTime ? stopThreadPriority : -1;
    while (this->threadValid()
           && (this->NextTaskTime < stopThreadNanos
               || (this->NextTaskTime == stopThreadNanos && this->nextProcPriority >= stopThreadPriority))) {
        this->singleStepProcesses(inPri);
        inPri = stopThreadNanos == this->NextTaskTime ? stopThreadPriority : -1;
    }
}

/*! This method is currently vestigial and needs to be populated once the message
    sharing process between different threads is handled.
    TODO: Make this method move messages safely between threads
 @return void
 */
void SimThreadExecution::moveProcessMessages() const {
    for (auto const* process : this->processList) {
        static_cast<void>(process);  // "use" the unused variable
        // process->routeInterfaces(this->CurrentNanos);
    }
}

/*! Once threads are released for execution, this method ensures that they finish
    their startup before the system starts to go through its initialization
    activities.  It's very similar to the locking process, but provides different
    functionality.
 @return void
 */
void SimThreadExecution::waitOnInit() {
    std::unique_lock<std::mutex> lck(this->initReadyLock);
    while (!this->threadActive()) { (this)->initHoldVar.wait(lck); }
}

/*! This method allows the startup activities to alert the parent thread once
    they have cleared their construction phase and are ready to go through
    initialization.
 @return void
 */
void SimThreadExecution::postInit() {
    std::unique_lock<std::mutex> lck(this->initReadyLock);
    this->threadReady();
    this->initHoldVar.notify_one();
}

/*! This method is used by the "child" thread to walk through all of its tasks
    and processes and initialize them serially.  Note that other threads can also
    be initializing their systems simultaneously.
 @return void
 */
void SimThreadExecution::selfInitProcesses() const {
    for (auto const &process : this->processList) { process->selfInitialize(); }
}

/*! This method allows the "child" thread to reset both its timing/scheduling, as
    well as all of its allocated tasks/modules when commanded.  This is always
    called during init, but can be called during runtime as well.
 @return void
 */
void SimThreadExecution::resetProcesses() {
    this->currentThreadNanos = 0;
    this->CurrentNanos = 0;
    this->NextTaskTime = 0;
    for (auto const &process : this->processList) { process->reset(this->currentThreadNanos); }
}

/*! This method pops a new process onto the execution stack for the "child"
 *  thread.  It allows the user to put specific processes onto specific threads
 *  if that is desired.
 *
 *  @todo
 *    Process priority still needs to be respected in multithreaded simulation.
 *    We can't guarantee ordering of processes *across* threads, but we can still
 *    ensure that the processes hosted within a single thread are prioritized
 *    properly.
 */
void SimThreadExecution::addNewProcess(SysProcess* newProc) {
    processList.push_back(newProc);
    newProc->setProcessControlStatus(true);
}

/*! This Constructor is used to initialize the top-level sim model.
 */
SimModel::SimModel() {
    // Spawn a worker thread
    auto* newThread = new SimThreadExecution(0, this->CurrentNanos);
    newThread->threadContext = new std::thread([=]() { activateNewThread(*newThread); });
    newThread->waitOnInit();

    this->threadList.push_back(newThread);
}

/*! This method walks through all of the child threads that have been created in
    the system, detaches them from the architecture, and then cleans up any
    memory that has been allocated to them in the architecture.  It just ensures
    clean shutdown of any existing runtime stuff.
 */
SimModel::~SimModel() {
    for (auto const &simThread : this->threadList) {
        simThread->killThread();
        simThread->unlockThread();
        simThread->threadContext->join();
        delete simThread->threadContext;
        delete simThread;
    }
    this->threadList.clear();
}

/*! This method steps the simulation until the specified stop time and
 stop priority have been reached.
 @param SimStopTime Nanoseconds to step the simulation for
 @param stopPri The priority level below which the sim won't go
 @return void
 */
void SimModel::stepUntilStop(uint64_t SimStopTime, int64_t stopPri) {
    std::cout << std::flush;
    for (auto const* simThread : this->threadList) { simThread->moveProcessMessages(); }
    for (auto const &simThread : this->threadList) {
        simThread->setStopThreadNanos(SimStopTime);
        simThread->stopThreadPriority = stopPri;
        if (simThread->procCount() > 0) { simThread->unlockThread(); }
    }
    this->NextTaskTime = (uint64_t) ~0;
    this->CurrentNanos = (uint64_t) ~0;
    for (auto const &simThread : this->threadList) {
        if (simThread->procCount() > 0) {
            simThread->lockParent();
            this->NextTaskTime =
                simThread->getNextTaskTime() < this->NextTaskTime ? simThread->getNextTaskTime() : this->NextTaskTime;
            this->CurrentNanos =
                simThread->getCurrentNanos() < this->CurrentNanos ? simThread->getCurrentNanos() : this->CurrentNanos;
        }
    }
}

/*! This method allows the user to attach a process to the simulation for
    execution.  Note that the priority level of the process determines what
    order it gets called in: higher priorities are called before lower
    priorities. If priorities are the same, the proc added first goes first.
    @return void
    @param newProc the new process to be added
*/
void SimModel::addNewProcess(SysProcess* newProc) {
    for (auto it = this->processList.begin(); it != this->processList.end(); it++) {
        if (newProc->processPriority > (*it)->processPriority) {
            this->processList.insert(it, newProc);
            return;
        }
    }
    this->processList.push_back(newProc);
}

/*! This method goes through all of the processes in the simulation,
 *  all of the tasks within each process, and all of the models within
 *  each task and self-inits them.
 @return void
 */
void SimModel::selfInitSimulation() {
    for (auto const &simThread : this->threadList) {
        simThread->selfInitNow = true;
        simThread->unlockThread();
    }
    for (auto const &simThread : this->threadList) { simThread->lockParent(); }
    this->NextTaskTime = 0;
    this->CurrentNanos = 0;
}

/*! This method goes through all of the processes in the simulation,
 *  all of the tasks within each process, and all of the models within
 *  each task and resets them.
 @return void
 */
void SimModel::resetInitSimulation() const {
    for (auto const &simThread : this->threadList) {
        simThread->resetNow = true;
        simThread->unlockThread();
    }
    for (auto const &simThread : this->threadList) { simThread->lockParent(); }
}

void SimModel::singleStepProcesses(int64_t const stopPri) {
    // Advance the simulation clock to the time of the next task.
    this->CurrentNanos = this->NextTaskTime;

    // Step all processes up to the desired stop instant.
    // Keep track of the earliest resumption time *following* the stop instant.
    auto stopTime = SimInstant::atNanos(this->CurrentNanos).atPriority(stopPri);
    auto nextTaskTime = SimInstant::endOfTime();
    for (auto &localProc : this->processList) {
        if (!localProc->isEnabled()) { continue; }

        nextTaskTime = std::min(nextTaskTime, stepProcessUpTo(localProc, stopTime));
    }

    // Record the next earliest resumption time. Note that `nextTaskTime` should
    // only be `endOfTime` if no process was enabled.
    if (nextTaskTime != SimInstant::endOfTime()) {
        this->NextTaskTime = nextTaskTime.realNanos;
        this->nextProcPriority = nextTaskTime.causalPriority;
    }
}

/*! This method is used to reset a simulation to time 0. It sets all process and
 * tasks back to the initial call times. It clears all message logs. However,
 * it does not clear all message buffers and does not reset individual models.
 @return void
 */
void SimModel::resetSimulation() {
    this->NextTaskTime = 0;
    this->CurrentNanos = 0;

    // Re-initialize threads
    for (auto const &simThread : this->threadList) {
        simThread->clearProcessList();
        simThread->setNextTaskTime(this->CurrentNanos);
        simThread->setCurrentNanos(this->CurrentNanos);
        simThread->nextProcPriority = (!this->processList.empty()) ? this->processList.front()->processPriority : -1;
    }

    // Assign processes to threads in a round-robin fashion
    auto thrIt = threadList.begin();
    for (auto &process : this->processList) {
        process->setProcessControlStatus(false);
        process->reInitialize();

        (*thrIt)->addNewProcess(process);

        // Advance to the next thread, looping back around if necessary
        ++thrIt;
        if (thrIt == threadList.end()) { thrIt = threadList.begin(); }
    }
}

uint64_t SimModel::getCurrentNanos() const {
    return this->CurrentNanos;
}

uint64_t SimModel::getNextTaskTime() const {
    return this->NextTaskTime;
}
