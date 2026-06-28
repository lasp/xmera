// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sim_model.h"

#include <architecture/system_model/sim_instant.h>

#include <algorithm>
#include <iostream>

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

/*! This method steps the simulation until the specified stop time and
 stop priority have been reached.
 @param SimStopTime Nanoseconds to step the simulation for
 @param stopPri The priority level below which the sim won't go
 @return void
 */
void SimModel::stepUntilStop(uint64_t SimStopTime, int64_t stopPri) {
    std::cout << std::flush;

    /*! - Note that we have to step until both the time is greater and the next
     Task's start time is in the future. If the NextTaskTime is less than
     SimStopTime, then the inPri shouldn't come into effect, so set it to -1
     (that's less than all process priorities, so it will run through the next
     process)*/
    auto simStopTime = SimInstant::atNanos(SimStopTime).atPriority(stopPri);
    while (true) {
        auto nextTaskTime = SimInstant::atNanos(this->NextTaskTime).atPriority(this->nextProcPriority);

        if (nextTaskTime > simStopTime) { break; }

        int64_t inPri = (SimStopTime == this->NextTaskTime) ? stopPri : -1;
        this->singleStepProcesses(inPri);
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
    auto it = this->processList.begin();
    for (; it != this->processList.end(); ++it) {
        if (newProc->processPriority > (*it)->processPriority) { break; }
    }
    this->processList.insert(it, newProc);
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

/*! This method is used to reset a simulation to time 0. It fully resets all
 * processes, tasks, and modules.
 @return void
 */
void SimModel::resetSimulation() {
    // Reset all processes
    this->CurrentNanos = 0;
    for (auto &process : this->processList) { process->reset(this->CurrentNanos); }

    // Figure out which process will update first
    auto nextTaskTime = SimInstant::endOfTime();
    for (auto &process : this->processList) {
        if (!process->isEnabled()) { continue; }

        auto nextProcTime = SimInstant::atNanos(process->getNextTaskTime()).atPriority(process->processPriority);
        nextTaskTime = std::min(nextTaskTime, nextProcTime);
    }
    this->NextTaskTime = nextTaskTime.realNanos;
    this->nextProcPriority = nextTaskTime.causalPriority;
}

uint64_t SimModel::getCurrentNanos() const {
    return this->CurrentNanos;
}

uint64_t SimModel::getNextTaskTime() const {
    return this->NextTaskTime;
}
