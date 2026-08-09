// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sim_model.h"

#include <algorithm>
#include <iostream>

SimInstant SimModel::stepProcessUpTo(SysProcess &process, SimInstant stopTime) {
    if (process.processTasks.empty()) { return SimInstant::endOfTime().atPriority(process.processPriority); }

    while (true) {
        auto nextTaskIt = process.getNextTask();
        auto nextTaskTime = SimInstant::atNanos(nextTaskIt->TaskPtr->nextUpdateNanos).atPriority(process.processPriority);
        process.nextTaskTime = nextTaskTime.realNanos;

        if (stopTime < nextTaskTime) { return nextTaskTime; }
        if (nextTaskTime.realNanos == SimInstant::endOfTime().realNanos) { return nextTaskTime; }

        // Update the next task, and record when it wants to be updated again
        nextTaskIt->TaskPtr->nextUpdateNanos += nextTaskIt->TaskPtr->updatePeriodNanos;
        if (nextTaskIt->TaskPtr->taskActive) {
            for (auto &modelPair : nextTaskIt->TaskPtr->TaskModels) {
                modelPair.ModelPtr->updateState(stopTime.realNanos);
            }
        }
    }
}

void SimModel::stepUntilStop(uint64_t SimStopTime, int64_t stopPri) {
    //! @todo Remove this misplaced flush on stdout.
    std::cout << std::flush;

    // Keep single-stepping until the next process would occur after the stop time.
    auto simStopTime = SimInstant::atNanos(SimStopTime).atPriority(stopPri);
    while (true) {
        auto nextTaskTime = SimInstant::atNanos(this->NextTaskTime).atPriority(this->nextProcPriority);

        if (nextTaskTime > simStopTime) { break; }

        // If we're not stopping at this time, run processes of *all* priorities.
        int64_t inPri = (SimStopTime == this->NextTaskTime) ? stopPri : SimInstant::endOfTime().causalPriority;
        this->singleStepProcesses(inPri);
    }
}

SysProcess &SimModel::addNewProcess(std::string name, int64_t priority) {
    // Find the index separating lower priorities from higher priorities.
    auto it = this->processList.begin();
    for (; it != this->processList.end(); ++it) {
        if (priority > (*it)->processPriority) { break; }
    }

    it = this->processList.emplace(it, std::make_unique<SysProcess>(name, priority));

    return *it->get();
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

        nextTaskTime = std::min(nextTaskTime, stepProcessUpTo(*localProc, stopTime));
    }

    // Record the next earliest resumption time. Note that `nextTaskTime` should
    // only be `endOfTime` if no process was enabled.
    if (nextTaskTime != SimInstant::endOfTime()) {
        this->NextTaskTime = nextTaskTime.realNanos;
        this->nextProcPriority = nextTaskTime.causalPriority;
    }
}

void SimModel::resetSimulation() {
    this->CurrentNanos = 0;

    // Reset all processes, tasks, and modules
    //! @todo Should we skip resetting disabled tasks?
    for (auto &process : this->processList) {
        auto nextTaskNanos = std::numeric_limits<uint64_t>::max();
        for (auto &task : process->processTasks) {
            task.TaskPtr->nextUpdateNanos = task.TaskPtr->firstUpdateNanos;
            for (auto const &modelPair : task.TaskPtr->TaskModels) {
                modelPair.ModelPtr->reset(this->CurrentNanos);
            }

            nextTaskNanos = std::min(nextTaskNanos, task.TaskPtr->nextUpdateNanos);
        }

        process->nextTaskTime = nextTaskNanos;
    }

    // Figure out which process will update first
    auto nextTaskTime = SimInstant::endOfTime();
    for (auto &process : this->processList) {
        if (!process->enabled) { continue; }

        auto nextProcTime = SimInstant::atNanos(process->nextTaskTime).atPriority(process->processPriority);
        nextTaskTime = std::min(nextTaskTime, nextProcTime);
    }
    this->NextTaskTime = nextTaskTime.realNanos;
    this->nextProcPriority = nextTaskTime.causalPriority;
}
