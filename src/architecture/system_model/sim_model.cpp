// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sim_model.h"

#include <algorithm>

inline uint64_t SysModelTask::projectToCurrentSchedule(uint64_t time) {
    if (time <= this->firstUpdateNanos) { return this->firstUpdateNanos; }

    // Judge everything relative to the origin, `firstUpdateNanos`.
    time -= this->firstUpdateNanos;

    // Truncate `time` to the nearest mutiple of `updatePeriodNanos`.
    time = (time / this->updatePeriodNanos) * this->updatePeriodNanos;

    // Re-add the offset against `firstUpdateNanos`.
    time += this->firstUpdateNanos;

    return time;
}

void SysModelTask::addModel(SysModel* module, int32_t priority) {
    // Find the index separating lower priorities from higher priorities.
    auto it = this->TaskModels.begin();
    for (; it != this->TaskModels.end(); ++it) {
        if (priority > it->CurrentModelPriority) { break; }
    }

    // Insert the module at this index. (It's okay if it's the end() iterator.)
    this->TaskModels.insert(it, {.CurrentModelPriority = priority, .ModelPtr = module});
}

void SysModelTask::setPeriod(uint64_t updatePeriodNanos) {
    if (this->nextUpdateNanos > this->firstUpdateNanos) {
        uint64_t lastUpdateTime = this->nextUpdateNanos - this->updatePeriodNanos;

        this->updatePeriodNanos = updatePeriodNanos;
        this->nextUpdateNanos = this->projectToCurrentSchedule(this->nextUpdateNanos);
        if (this->nextUpdateNanos <= lastUpdateTime) { this->nextUpdateNanos += updatePeriodNanos; }
    } else {
        // If we haven't yet performed an update, our next update time is *still*
        // our first update time.
        this->updatePeriodNanos = updatePeriodNanos;
    }

    // Mark the heap for re-heaping according to the task's new update time.
    this->owner.isHeap = false;
}


SysModelTask &SysProcess::addTask(uint64_t updatePeriodNanos, uint64_t firstUpdateNanos, int32_t priority) {
    auto task_id = this->processTasks.size();
    auto &task = this->processTasks.emplace_back(std::make_unique<SysModelTask>(
        SysModelTask::Passkey{},
        this->owner,
        updatePeriodNanos,
        firstUpdateNanos,
        priority
    ));

    // Mark the heap for re-heaping according to the new task's update time.
    this->owner.isHeap = false;
    this->owner.jobHeap.push_back({
        .process = this,
        .process_id = this->processId,
        .task = task.get(),
        .task_id = task_id,
    });

    return *task.get();
}

bool SysProcess::changeTaskPeriod(std::string const &taskName, uint64_t newPeriod) {
    for (auto &entry : this->processTasks) {
        if (entry->TaskName != taskName) { continue; }

        entry->setPeriod(newPeriod);

        return true;
    }

    return false;
}


void SimModel::ensureHeap() const {
    if (this->isHeap) { return; }

    std::make_heap(this->jobHeap.begin(), this->jobHeap.end());
    this->isHeap = true;
}

SysProcess &SimModel::addNewProcess(std::string name, int64_t priority) {
    // Find the index separating lower priorities from higher priorities.
    auto it = this->processList.begin();
    for (; it != this->processList.end(); ++it) {
        if (priority > (*it)->processPriority) { break; }
    }

    auto processId = this->processList.size();
    it = this->processList.emplace(it, std::make_unique<SysProcess>(SysProcess::Passkey{}, *this, processId, name, priority));

    return *it->get();
}

void SimModel::resetSimulation() {
    this->CurrentNanos = 0;

    // Reset all processes, tasks, and modules.
    for (auto &process : this->processList) {
        for (auto &task : process->processTasks) {
            task->nextUpdateNanos = task->firstUpdateNanos;
            for (auto const &modelPair : task->TaskModels) {
                modelPair.ModelPtr->reset(this->CurrentNanos);
            }
        }
    }

    // Mark the heap for re-heaping according to the tasks' new update times.
    this->isHeap = false;
}

void SimModel::stepUntilStop(uint64_t stopNanos, int64_t stopPriority) {
    if (this->jobHeap.empty()) { return; }

    // We reserve UINT64_MAX as the "end of time" sentinel.
    // Hence, UINT64_MAX - 1 is the last accessible simulation instant.
    if (stopNanos == std::numeric_limits<uint64_t>::max()) {
        stopNanos -= 1;
        stopPriority = std::numeric_limits<int64_t>::min();
    }

    // If reprioritizations or other heap-invalidating modifications have occurred, re-heap the heap.
    // Delaying heapification to this point allows us to amortize the impact of a series of independent,
    // uncorrelated modifications by re-heaping just once before we require the heap property again.
    this->ensureHeap();

    // Pump the job queue until the next job is beyond our stopping threshold.
    while (this->jobHeap.front().noLaterThan(stopNanos, stopPriority)) {
        // Extract the front element from the heap (moving it to the back).
        std::pop_heap(this->jobHeap.begin(), this->jobHeap.end());
        auto job = this->jobHeap.back();

        // Advance the simulation clock to the time of this job.
        this->CurrentNanos = job.task->nextUpdateNanos;

        // Re-schedule the job in the future (using saturating addition).
        job.task->nextUpdateNanos += job.task->updatePeriodNanos;
        if (job.task->nextUpdateNanos < job.task->updatePeriodNanos) {
            job.task->nextUpdateNanos = std::numeric_limits<uint64_t>::max();
        }
        std::push_heap(this->jobHeap.begin(), this->jobHeap.end());

        // Execute the job.
        if (job.process->enabled && job.task->taskActive) {
            for (auto &modelPair : job.task->TaskModels) {
                modelPair.ModelPtr->updateState(this->CurrentNanos);
            }
        }

        // On the off chance that the task mucked with task timings,
        // make sure the heap is still a heap.
        this->ensureHeap();
    }
}
