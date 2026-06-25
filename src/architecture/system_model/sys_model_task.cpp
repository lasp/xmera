// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sys_model_task.h"

/*! A construction option that allows the user to set some task parameters.
 Note that the only required argument is InputPeriod.
 @param InputPeriod The amount of nanoseconds between calls to this Task.
 @param FirstStartTime The amount of time in nanoseconds to hold a task dormant before starting.
        After this time the task is executed at integer amounts of InputPeriod again
 */
SysModelTask::SysModelTask(uint64_t InputPeriod, uint64_t FirstStartTime)
    : NextStartTime(FirstStartTime), TaskPeriod(InputPeriod), FirstTaskTime(FirstStartTime) {}

/*! This method resets all of the models that have been added to the Task at the CurrentSimTime.
 * See sys_model_task.h for related method reset()
 @return void
 @param CurrentSimTime The time to start at after reset
*/
void SysModelTask::resetModels(uint64_t CurrentSimTime) {
    for (auto const& modelPair : this->TaskModels) {
        modelPair.ModelPtr->reset(CurrentSimTime);
    }
    this->NextStartTime = CurrentSimTime;
}

/*! This method executes all of the models on the Task during runtime.
 Then, it sets its NextStartTime appropriately.
 @return void
 @param currentSimNanos The current simulation time in [ns]
 */
void SysModelTask::executeModels(uint64_t currentSimNanos) {
    this->NextStartTime += this->TaskPeriod;

    if (!this->taskActive) { return; }

    for (auto &modelPair : this->TaskModels) { modelPair.ModelPtr->updateState(currentSimNanos); }
}

/*! This method adds a new model into the Task list.  Note that the Priority
 parameter is option as it defaults to -1 (lowest, latest)
 @return void
 @param NewModel The new model that we are adding to the Task
 @param Priority The selected priority of the model being added (highest goes first)
 */
void SysModelTask::addModel(SysModel* NewModel, int32_t Priority) {
    ModelPriorityPair LocalPair;

    //! - Set the local pair with the requested priority and mode
    LocalPair.CurrentModelPriority = Priority;
    LocalPair.ModelPtr = NewModel;

    //! - Loop through the ModelPair vector and if Priority is higher than next, insert
    for (auto ModelPair = this->TaskModels.begin(); ModelPair != this->TaskModels.end(); ModelPair++) {
        if (Priority > ModelPair->CurrentModelPriority) {
            this->TaskModels.insert(ModelPair, LocalPair);
            return;
        }
    }
    //! - If we make it to the end of the loop, this is lowest priority, put it at end
    this->TaskModels.push_back(LocalPair);
}

/*! This method changes the period of a given task over to the requested period.
   It attempts to keep the same offset relative to the original offset that
   was specified at task creation.
 @return void
 @param newPeriod The period that the task should run at going forward
 */
void SysModelTask::setPeriod(uint64_t newPeriod) {
    //! - If the requested time is above the min time, set the next time based on the previous time plus the new period
    if (this->NextStartTime > this->FirstTaskTime) {
        uint64_t newStartTime =
            (((this->NextStartTime - this->FirstTaskTime) / newPeriod) * newPeriod) + this->FirstTaskTime;
        if (newStartTime <= (this->NextStartTime - this->TaskPeriod)) {
            newStartTime += newPeriod;
        }
        this->NextStartTime = newStartTime;
    }
    //! - Otherwise, we just should keep the original requested first call time for the task
    else {
        this->NextStartTime = this->FirstTaskTime;
    }
    //! - Change the period of the task so that future calls will be based on the new period
    this->TaskPeriod = newPeriod;
}

uint64_t SysModelTask::getNextStartTime() const { return this->NextStartTime; }

uint64_t SysModelTask::getTaskPeriod() const { return this->TaskPeriod; }

uint64_t SysModelTask::getFirstTaskTime() const { return this->FirstTaskTime; }
