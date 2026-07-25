// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <architecture/system_model/sim_model.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>

using testing::ElementsAre;
using testing::Eq;
using testing::IsEmpty;
using testing::Pointer;

TEST(SimModel, getProcesses) {
    RecordProperty(
        "description",
        ("getProcesses() should reflect the priority order of the processes added"
         " to the simulation.")
    );

    auto sim = SimModel();

    // `getProcesses()` is a live view of the processes in the sim.
    auto &processes = sim.getProcesses();

    // A newly-created simulation should have no processes.
    EXPECT_THAT(processes, IsEmpty());

    // If we add a process, there should be exactly that process.
    auto &process1 = sim.addNewProcess("", 1);
    EXPECT_THAT(processes, ElementsAre(Pointer(&process1)));

    // If we add a higher-priority process, it should appear first.
    auto &process2a = sim.addNewProcess("", 2);
    EXPECT_THAT(processes, ElementsAre(Pointer(&process2a), Pointer(&process1)));

    // If we add another process at the same priority, it should appear later.
    auto &process2b = sim.addNewProcess("", 2);
    EXPECT_THAT(processes, ElementsAre(Pointer(&process2a), Pointer(&process2b), Pointer(&process1)));

    // Resetting the simulation shouldn't change the collection of processes.
    sim.resetSimulation();
    EXPECT_THAT(processes, ElementsAre(Pointer(&process2a), Pointer(&process2b), Pointer(&process1)));

    // The collection of processes shouldn't change over the course of simulation.
    sim.singleStepProcesses();
    EXPECT_THAT(processes, ElementsAre(Pointer(&process2a), Pointer(&process2b), Pointer(&process1)));
}

TEST(SimModel, taskTimings_withEmptySim) {
    RecordProperty(
        "description",
        ("On an empty simulation, the task timings should jump from 0 to the latest"
         " possible time.")
    );

    auto sim = SimModel();

    // Immediately after a reset, the "next" task should occur "at the end of time".
    // (Why? Well, the sim never has to update. UINT_MAX and INT_MIN are our best
    // sentinels for indicating that nothing ever happens.)
    sim.resetSimulation();
    EXPECT_THAT(sim.getCurrentNanos(), Eq(0));
    EXPECT_THAT(sim.getNextTaskTime(), Eq(std::numeric_limits<uint64_t>::max()));
    EXPECT_THAT(sim.getNextProcPriority(), Eq(std::numeric_limits<int64_t>::min()));

    // After stepping, the "next" task should still occur "at the end of time".
    // (What else could it be?)
    sim.singleStepProcesses();
    EXPECT_THAT(sim.getCurrentNanos(), Eq(std::numeric_limits<uint64_t>::max()));
    EXPECT_THAT(sim.getNextTaskTime(), Eq(std::numeric_limits<uint64_t>::max()));
    EXPECT_THAT(sim.getNextProcPriority(), Eq(std::numeric_limits<int64_t>::min()));
}

TEST(SimModel, taskTimings_withDisabledProcesses) {
    RecordProperty(
        "description",
        ("On a simulation with only disabled processes, the task timings should"
         " jump from 0 to the latest possible time.")
    );

    auto sim = SimModel();
    sim.addNewProcess("", 1).disable();
    sim.addNewProcess("", 2).disable();

    // Immediately after a reset, the "next" task should occur "at the end of time".
    // (Why? Well, the sim never has to update. UINT_MAX and INT_MIN are our best
    // sentinels for indicating that nothing ever happens.)
    sim.resetSimulation();
    EXPECT_THAT(sim.getCurrentNanos(), Eq(0));
    EXPECT_THAT(sim.getNextTaskTime(), Eq(std::numeric_limits<uint64_t>::max()));
    EXPECT_THAT(sim.getNextProcPriority(), Eq(std::numeric_limits<int64_t>::min()));

    // After stepping, the "next" task should still occur "at the end of time".
    // (What else could it be?)
    sim.singleStepProcesses();
    EXPECT_THAT(sim.getCurrentNanos(), Eq(std::numeric_limits<uint64_t>::max()));
    EXPECT_THAT(sim.getNextTaskTime(), Eq(std::numeric_limits<uint64_t>::max()));
    EXPECT_THAT(sim.getNextProcPriority(), Eq(std::numeric_limits<int64_t>::min()));
}

TEST(SimModel, taskTimings_withEmptyProcesses) {
    RecordProperty(
        "description",
        ("On a simulation with empty processes, the task timings should jump from"
         " 0 to the latest possible time.")
    );

    auto sim = SimModel();
    auto &process1 [[maybe_unused]] = sim.addNewProcess("", 1);
    auto &process2 = sim.addNewProcess("", 2);

    // Immediately after a reset, the "next" task should occur "at the end of time".
    // (Nevertheless, the next priority is that of the highest-priority process.)
    sim.resetSimulation();
    EXPECT_THAT(sim.getCurrentNanos(), Eq(0));
    EXPECT_THAT(sim.getNextTaskTime(), Eq(process2.getNextTaskTime()));
    EXPECT_THAT(sim.getNextProcPriority(), Eq(process2.processPriority));

    // After stepping, the "next" task should still occur "at the end of time".
    // (What else could it be?)
    sim.singleStepProcesses();
    EXPECT_THAT(sim.getCurrentNanos(), Eq(process2.getNextTaskTime()));
    EXPECT_THAT(sim.getNextTaskTime(), Eq(process2.getNextTaskTime()));
    EXPECT_THAT(sim.getNextProcPriority(), Eq(process2.processPriority));
}
