// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <architecture/system_model/sim_model.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>

using testing::ElementsAre;
using testing::Eq;
using testing::IsEmpty;

TEST(SimModel, getProcesses) {
    RecordProperty(
        "description",
        ("getProcesses() should reflect the priority order of the processes added"
         " to the simulation.")
    );

    auto process1 = SysProcess();
    process1.processPriority = 1;

    auto process2a = SysProcess();
    process2a.processPriority = 2;

    auto process2b = SysProcess();
    process2b.processPriority = 2;

    auto sim = SimModel();

    // `getProcesses()` is a live view of the processes in the sim.
    auto &processes = sim.getProcesses();

    // A newly-created simulation should have no processes.
    EXPECT_THAT(processes, IsEmpty());

    // If we add a process, there should be exactly that process.
    sim.addNewProcess(&process1);
    EXPECT_THAT(processes, ElementsAre(&process1));

    // If we add a higher-priority process, it should appear first.
    sim.addNewProcess(&process2a);
    EXPECT_THAT(processes, ElementsAre(&process2a, &process1));

    // If we add another process at the same priority, it should appear later.
    sim.addNewProcess(&process2b);
    EXPECT_THAT(processes, ElementsAre(&process2a, &process2b, &process1));

    // Resetting the simulation shouldn't change the collection of processes.
    sim.resetSimulation();
    EXPECT_THAT(processes, ElementsAre(&process2a, &process2b, &process1));

    // The collection of processes shouldn't change over the course of simulation.
    sim.singleStepProcesses();
    EXPECT_THAT(processes, ElementsAre(&process2a, &process2b, &process1));
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

    auto process1 = SysProcess();
    process1.processPriority = 1;
    process1.disable();

    auto process2 = SysProcess();
    process2.processPriority = 2;
    process2.disable();

    auto sim = SimModel();
    sim.addNewProcess(&process1);
    sim.addNewProcess(&process2);

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

    auto process1 = SysProcess();
    process1.processPriority = 1;
    process1.enable();

    auto process2 = SysProcess();
    process2.processPriority = 2;
    process2.enable();

    auto sim = SimModel();
    sim.addNewProcess(&process1);
    sim.addNewProcess(&process2);

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
