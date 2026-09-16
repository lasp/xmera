# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
import inspect
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

import numpy as np
import pytest
from xmera.architecture import messaging  # noqa: F401  registers the SWIG message types
from xmera.simulation import spacecraft
from xmera.utilities.MonteCarlo.Controller import (
    Controller,
    RetentionPolicy
)
from xmera.utilities.MonteCarlo.Dispersions import (
    UniformEulerAngleMRPDispersion,
    UniformDispersion,
    NormalVectorCartDispersion,
    OrbitalElementDispersion
)
from xmera.utilities.MonteCarlo._UnitTests.SimpleTestModule import SimpleTestModule
from xmera.utilities import (
    macros,
    SimulationBaseClass
)

NUMBER_OF_RUNS = 3
VERBOSE = True
PROCESSES = 2

retained_message_name = "spacecraftStateMsg"
retained_variable_name = "helloworldModule"
var1 = "v_BN_N"
var2 = "r_BN_N"
fun1 = ".GetTicker()"
disp1_name = 'TaskList[0].TaskModels[0].hub.sigma_BNInit'
disp2_name = 'TaskList[0].TaskModels[0].hub.omega_BN_BInit'
disp3_name = 'TaskList[0].TaskModels[0].hub.mHub'
disp4_name = 'TaskList[0].TaskModels[0].hub.r_BcB_B'
disp5_name = 'TaskList[0].TaskModels[0].hub.r_CN_NInit'
disp6_name = 'TaskList[0].TaskModels[0].hub.v_CN_NInit'

@pytest.fixture(scope="module")
def mc_data_directory(tmp_path_factory):
    """A directory outside the source tree, so that a stopped run leaves no files in the repository."""
    return str(tmp_path_factory.mktemp("mc_data"))


def my_creation_function():
    """ A function that returns a simulation. """
    sim = SimulationBaseClass.SimBaseClass()
    simulation_time_step = macros.sec2nano(1.0)

    sim_task_name = "simTask"
    dyn_process = sim.CreateNewProcess("process")
    dyn_process.addTask(sim.CreateNewTask(sim_task_name, simulation_time_step))

    # Initialize the spacecraft object and set its properties
    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "bskSat"
    sim.AddModelToTask(sim_task_name, sc_object)

    # Add SimpleTestModule so that the test can examine a getter function through AddVariableForMultiProcessLogging
    test_module = SimpleTestModule()
    test_module.modelTag = "helloworldModule"
    sim.AddModelToTask(sim_task_name, test_module)

    # Initialize the spacecraft states with the initialization variables
    sc_object.hub.r_CN_NInit = [0.0, 0.0, 0.0]
    sc_object.hub.v_CN_NInit = [1.0, 0.0, 0.0]

    simulation_time = macros.sec2nano(2.0)

    sim.msgRecList = {retained_message_name: sc_object.scStateOutMsg.recorder(macros.sec2nano(1.0))}
    sim.AddModelToTask(sim_task_name, sim.msgRecList[retained_message_name])

    sim.ConfigureStopTime(simulation_time)

    return sim


def my_execution_function(sim):
    """ A function that executes a simulation. """
    sim.InitializeSimulation()
    sim.ExecuteSimulation()


def my_data_callback(monte_carlo_data, retention_policy):
    data = np.array(monte_carlo_data["messages"][retained_message_name + ".r_BN_N"])
    return sum(data[:, 1:])


@pytest.fixture()
def monte_carlo_simulation(mc_data_directory):
    monte_carlo = Controller()
    monte_carlo.set_should_disperse_seeds(True)
    monte_carlo.set_execution_function(my_execution_function)
    monte_carlo.set_simulation_function(my_creation_function)
    monte_carlo.set_execution_count(NUMBER_OF_RUNS)
    monte_carlo.set_num_worker_processes(PROCESSES)
    monte_carlo.log_level = "INFO"
    monte_carlo.archive_dir = mc_data_directory

    # Add some dispersions
    disp_dict = {"mu": 0.3986004415E+15,
                 "a": ["normal", 42000 * 1E3, 2000 * 1E3],
                 "e": ["uniform", 0, 0.5],
                 "i": ["uniform", -80, 80],
                 "Omega": None,
                 "omega": ["uniform", 80, 90],
                 "f": ["uniform", 0, 359]}
    monte_carlo.add_dispersion(OrbitalElementDispersion(disp5_name, disp6_name, disp_dict))
    monte_carlo.add_dispersion(UniformEulerAngleMRPDispersion(disp1_name))
    monte_carlo.add_dispersion(NormalVectorCartDispersion(disp2_name, 0.0, 0.75 / 3.0 * np.pi / 180))
    monte_carlo.add_dispersion(UniformDispersion(disp3_name, ([1300.0 - 812.3, 1500.0 - 812.3])))
    monte_carlo.add_dispersion(
        NormalVectorCartDispersion(disp4_name, [0.0, 0.0, 1.0], [0.05 / 3.0, 0.05 / 3.0, 0.1 / 3.0]))

    # Add a retention policy
    retention_policy = RetentionPolicy()
    retention_policy.add_message_log(retained_message_name, [var1, var2])
    retention_policy.add_variable_log("helloworldModule.GetTicker()")
    retention_policy.add_variable_log("bskSat.totOrbEnergy")
    retention_policy.set_data_callback(my_data_callback)
    monte_carlo.add_retention_policy(retention_policy)

    return monte_carlo


@pytest.mark.slowtest
def test_monte_carlo_simulation(mc_data_directory, monte_carlo_simulation, show_plots):
    monte_carlo_simulation.log_level = "INFO"
    failures = monte_carlo_simulation.execute_simulations()
    assert len(failures) == 0, "No runs should fail"


@pytest.mark.slowtest
def test_run_initial_conditions(mc_data_directory, monte_carlo_simulation):
    _ = monte_carlo_simulation.execute_simulations()
    monte_carlo_controller = Controller.load(monte_carlo_simulation.mc_run_dir)
    monte_carlo_controller.archive_dir = mc_data_directory
    monte_carlo_controller.run_initial_conditions([0, 2], monte_carlo_simulation.ic_directory)


@pytest.mark.slowtest
def test_initial_parameters_dispersed(mc_data_directory, monte_carlo_simulation):
    _ = monte_carlo_simulation.execute_simulations()
    monte_carlo_loaded = Controller.load(monte_carlo_simulation.mc_run_dir)

    # Make sure that the runs saved the initial parameters and that the parameters are different between runs
    params1 = monte_carlo_loaded.get_parameters(NUMBER_OF_RUNS-1)
    params2 = monte_carlo_loaded.get_parameters(NUMBER_OF_RUNS-2)
    assert "TaskList[0].TaskModels[0].RNGSeed" in params1, "random number seed should be applied"
    for disp_name in [disp1_name, disp2_name, disp3_name, disp4_name]:
        assert disp_name in params1, "dispersion should be applied"
        # Assert that two different runs had different parameters.
        assert params1[disp_name] != params2[disp_name], "dispersion should be different in each run"


@pytest.mark.slowtest
def test_rerun_repeatability(mc_data_directory, monte_carlo_simulation):
    """
        Do the case again. The output must be the same if the controller disperses the
        random seeds from the same primary seed in the two MC batches.
    """
    _ = monte_carlo_simulation.execute_simulations()
    monte_carlo_loaded = Controller.load(monte_carlo_simulation.mc_run_dir)

    retained_data = monte_carlo_loaded.get_retained_data(NUMBER_OF_RUNS-1)

    old_output = retained_data["messages"][retained_message_name + ".r_BN_N"]

    failed = monte_carlo_loaded.re_run_cases([NUMBER_OF_RUNS-1])
    assert len(failed) == 0, "Should rerun case successfully"

    retained_data = monte_carlo_loaded.get_retained_data(NUMBER_OF_RUNS-1)
    new_output = retained_data["messages"][retained_message_name + ".r_BN_N"]
    for k1, v1 in enumerate(old_output):
        for k2, v2 in enumerate(v1):
            assert np.fabs(old_output[k1][k2] - new_output[k1][k2]) < .001, \
            "Outputs shouldn't change on runs if random seeds are same"


@pytest.fixture()
def monte_carlo_simulation_no_dispersions(mc_data_directory):
    monte_carlo = Controller()
    monte_carlo.set_should_disperse_seeds(True)
    monte_carlo.set_execution_function(my_execution_function)
    monte_carlo.set_simulation_function(my_creation_function)
    monte_carlo.set_execution_count(NUMBER_OF_RUNS)
    monte_carlo.set_num_worker_processes(PROCESSES)
    monte_carlo.log_level = "INFO"
    monte_carlo.archive_dir = mc_data_directory

    retention_policy = RetentionPolicy()
    retention_policy.add_message_log(retained_message_name, [var1, var2])
    retention_policy.add_variable_log("helloworldModule.GetTicker()")
    retention_policy.add_variable_log("bskSat.totOrbEnergy")
    monte_carlo.add_retention_policy(retention_policy)

    return monte_carlo

@pytest.mark.slowtest
def test_data_is_retained(mc_data_directory, monte_carlo_simulation_no_dispersions):
    _ = monte_carlo_simulation_no_dispersions.execute_simulations()
    monte_carlo_loaded = Controller.load(monte_carlo_simulation_no_dispersions.mc_run_dir)

    retained_data = monte_carlo_loaded.get_retained_data(NUMBER_OF_RUNS-1)
    assert retained_data is not None, "Retained data should be available after execution"

    assert "messages" in retained_data, "Retained data should retain messages"
    assert retained_message_name + ".r_BN_N" in retained_data["messages"], "Retained messages should exist"
    assert retained_message_name + ".v_BN_N" in retained_data["messages"], "Retained messages should exist"

    assert "variables" in retained_data, "Retained data should retain variables"
    assert retained_variable_name + ".GetTicker()" in retained_data["variables"], "Retained variables should exist"
