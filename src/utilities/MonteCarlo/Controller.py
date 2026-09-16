# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import multiprocessing.queues
import os
import random
import shutil
import traceback
import warnings

from typing import Generator

with warnings.catch_warnings():
    warnings.simplefilter("ignore", category=DeprecationWarning)

import glob
import gzip
import json
import signal
import time
import numpy as np
import multiprocessing as mp
import pickle as pickle
from xmera.utilities.MonteCarlo.DataWriter import DataWriter
from xmera.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy
from xmera.utilities.simulationProgessBar import SimulationProgressBar


class SimulationParameters:
    """
    Run parameters for one simulation in a Monte Carlo batch.
    """

    def __init__(self,
                 creation_function,
                 execution_function,
                 configure_function,
                 retention_policies,
                 dispersions,
                 should_disperse_seeds,
                 results_filename,
                 initial_conditions_filename,
                 magnitudes_filename,
                 index=None):
        self.index = index
        self.creation_function = creation_function
        self.execution_function = execution_function
        self.configure_function = configure_function
        self.retention_policies = retention_policies
        self.dispersions = dispersions
        self.should_disperse_seeds = should_disperse_seeds
        self.results_filename = results_filename
        self.initial_conditions_filename = initial_conditions_filename
        self.magnitudes_filename = magnitudes_filename
        self.verbose = False
        self.modifications = {}
        self.dispersion_mag = {}
        self.should_save_disp_mag = False
        self.show_progress_bar = False


class Controller:
    """
    The Monte Carlo controller executes many runs of a simulation with different initial parameters.
    The controller keeps the data from each run for analysis at a later time.
    """

    def __init__(self):
        self.should_save_disp_mag = None
        self.should_disperse_seeds = False
        self.num_simulation_runs = 0
        self.var_cast = None
        self.num_processes = mp.cpu_count()
        self.verbose = False
        self.show_progress_bar = False
        self.creation_function=None
        self.execution_function=None
        self.configure_function=None
        self.retention_policies=[]
        self.dispersions=[]
        self.multi_proc_manager = None
        self.data_out_queue = None
        self.data_writer = None
        self._ic_directory = ""
        self._archive_dir = ""
        self._mc_run_dir = None
        self._results_dir = None

    def set_show_progress_bar(self, value: bool) -> None:
        """
        Enable or disable the progress bar that shows the progress of the simulation.
        :param value: True shows the progress bar. False does not show it.
        :type value: bool
        """
        self.show_progress_bar = value

    @staticmethod
    def latest_run_dir(archive_dir: str) -> str:
        """
        Find the most recent run directory inside an archive directory.

        Each execution writes into its own ``mc_run_<timestamp>`` directory in the archive
        directory. A caller that knows only the archive directory uses this method to find the
        data of the most recent execution.

        :param archive_dir: The archive directory that the controller used for the execution.
        :type archive_dir: str

        :return: The path of the most recently created run directory.
        :rtype: str
        """
        candidates = glob.glob(os.path.join(archive_dir, "mc_run_*"))
        candidates = [c for c in candidates if os.path.isdir(c)]
        if not candidates:
            raise FileNotFoundError(f"No mc_run_* directory found in {archive_dir}")
        return max(candidates, key=lambda c: (os.path.getmtime(c), c))

    @staticmethod
    def load(run_directory: str):
        """
        Load a completed Monte Carlo batch.
        :param run_directory: The path to the directory that contains the archived Monte Carlo file MonteCarlo.data
        :type run_directory: str
        """
        filename = os.path.abspath(run_directory) + "/MonteCarlo.data"

        with gzip.open(filename) as pickled_data:
            data = pickle.load(pickled_data)
            if data.verbose:
                print("Loading montecarlo at", filename)
            data.multi_proc_manager = mp.Manager()
            data.data_out_queue = data.multi_proc_manager.Queue()
            data.data_writer = DataWriter(data.data_out_queue)
            data.data_writer.daemon = False
            return data

    def set_execution_function(self, execution_function):
        """
        Set an execution function that executes a simulation instance.

        :param execution_function: A function with one parameter, a simulation instance. In each run, the controller
        calls this function after the creation_function and the configurationFunction. The function must execute the
        simulation. The controller ignores its return value.
        :type execution_function: type.function(SimulationBaseClass) -> None
        """
        self.execution_function = execution_function

    def set_configure_function(self, configure_function):
        """
        Set a configure function that configures a simulation instance.

        :param configure_function: A function with one parameter, a simulation instance. In each run, the controller
        calls this function after the creation_function and before it applies the dispersions. The controller ignores
        its return value.
        :type configure_function: type.function(SimulationBaseClass) -> None
        """
        self.configure_function = configure_function

    def set_simulation_function(self, simulation_function):
        """
        Set the function that creates the simulation instance.

        :param simulation_function: A function with no parameters that returns a simulation instance.
        :type simulation_function: type.function() -> None
        """
        self.creation_function = simulation_function

    def set_should_disperse_seeds(self, seed_disp: bool):
        """
        Set whether the controller disperses the RNG seeds of each run in the Monte Carlo batch.

        :param seed_disp: True disperses the RNG seeds in each run of the simulation.
        :type seed_disp: bool
        """
        self.should_disperse_seeds = seed_disp

    def set_execution_count(self, num_runs: int):
        """
        Set the number of runs in the Monte Carlo batch.

        :param num_runs: The number of runs in the Monte Carlo batch.
        :type num_runs: int
        """
        self.num_simulation_runs = num_runs

    def add_dispersion(self, disp):
        """
        Add a dispersion to the simulation.

        :param disp: The dispersion to add to the simulation.
        :type disp: Dispersion

        """
        self.dispersions.append(disp)

    def add_retention_policy(self, policy):
        """
        Add a retention policy to the simulation.

        :param policy: The retention policy to add to the simulation. It gives the variables that the controller logs
            and saves.
        :type policy: RetentionPolicy
        """
        self.retention_policies.append(policy)

    def set_num_worker_processes(self, num_processes: int):
        """
        Set the number of worker processes for the Monte Carlo batch.

        :param num_processes: The number of worker processes that execute the runs.
        :type num_processes: int
        """
        self.num_processes = num_processes

    def set_verbose(self, verbose):
        """
        Use verbose output for this MonteCarlo run

        :param verbose: Whether to print verbose information during this MonteCarlo sim.
        :type verbose: bool
        """
        self.verbose = verbose

    def set_disp_magnitude_file(self, magnitudes):
        """
        Set whether each run saves a .txt file with the magnitude of each dispersion.

        The file gives each magnitude as a percent or as a number of sigma from the mean.

        :param magnitudes: True saves these additional files for analysis.
        :type magnitudes: bool
        """
        self.should_save_disp_mag = magnitudes

    def set_var_cast(self, var_cast: str):
        """
        Set the type that the data writer casts double values to.

        :param var_cast: 'float', 'integer', 'signed', or 'unsigned'. Refer to the pandas.to_numeric documentation.
        :type var_cast: str
        """
        self.var_cast = var_cast

    @property
    def mc_run_dir(self):
        return self._mc_run_dir

    @property
    def results_dir(self):
        return self._results_dir

    @property
    def ic_directory(self):
        return self._ic_directory

    @property
    def archive_dir(self):
        return self._archive_dir

    @archive_dir.setter
    def archive_dir(self, directory: str):
        """
        Set the archive directory for this Monte Carlo batch.

        :param directory: The path of the directory that will contain the Monte Carlo data root directory.
        :type directory: str
        """
        self._archive_dir = os.path.abspath(directory)

    def _setup_archive_directory(self):
        """
        Make the directory structure for the data of this Monte Carlo batch.
        The directory structure is:
            - root directory (_archive_dir)
            -- mc run directory with timestamp (_mc_run_dir)
            --- initial_conditions
            --- results
        """
        if not self._archive_dir:
            return

        os.makedirs(self._archive_dir, exist_ok=True)

        # The timestamp has a resolution of one second. Thus two batches that start in the same
        # second get the same name. Keep the readable name and add a counter if the name is in use.
        directory_id = time.strftime("%Y%m%d-%H%M%S")
        attempt = 0
        while True:
            suffix = "" if attempt == 0 else f"_{attempt}"
            candidate = os.path.join(self._archive_dir, f"mc_run_{directory_id}{suffix}")
            try:
                os.mkdir(candidate)
                break
            except FileExistsError:
                attempt += 1
        self._mc_run_dir = candidate

        self._ic_directory = os.path.join(self._mc_run_dir, "initial_conditions")
        os.mkdir(self._ic_directory)

        self._results_dir = os.path.join(self._mc_run_dir, "results")
        os.mkdir(self._results_dir)

    def _make_results_directory_file_name(self, index):
        return os.path.join(self._results_dir, "run" + str(index) + ".data")

    def _make_dispersion_magnitudes_file_name(self, index):
        return os.path.join(self._results_dir, "run" + str(index) + "mag.txt")

    def _make_initial_conditions_directory_file_name(self, index):
        return os.path.join(self._ic_directory, "run" + str(index) + ".json")

    def get_retained_data(self, case: int):
        """
        Get the retained data of one run.

        :param case: The run to get the data from.
        :type case: int

        :return The retained data.
        :rtype: list
        """
        results_data_file = self._make_results_directory_file_name(case)

        with gzip.open(results_data_file) as pickled_data:
            data = pickle.load(pickled_data)
            return data

    def get_retained_datas(self, cases: list[int]):
        """
        Get the retained data of a list of runs.

        :param cases: The run_indexes to get the data from.
        :type cases: list[int]

        :return A generator that yields the retained data of each of these run_indexes, in the given order
        """
        for case in cases:
            yield self.get_retained_data(case)  # call this method recursively, yielding the result

    def get_parameters(self, run_index):
        """
        Get the parameters of one run of the Monte Carlo batch.

        :param run_index: The number of the run.
        :type run_index: int

        :return: A dictionary of the parameters of the simulation.
                 For example:
                 {"keyForSim": parameterValue, 'TaskList[0].TaskModels[0].RNGSeed': 1674764759}
        """
        filename = self._make_initial_conditions_directory_file_name(run_index)

        with open(filename, "r") as dispersion_file:
            dispersions = json.load(dispersion_file)
            return dispersions

    def re_run_cases(self, run_indexes: list[int]):
        """
        Rerun selected run indexes from a Monte Carlo batch. The reruns do not occur in parallel.

        :param run_indexes: The list of runs to do again, a list of numbers.
        :type run_indexes: list[int]

        :return: failed The list of failed runs.
        :rtype: list[int]
        """
        # the list of failures
        failed = []

        for run_index in run_indexes:
            if self.verbose:
                print("Rerunning", run_index)

            old_run_file = self._make_initial_conditions_directory_file_name(run_index)
            if not os.path.exists(old_run_file):
                print(f"File {old_run_file} not found. Therefore, cannot re-run case: {run_index}")
                continue

            # use old simulation parameters, modified slightly.
            sim_params = self.create_sim_parameters(run_index)
            sim_params.index = run_index
            # don't redisperse seeds, we want to use the ones saved in the old_run_file
            sim_params.should_disperse_seeds = False
            # don't retain any data so remove all retention policies
            sim_params.retention_policies = []

            with open(old_run_file, "r") as run_parameters:
                sim_params.modifications = json.load(run_parameters)

            # execute simulation with dispersion
            executor = SimulationExecutor()
            success = executor((sim_params, self.data_out_queue))

            if not success:
                print("Error re-executing run", run_index)
                failed.append(run_index)

        if len(failed) > 0:
            failed.sort()
            print("Failed rerunning run_indexes:", failed)

        return failed

    def run_initial_conditions(self, run_indexes, ic_directory):
        """
        Run the initial conditions of selected run indexes.

        :param run_indexes: The list of runs to do again, a list of numbers.
        :type run_indexes: int[]
        :param ic_directory: The directory that contains the initial conditions data files.
        :type ic_directory: str

        :return: failed: The list of failed runs.
        :rtype: list
        """
        # the list of failures
        failed = []

        assert ic_directory != "", "No initial condition directory was given"

        if self.verbose:
            print("Beginning simulation with {0} runs on {1} processes".format(self.num_simulation_runs,
                                                                               self.num_processes))
        self._setup_archive_directory()

        # Copy IC files into new MC directory
        file_paths = [os.path.join(ic_directory, "run" + str(case) + ".json") for case in run_indexes]
        destination_file_paths = [self._make_initial_conditions_directory_file_name(case) for case in run_indexes]
        [shutil.copyfile(src, dst) for src, dst in zip(file_paths, destination_file_paths)]

        self._save_monte_carlo_controller()

        # Create Queue, but don't ever start it.
        self.multi_proc_manager = mp.Manager()
        self.data_out_queue = self.multi_proc_manager.Queue()
        self.data_writer = DataWriter(self.data_out_queue)
        self.data_writer.daemon = False

        self.data_writer.set_log_dir(self.results_dir)
        self.data_writer.start()

        jobs_finished = 0  # keep track of what simulations have finished

        # The simulation executor is responsible for executing simulation given a simulation's parameters
        # It is called within worker processes with each worker's simulation parameters
        simulation_executor = SimulationExecutor()

        progress_bar = SimulationProgressBar(len(run_indexes), self.show_progress_bar)
        if self.num_processes == 1:
            if self.verbose:
                print("Executing sequentially...")
            i = 0
            for i in range(len(run_indexes)):
                sim_generator = self.generate_ic_sims(run_indexes[i:i + 1])
                for sim in sim_generator:
                    try:
                        simulation_executor((sim, self.data_out_queue))
                    except:
                        failed.append(i)
                i += 1
                progress_bar.update(i)
        else:
            num_sims = len(run_indexes)
            if self.num_processes > num_sims:
                print("Fewer MCs spawned than processes assigned (%d < %d). Changing processes count to %d." % (num_sims, self.num_processes, num_sims))
                self.num_processes = num_sims
            for i in range(num_sims//self.num_processes):
                # If number of sims doesn't factor evenly into the number of processes:
                if num_sims % self.num_processes != 0 and i == len(list(range(num_sims // self.num_processes)))-1:
                    offset = num_sims % self.num_processes
                else:
                    offset = 0

                sim_generator = self.generate_ic_sims(run_indexes[self.num_processes * i:self.num_processes * (i + 1) + offset])
                pool = mp.Pool(self.num_processes)
                try:
                    # yields results *as* the workers finish jobs
                    for result in pool.imap_unordered(simulation_executor, [(x, self.data_out_queue) for x in sim_generator]):
                        if result[0] is not True:  # workers return True on success
                            failed.append(result[1])  # add failed jobs to the list of failures
                            print("Job", result[1], "failed...")

                        jobs_finished += 1
                        progress_bar.update(jobs_finished)
                    pool.close()
                except KeyboardInterrupt as e:
                    print("Ctrl-C was hit, closing pool")
                    # failed.extend(range(jobs_finished, num_sims))  # fail all potentially running jobs...
                    pool.terminate()
                    raise e
                except Exception as e:
                    print("Unknown exception while running simulations:", e)
                    # failed.extend(range(jobs_finished, num_sims))  # fail all potentially running jobs...
                    traceback.print_exc()
                    pool.terminate()
                finally:
                    pool.join()

        progress_bar.markComplete()
        progress_bar.close()
        while not self.data_out_queue.empty():
           time.sleep(1)
        self.data_out_queue.put((None, None, True))
        time.sleep(5)

        self._save_failed_indexes(failed)

        return failed

    def generate_ic_sims(self, run_indexes: list[int]) -> Generator[SimulationParameters, None, None]:
        """
        Generator function that clones a baseSimulation for an initial conditions run.

        :param run_indexes: The run indexes. The generator makes simulation parameters from the saved
            initial conditions file of each run index.
        :type run_indexes: list[int]

        :return sim_params: A generator that yields that number of cloned simulations
        :rtype: sim_params: Generator[SimulationParameters]
        """

        # make a list of simulations to execute by cloning the base-simulation and
        # changing each clone's index and filename to make a list of
        # simulations to execute
        # use old simulation parameters, modified slightly.
        for run_index in run_indexes:
            sim_params = self.create_sim_parameters(run_index)
            sim_params.index = run_index
            # don't redisperse seeds, we want to use the ones saved in the old_run_file
            sim_params.should_disperse_seeds = False

            sim_params.initial_conditions_filename = self._make_initial_conditions_directory_file_name(run_index)
            with open(sim_params.initial_conditions_filename, "r") as run_parameters:
                sim_params.modifications = json.load(run_parameters)

            yield sim_params

    def create_sim_parameters(self, index: int) -> SimulationParameters:
        """
        Create a simulation job definition.

        :param index: The index of the simulation job
        :type index: int

        :return sim_params: A simulation parameter job definition
        :rtype: sim_params: SimulationParameters
        """
        sim_params = SimulationParameters(self.creation_function,
                                          self.execution_function,
                                          self.configure_function,
                                          self.retention_policies,
                                          self.dispersions,
                                          self.should_disperse_seeds,
                                          self._make_results_directory_file_name(index),
                                          self._make_initial_conditions_directory_file_name(index),
                                          self._make_dispersion_magnitudes_file_name(index),
                                          index)
        sim_params.verbose = self.verbose
        sim_params.show_progress_bar = self.show_progress_bar
        sim_params.should_save_disp_mag = self.should_save_disp_mag
        return sim_params

    def generate_sims(self, sim_run_indexes: list[int]) -> Generator[SimulationParameters, None, None]:
        """
        Generator function that clones a baseSimulation.

        :param sim_run_indexes: The run indexes to make simulation parameters for
        :type sim_run_indexes: list[int]

        :return sim_params: A generator that yields that number of simulations
        :rtype: sim_params: Generator[SimulationParameters]
        """

        # make a list of simulations to execute by cloning the base-simulation and
        # changing each clone's index and filename to make a list of
        # simulations to execute
        for run_index in sim_run_indexes:
            sim_params = self.create_sim_parameters(run_index)
            sim_params.index = run_index

            yield sim_params

    def execute_callbacks(self, run_indexes=None, retention_policies=[]):
        """
        Execute the retention policy callbacks after a Monte Carlo batch.

        :param run_indexes: The simulations to execute the callbacks on.
        :type run_indexes: list[int]
        :param retention_policies: The retention policies to execute.
        :type retention_policies: list[RetentionPolicy]
        """

        if run_indexes is None:
            run_indexes = list(range(self.num_simulation_runs))

        if not retention_policies:
            retention_policies = self.retention_policies

        for index in run_indexes:
            data = self.get_retained_data(index)
            for retention_policy in retention_policies:
                retention_policy.execute_callback(data)

    def _save_failed_indexes(self, failed):
        """
        Save a list of failed simulation run indexes.

        :param failed: The list of failed simulation runs.
        :type failed: list[int]
        """
        if len(failed) == 0: return

        if self.verbose:
            print("Failed", failed, "saving to 'failures.txt'")
        failed.sort()
        # write a file that contains log of failed runs
        with open(os.path.join(self._mc_run_dir, "failures.txt"), "w") as fail_file:
            fail_file.write(str(failed))

    def _save_monte_carlo_controller(self):
        """
        Save a serialized copy of the Monte Carlo controller.
        """
        if self.verbose:
            print("Archiving a copy of this simulation before running it in 'MonteCarlo.data'")
        try:
            with gzip.open(os.path.join(self._mc_run_dir, "MonteCarlo.data"), "wb") as pickleFile:
                pickle.dump(self, pickleFile)  # dump this controller object into a file.
        except Exception as e:
            print("Unknown exception while trying to pickle monte-carlo-controller... \ncontinuing...\n\n", e)

    def execute_simulations(self) -> list[int]:
        """
        Execute the simulation runs.

        :return: failed: A list of the indices of all failed simulation runs.
        :rtype: list[int]
        """

        if self.verbose:
            print("Beginning simulation with {0} runs on {1} processes".format(self.num_simulation_runs,
                                                                               self.num_processes))
        self._setup_archive_directory()
        self._save_monte_carlo_controller()

        self.multi_proc_manager = mp.Manager()
        self.data_out_queue = self.multi_proc_manager.Queue()
        self.data_writer = DataWriter(self.data_out_queue)
        self.data_writer.daemon = False

        num_sims = self.num_simulation_runs

        # start data writer process
        self.data_writer.set_log_dir(self.results_dir)
        self.data_writer.set_var_cast(self.var_cast)
        self.data_writer.start()

        # Avoid building a full list of all simulations to run in memory,
        # instead only generating simulations right before they are needed by a waiting worker
        # This is accomplished using a generator and pool.imap, -- simulations are only built
        # when they are about to be passed to a worker, avoiding memory overhead of first building simulations
        # There is a system-dependent chunking behavior, sometimes 10-20 are generated at a time.
        # sim_generator = self.generateSims(range(num_sims))
        failed = []  # keep track of the indices of failed simulations
        jobs_finished = 0  # keep track of what simulations have finished

        # The simulation executor is responsible for executing simulation given a simulation's parameters
        # It is called within worker processes with each worker's simulation parameters
        simulation_executor = SimulationExecutor()

        progress_bar = SimulationProgressBar(num_sims, self.show_progress_bar)

        # The outermost for-loop for both the serial and multiprocessed sim generator is not necessary. It
        # is a temporary fix to a memory leak which is assumed to be a result of the sim_generator not collecting
        # garbage properly. # TODO: Find a more permenant solution to the leak.

        if self.num_processes == 1:
            if self.verbose:
                print("Executing sequentially...")
            i = 0
            for i in range(num_sims):
                sim_generator = self.generate_sims(list(range(i, i + 1)))
                for sim in sim_generator:
                    try:
                        run_ok = simulation_executor((sim, self.data_out_queue))[0]
                    except:
                        failed.append(i)
                    else:
                        if not run_ok:
                            failed.append(i)
                    i += 1
                    progress_bar.update(i)
        else:
            if self.num_processes > num_sims:
                print("Fewer MCs spawned than processes assigned (%d < %d). Changing processes count to %d." % (num_sims, self.num_processes, num_sims))
                self.num_processes = num_sims
            for i in range(num_sims//self.num_processes):
                # If number of sims doesn't factor evenly into the number of processes:
                if num_sims % self.num_processes != 0 and i == len(list(range(num_sims // self.num_processes)))-1:
                    offset = num_sims % self.num_processes
                else:
                    offset = 0
                sim_generator = self.generate_sims(list(range(self.num_processes * i, self.num_processes * (i + 1) + offset)))
                pool = mp.Pool(self.num_processes)
                try:
                    # yields results *as* the workers finish jobs
                    for result in pool.imap_unordered(simulation_executor, [(x, self.data_out_queue) for x in sim_generator]):
                        if result[0] is not True:  # workers return True on success
                            failed.append(result[1])  # add failed jobs to the list of failures
                            print("Job", result[1], "failed...")

                        jobs_finished += 1
                        progress_bar.update(jobs_finished)
                    pool.close()
                except KeyboardInterrupt as e:
                    print("Ctrl-C was hit, closing pool")
                    failed.extend(list(range(jobs_finished, num_sims)))  # fail all potentially running jobs...
                    pool.terminate()
                    raise e
                except Exception as e:
                    print("Unknown exception while running simulations:", e)
                    failed.extend(list(range(jobs_finished, num_sims)))  # fail all potentially running jobs...
                    traceback.print_exc()
                    pool.terminate()
                finally:
                    # Wait until all data is logged from the spawned runs before proceeding with the next set.
                    pool.join()

        progress_bar.markComplete()
        progress_bar.close()
        # Wait until all data logging is finished before concatenation dataframes and shutting down the pool
        while not self.data_out_queue.empty():
           time.sleep(1)
        self.data_out_queue.put((None, None, True))
        time.sleep(5)

        self._save_failed_indexes(failed)

        return failed


class SimulationExecutor:
    """
    This class executes a simulation in a worker process.
    To use it, create an instance of this class. Then call the instance with the simulation parameters::

        executor = SimulationExecutor()
        sim_params = SimulationParameters()
        successFlag = executor(sim_params)

    To execute a simulation in a different process, use this class as the target of that process.
    """

    @classmethod
    def __call__(cls, params: tuple[SimulationParameters, multiprocessing.Queue]) -> tuple[bool, int]:
        """
        In each worker process, we execute this function (by calling this object)

        :param params: The SimulationParameters object of the simulation to execute, and the output data queue of
        the data writer.
        :type params: tuple[SimulationParameters, multiprocessing.Queue]

        :return success: A pair of the simulation run's success and run index
        :rtype: tuple[bool, int]
        """
        sim_params = params[0]
        data_out_queue = params[1]

        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)  # On ctrl-c ignore the signal... let the parent deal with it.

            # each new process must make a new random seed.
            np.random.seed(sim_params.index * 10)
            random.seed(sim_params.index * 10)

            # create the users sim by calling their supplied creation_function
            sim_instance = sim_params.creation_function()

            # build a list of the parameter and random seed modifications to make
            modifications = sim_params.modifications
            magnitudes = sim_params.dispersion_mag

            # we may want to disperse random seeds
            if sim_params.should_disperse_seeds:
                # generate the random seeds for the model (but don't apply them yet)
                # Note: This sets the RNGSeeds before all other modifications
                random_seed_dispersions = cls.disperse_seeds(sim_instance)
                for name, value in random_seed_dispersions.items():
                    modifications[name] = value

            # used if rerunning ICs from a .json file, modifications will contain the
            # RNGSeeds that need to be set before reset()
            cls.populate_seeds(sim_instance, modifications)

            # we may want to disperse parameters
            for disp in sim_params.dispersions:
                try:
                    name = disp.get_name()
                    if name not in modifications:  # could be using a saved parameter.
                        modifications[name] = disp.generate_string(sim_instance)
                        if sim_params.should_save_disp_mag:
                            magnitudes[name] = disp.generate_mag_string()
                except TypeError:
                    # This accomodates dispersion variables that are co-dependent
                    disp.generate(sim_instance)
                    for i in range(1, disp.number_of_sub_disps + 1):
                        name = disp.get_name(i)
                        if name not in modifications:  # could be using a saved parameter.
                            modifications[name] = disp.generate_string(i, sim_instance)
                            if sim_params.should_save_disp_mag:
                                magnitudes[name] = disp.generate_mag_string()

            # if archiving, this run's parameters and random seeds are saved in its own json file
            # save the dispersions and random seeds for this run
            with open(sim_params.initial_conditions_filename, 'w') as outfile:
                json.dump(modifications, outfile)
            if sim_params.should_save_disp_mag:
                with open(sim_params.magnitudes_filename, 'w') as outfileMag:
                    for k in sorted(magnitudes.keys()):
                        outfileMag.write("'%s':'%s', \n" % (k, magnitudes[k]))

            if sim_params.configure_function is not None:
                if sim_params.verbose:
                    print("Configuring sim")
                sim_params.configure_function(sim_instance)

            # apply the dispersions and the random seeds
            for variable, value in list(modifications.items()):
                expression = "sim_instance." + variable
                dispersion_expression = None
                if eval("callable(" + expression + ")"):
                    dispersion_expression = expression + "(" + value + ")"
                else:
                    dispersion_expression = expression + "=" + value

                if sim_params.verbose:
                    print("Executing parameter modification -> ", dispersion_expression)
                exec(dispersion_expression)

            # setup data logging
            if len(sim_params.retention_policies) > 0:
                if sim_params.verbose:
                    print("Adding retained data")
                RetentionPolicy.add_retention_policies_to_sim(sim_instance, sim_params.retention_policies)

            if sim_params.verbose:
                print("Executing simulation")
            # execute the simulation, with the user-supplied execution_function
            try:
                sim_params.execution_function(sim_instance)
            except TypeError:
                sim_params.execution_function(sim_instance, sim_params.results_filename)

            if len(sim_params.retention_policies) > 0:
                retention_file = sim_params.results_filename

                if sim_params.verbose:
                    print("Retaining data for run in", retention_file)

                retained_data = RetentionPolicy.get_data_for_retention(sim_instance, sim_params.retention_policies)
                data_out_queue.put((retained_data, sim_params.index, None))
                time.sleep(1)

                with gzip.open(retention_file, "w") as archive:
                    retained_data["index"] = sim_params.index # add run index
                    pickle.dump(retained_data, archive)

            if sim_params.verbose:
                print("Terminating simulation")

            if sim_params.verbose:
                print("Process", os.getpid(), "Job", sim_params.index, "finished successfully")

            return True, sim_params.index  # this function returns true only if the simulation was successful

        except Exception as e:
            print("Error in worker process", e)
            traceback.print_exc()
            return False, sim_params.index  # there was an error

    @staticmethod
    def disperse_seeds(sim_instance):
        """
        Disperses the RNG seeds of all the tasks in the sim, and returns a statement that contains the seeds.
        Example return dictionary::

             {
                '.TaskList[0].TaskModels[1]': 1934586,
                '.TaskList[0].TaskModels[2]': 3450093,
                '.TaskList[1].TaskModels[0]': 2221934,
                '.TaskList[2].TaskModels[0]': 1123244
             }

        :param sim_instance: A xmera simulation to set random seeds on
        :type sim_instance: SimulationBaseClass
        :return: A dictionary with the random seeds that should be applied to the sim
        """
        random_seeds = {}
        for i, task in enumerate(sim_instance.TaskList):
            for j, model in enumerate(task.TaskModels):
                task_var = 'TaskList[' + str(i) + '].TaskModels' + '[' + str(j) + '].RNGSeed'
                rand = str(random.randint(0, 1 << 32 - 1))
                try:
                    exec_statement = "sim_instance." + task_var + "=" + str(rand)
                    exec(exec_statement)
                    random_seeds[task_var] = rand
                except:
                    pass
        return random_seeds

    @staticmethod
    def populate_seeds(sim_instance, modifications):
        """
        Set the RNG seeds of all the tasks in the simulation.

        :param sim_instance: A xmera simulation to set random seeds on
        :type sim_instance: SimulationBaseClass
        :param modifications: A dictionary with the modifications to apply
        :type modifications: dict
        """
        for variable, value in modifications.items():
            if ".RNGSeed" in variable:
                rng_statement = "sim_instance." + variable + "=" + value
                exec(rng_statement)
