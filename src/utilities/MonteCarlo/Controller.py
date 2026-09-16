# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import os
import random
import shutil
import sys
import traceback
import warnings

with warnings.catch_warnings():
    warnings.simplefilter("ignore", category=DeprecationWarning)
import copy
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


class Controller:
    """
    The MonteCarloController class is used to run a monte carlo simulation.
    It is used to execute multiple runs of a simulation with varying initial parameters. Data from each run is retained
    in order to analyze differences in the simulation runs and the parameters used.
    """

    def __init__(self):
        self.should_save_disp_mag = None
        self.should_disperse_seeds = False
        self.ic_filename = None
        self.num_simulation_runs = 0
        self.should_run_using_ic = False
        self.ic_directory = ""
        self.archive_dir = None
        self.var_cast = None
        self.num_processes = mp.cpu_count()
        self.verbose = False
        self.should_archive_parameters = False
        self.show_progress_bar = False
        self.creation_function=None
        self.execution_function=None
        self.configure_function=None
        self.retention_policies=[]
        self.dispersions=[]
        self.multi_proc_manager = None
        self.data_out_queue = None
        self.data_writer = None


    def set_show_progress_bar(self, value):
        """
        To enable or disable progress bar to show simulation progress
        Args:
            value: boolean value, decide to show/hide progress bar
        """
        self.show_progress_bar = value

    @staticmethod
    def load(run_directory):
        """
        Load a previously completed MonteCarlo simulation
        Args:
            The path to the MonteCarlo.data file that contains the archived MonteCarlo run
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

        Args:
            execution_function: (sim: SimulationBaseClass) => None
                A function with one parameter, a simulation instance.
                The function will be called after the creation_function and configurationFunction in each simulation run.
                It must execute the simulation.
                Its return value is not used.
        """
        self.execution_function = execution_function

    def set_configure_function(self, configure_function):
        """
        Set an execution function that executes a simulation instance.

        Args:
            configure_function: (sim: SimulationBaseClass) => None
                A function with one parameter, a simulation instance.
                The function will be called after the creation_function and configurationFunction in each simulation run.
                It must execute the simulation.
                Its return value is not used.
        """
        self.configure_function = configure_function

    def set_simulation_function(self, simulation_function):
        """
        Set the function that creates the simulation instance.

        Args:
            simulation_function: () => SimulationBaseClass
                A function with no parameters, that returns a simulation instance.
        """
        self.creation_function = simulation_function

    def set_should_disperse_seeds(self, seed_disp):
        """
        Disperse the RNG seeds of each run in the MonteCarlo

        Args:
            seed_disp: bool
                Whether to disperse the RNG seeds in each run of the simulation
        """
        self.should_disperse_seeds = seed_disp

    def set_execution_count(self, num_runs):
        """
        Set the number of runs for the MonteCarlo simulation

        Args:
            num_runs: int
                The number of runs to use for the simulation
        """
        self.num_simulation_runs = num_runs

    def add_dispersion(self, disp):
        """
        Add a dispersion to the simulation.

        Args:
            disp: Dispersion
                The dispersion to add to the simulation.
        """
        self.dispersions.append(disp)

    def add_retention_policy(self, policy):
        """
        Add a retention policy to the simulation.

        Args:
            disp: RetentionPolicy
                The retention policy to add to the simulation.
                This defines variables to be logged and saved
        """
        self.retention_policies.append(policy)

    def set_num_worker_processes(self, num_processes):
        """
        Set the number of worker processes for the Monte Carlo batch.

        Args:
            num_processes: int
                Number of num_processes to execute the montecarlo run on.
        """
        self.num_processes = num_processes

    def set_verbose(self, verbose):
        """
        Use verbose output for this MonteCarlo run

        Args:
            verbose: bool
                Whether to print verbose information during this MonteCarlo sim.
        """
        self.verbose = verbose

    def set_disp_magnitude_file(self, magnitudes):
        """
        Save .txt with the magnitude of each dispersion in % or sigma away from mean

        Args:
            magnitudes: bool
                Whether to save extra files for analysis.
        """
        self.should_save_disp_mag = magnitudes

    def set_should_archive_parameters(self, should_archive_parameters):
        self.should_archive_parameters = should_archive_parameters

    def set_archive_dir(self, dir_name):
        """
        Set-up archives for this MonteCarlo run

        Args:
            dir_name: string
                The name of the directory to archive runs in.
                None, if no archive desired.
        """
        self.archive_dir = os.path.abspath(dir_name) + "/"
        self.should_archive_parameters = dir_name is not None

    def set_var_cast(self, var_cast):
        """
        Set the variable type to downcast the data to

        :param var_cast: 'float', 'integer', 'signed', or 'unsigned'. Refer to the pandas.to_numeric documentation.
        :return:
        """
        self.var_cast = var_cast

    def set_ic_dir(self, dir_name):
        """
        Set-up archives containing IC data

        Args:
            dir_name: string
                The name of the directory to archive runs in.
                None, if no archive desired.
        """
        self.ic_directory = os.path.abspath(dir_name) + "/"
        self.should_archive_parameters = True

    def set_should_run_using_ic(self, value):
        """
        Set the number of threads to use for the monte carlo simulation

        Args:
            value: bool
                Number of threads to execute the montecarlo run on.
        """
        self.should_run_using_ic = value

    def get_retained_data(self, case):
        """
        Get the data that was retained for a run, or list of runs.

        Args:
            case: int The desired case to get data from.
        Returns:
            The retained data for that run is returned.
        """
        if self.should_run_using_ic:
            old_run_data_file = self.ic_directory + "run" + str(case) + ".data"
        else:
            old_run_data_file = self.archive_dir + "run" + str(case) + ".data"

        with gzip.open(old_run_data_file) as pickled_data:
            data = pickle.load(pickled_data)
            return data

    def get_retained_datas(self, cases):
        """
        Get the data that was retained for a list of runs.

        Args:
            cases: int[] The desired run_indexes to get data from.
        Returns:
            A generator is returned, which will yield, in-order, the retained data for each of these run_indexes
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
        if self.should_run_using_ic:
            filename = self.ic_directory + "run" + str(run_index) + ".json"
        else:
            filename = self.archive_dir + "run" + str(run_index) + ".json"
        with open(filename, "r") as dispersion_file:
            dispersions = json.load(dispersion_file)
            return dispersions

    def re_run_cases(self, run_indexes):
        """
        Rerun some run_indexes from a MonteCarlo run. Does not run in parallel

        Args:
            run_indexes: int[]
                The list of runs to repeat, a list of numbers.
        Returns:
            failures: int[]
                The list of failed runs.
        """
        # the list of failures
        failed = []

        for run_index in run_indexes:
            if self.verbose:
                print("Rerunning", run_index)

            old_run_file = self.archive_dir + "run" + str(run_index) + ".json"
            if not os.path.exists(old_run_file):
                print("ERROR re-running case: " + old_run_file)
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
            success = executor([sim_params, self.data_out_queue])

            if not success:
                print("Error re-executing run", run_index)
                failed.append(run_index)

        if len(failed) > 0:
            failed.sort()
            print("Failed rerunning run_indexes:", failed)

        return failed

    def run_initial_conditions(self, run_indexes):
        """
        Run initial conditions given in a file

        Args:
            run_indexes: int[]
                The list of runs to repeat, a list of numbers.
        Returns:
            failures: int[]
                The list of failed runs.
        """
        # the list of failures
        failed = []

        assert self.ic_directory != "", "No initial condition directory was given"
        assert self.should_run_using_ic is not False, "IC run flag was not set"

        if self.verbose:
            print("Beginning simulation with {0} runs on {1} processes".format(self.num_simulation_runs,
                                                                               self.num_processes))

        if self.should_archive_parameters:
            if not os.path.exists(self.ic_directory):
                print("Cannot run initial conditions: the directory given does not exist")

            if self.verbose:
                print("Archiving a copy of this simulation before running it in 'MonteCarlo.data'")
            try:
                with gzip.open(self.ic_directory + "MonteCarlo.data", "w") as pickle_file:
                    pickle.dump(self, pickle_file)  # dump this controller object into a file.
            except Exception as e:
                print("Unknown exception while trying to pickle monte-carlo-controller... \ncontinuing...\n\n", e)

        # Create Queue, but don't ever start it.
        self.multi_proc_manager = mp.Manager()
        self.data_out_queue = self.multi_proc_manager.Queue()
        self.data_writer = DataWriter(self.data_out_queue)
        self.data_writer.daemon = False

        # If archiving the rerun data -- make sure not to delete the original data!
        if self.archive_dir is not None:
            if self.archive_dir != self.ic_directory:
                if os.path.exists(self.archive_dir):
                    shutil.rmtree(self.archive_dir)
                os.mkdir(self.archive_dir)
                self.data_writer.set_log_dir(self.archive_dir)
                self.data_writer.start()
            else:
                print("ERROR: The archive directory is set as the ic_directory. Proceeding would have overwriten all data " \
                      "within: " + self.archive_dir + " with the select rerun run_indexes! Exiting.\n")
                sys.exit("Change the archive directory to a new location when rerunning run_indexes.")
        else:
            print("No archive data specified; no data will be logged to dataframes")

        jobs_finished = 0  # keep track of what simulations have finished

        # The simulation executor is responsible for executing simulation given a simulation's parameters
        # It is called within worker processes with each worker's simulation parameters
        simulation_executor = SimulationExecutor()
        #
        progress_bar = SimulationProgressBar(len(run_indexes), self.show_progress_bar)
        if self.num_processes == 1:
            if self.verbose:
                print("Executing sequentially...")
            i = 0
            for i in range(len(run_indexes)):
                sim_generator = self.generate_ic_sims(run_indexes[i:i + 1])
                for sim in sim_generator:
                    try:
                        simulation_executor([sim, self.data_out_queue])
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
        # If the data was archiving, close the queue.
        if self.archive_dir is not None and self.archive_dir != self.ic_directory:
            while not self.data_out_queue.empty():
               time.sleep(1)
            self.data_out_queue.put((None, None, True))
            time.sleep(5)

        # if there are failures
        if len(failed) > 0:
            failed.sort()

            if self.verbose:
                print("Failed", failed, "saving to 'failures.txt'")

            if self.should_archive_parameters:
                # write a file that contains log of failed runs
                with open(self.ic_directory + "failures.txt", "w") as fail_file:
                    fail_file.write(str(failed))

        return failed

    def generate_ic_sims(self, run_indexes):
        """
        Generator function to clone a baseSimulation for IC run

        Args:
            run_indexes: int[]
                The desired run indexes to generate simulation parameters from saved IC file.
        Returns:
            generator<SimulationParams>
                A generator that yields that number of cloned simulations
        """

        # make a list of simulations to execute by cloning the base-simulation and
        # changing each clone's index and filename to make a list of
        # simulations to execute
        for run_index in run_indexes:
            if self.verbose:
                print("Running IC ", run_index)

            old_run_file = self.ic_directory + "run" + str(run_index) + ".json"
            if not os.path.exists(old_run_file):
                print("ERROR running IC case: " + old_run_file)
                continue

            # use old simulation parameters, modified slightly.
            sim_params = self.create_sim_parameters(run_index)
            sim_params.index = run_index
            # don't redisperse seeds, we want to use the ones saved in the old_run_file
            sim_params.should_disperse_seeds = False

            sim_params.ic_filename = self.ic_directory + "run" + str(run_index)
            with open(old_run_file, "r") as run_parameters:
                sim_params.modifications = json.load(run_parameters)

            yield sim_params

    def create_sim_parameters(self, index):
        sim_params = SimulationParameters(self.creation_function,
                                          self.execution_function,
                                          self.configure_function,
                                          self.retention_policies,
                                          self.dispersions,
                                          self.should_disperse_seeds,
                                          self.should_archive_parameters,
                                          os.path.join(self.archive_dir, "run" + str(index)),
                                          os.path.join(self.archive_dir, "run" + str(index)),
                                          os.path.join(self.archive_dir, "run" + str(index) + "mag.txt"),
                                          index)
        sim_params.verbose = self.verbose
        sim_params.show_progress_bar = self.show_progress_bar
        sim_params.should_save_disp_mag = self.should_save_disp_mag
        return sim_params

    def generate_sims(self, sim_run_indexes):
        """
        Generator function to clone a baseSimulation

        Args:
            sim_run_indexes: int[]
                The desired runs to generate.
        Returns:
            generator<SimulationParams>
                A generator that yields that number of cloned simulations
        """

        # make a list of simulations to execute by cloning the base-simulation and
        # changing each clone's index and filename to make a list of
        # simulations to execute
        for run_index in sim_run_indexes:
            sim_params = self.create_sim_parameters(run_index)
            sim_params.index = run_index
            sim_params.filename += "run" + str(run_index)

            yield sim_params

    def execute_callbacks(self, rng=None, retention_policies=[]):
        """
        Execute retention policy callbacks after running a monteCarlo sim.

        Args:
            rng: A list of simulations to execute callbacks on
            retention_policies: the retention policies to execute
        """

        if rng is None:
            rng = list(range(self.num_simulation_runs))

        if not retention_policies:
            retention_policies = self.retention_policies

        for sim_index in rng:
            data = self.get_retained_data(sim_index)
            for retention_policy in retention_policies:
                retention_policy.execute_callback(data)

    def execute_simulations(self):
        """
        Execute simulations in parallel

        :return: failed: int[]
                 A list of the indices of all failed simulation runs.
        """

        if self.verbose:
            print("Beginning simulation with {0} runs on {1} processes".format(self.num_simulation_runs,
                                                                               self.num_processes))

        if self.should_archive_parameters:
            if os.path.exists(self.archive_dir):
                shutil.rmtree(self.archive_dir, ignore_errors=True)
            os.mkdir(self.archive_dir)
            if self.verbose:
                print("Archiving a copy of this simulation before running it in 'MonteCarlo.data'")
            try:
                with gzip.open(self.archive_dir + "MonteCarlo.data", "wb") as pickle_file:
                    pickle.dump(self, pickle_file)  # dump this controller object into a file.
            except Exception as e:
                print("Unknown exception while trying to pickle monte-carlo-controller... \ncontinuing...\n\n", e)

        self.multi_proc_manager = mp.Manager()
        self.data_out_queue = self.multi_proc_manager.Queue()
        self.data_writer = DataWriter(self.data_out_queue)
        self.data_writer.daemon = False

        num_sims = self.num_simulation_runs

        # start data writer process
        self.data_writer.set_log_dir(self.archive_dir)
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
                        run_ok = simulation_executor([sim, self.data_out_queue])[0]
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

        # if there are failures
        if len(failed) > 0:
            failed.sort()

            if self.verbose:
                print("Failed", failed, "saving to 'failures.txt'")

            if self.should_archive_parameters:
                # write a file that contains log of failed runs
                with open(self.archive_dir + "failures.txt", "w") as fail_file:
                    fail_file.write(str(failed))

        return failed


class SimulationParameters:
    """
    This class represents the run parameters for a simulation, with information including

     - a function that creates the simulation
     - a function that executes the simulation
     - the dispersions to use on that simulation
     - parameters describing the data to be retained for a simulation
     - whether randomized seeds should be applied to the simulation
     - whether data should be archived
    """

    def __init__(self,
                 creation_function,
                 execution_function,
                 configure_function,
                 retention_policies,
                 dispersions,
                 should_disperse_seeds,
                 should_archive_parameters,
                 filename,
                 ic_filename,
                 magnitudes_filename,
                 index=None,
                 verbose=False,
                 modifications={}):
        self.magnitudes_filename = magnitudes_filename
        self.index = index
        self.creation_function = creation_function
        self.execution_function = execution_function
        self.configure_function = configure_function
        self.retention_policies = retention_policies
        self.dispersions = dispersions
        self.should_disperse_seeds = should_disperse_seeds
        self.should_archive_parameters = should_archive_parameters
        self.filename = filename
        self.ic_filename = ic_filename
        self.verbose = verbose
        self.modifications = modifications
        self.dispersion_mag = {}
        self.should_save_disp_mag = False
        self.show_progress_bar = False



class SimulationExecutor:
    """
    This class executes a simulation in a worker process.
    To use it, create an instance of this class. Then call the instance with the simulation parameters::

        executor = SimulationExecutor()
        sim_params = SimulationParameters()
        successFlag = executor(sim_params)

    To execute a simulation in a different process, use this class as the target of that process.
    """
    #

    @classmethod
    def __call__(cls, params):
        """
        In each worker process, we execute this function (by calling this object)

        Args:
            params [sim_params, data out queue]:
                A SimulationParameters object for the simulation to be executed and the output data queue
                for the data writer.
        Returns:
            success: bool
                (True, sim_params.index) if simulation run was successful
                (False, sim_params.index) if simulation run was unsuccessful
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
            if sim_params.should_archive_parameters:
                # save the dispersions and random seeds for this run
                if sim_params.ic_filename != "":
                    with open(sim_params.ic_filename + ".json", 'w') as outfile:
                        json.dump(modifications, outfile)
                else:
                    with open(sim_params.filename + ".json", 'w') as outfile:
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
                sim_params.execution_function(sim_instance, sim_params.filename)

            if len(sim_params.retention_policies) > 0:
                if sim_params.ic_filename != "":
                    retention_file = sim_params.ic_filename + ".data"
                else:
                    retention_file = sim_params.filename + ".data"

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
        only populate the RNG seeds of all the tasks in the sim

        Args:
            sim_instance: SimulationBaseClass
                A xmera simulation to set random seeds on
            modifications:
                A dictionary containing RNGSeeds to be populated for the sim, among other sim modifications.
        """
        for variable, value in modifications.items():
            if ".RNGSeed" in variable:
                rng_statement = "sim_instance." + variable + "=" + value
                exec(rng_statement)
