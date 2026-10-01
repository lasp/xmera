# MonteCarlo: Brief Guide

This guide gives a short overview of the `MonteCarlo` package. For all of the details, refer to the docstrings in
`MonteCarlo/Controller.py`. For examples, refer to `examples/scenarioMonteCarloAttRW.py`,
`examples/MonteCarloExamples/` and `MonteCarlo/_tests/test_monte_carlo_simulation.py`.

The `Controller` class executes a simulation many times. For each run, the controller does these steps:

1. It makes the simulation.
2. It sets random seeds, if you select this option.
3. It applies the dispersions to the initial parameters.
4. It executes the simulation.
5. It saves the retained data to disk.

After the batch, you can use the `Controller` class to load the saved data. You can also run cases again with the same
seeds and parameters.

## Configure a Monte Carlo batch

Import the controller, the retention policy and the dispersions that you need:

```python
from xmera.utilities.MonteCarlo.Controller import Controller
from xmera.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy
from xmera.utilities.MonteCarlo.Dispersions import UniformEulerAngleMRPDispersion

monte_carlo = Controller()
```

Write a function that makes the simulation and returns it. The worker processes use `pickle` to get this function.
Thus, put this function at module level. You can also use `functools.partial` of a module-level function. Do not use a
closure or a lambda.

```python
def create_sim():
    sim = SimulationBaseClass()
    # configure the sim ...
    return sim

monte_carlo.set_simulation_function(create_sim)
```

Write a function that executes the simulation:

```python
def execute_sim(sim):
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

monte_carlo.set_execution_function(execute_sim)
```

You can also set a configure function. The controller calls it after it sets the random seeds and before it applies
the dispersions.

```python
monte_carlo.set_configure_function(configure_sim)
```

Set the number of runs and the directory for the data. The controller does not start a run if `archive_dir` is not
set.

```python
monte_carlo.set_execution_count(100)
monte_carlo.archive_dir = "mc_data"
```

These settings are optional:

| Setting | Effect |
|---|---|
| `set_should_disperse_seeds(True)` | Sets a different random seed on each task model for each run. |
| `set_num_worker_processes(n)` | Sets the number of worker processes. The default is the number of CPU cores. A value of 1 executes the runs one after the other in the current process. |
| `set_var_cast("float")` | Changes the retained values to this type before the data writer writes them. This decreases the size of the files. |
| `set_should_save_disp_mag(True)` | Writes a `run<N>mag.txt` file that gives each dispersion in standard deviations. |
| `set_show_progress_bar(True)` | Shows a progress bar. |
| `log_level = "DEBUG"` | Sets the log level of the worker processes. |

## Dispersions

A dispersion gives a random value to one parameter of the simulation for each run. `MonteCarlo/Dispersions.py`
contains the available dispersions.

```python
monte_carlo.add_dispersion(UniformEulerAngleMRPDispersion("TaskList[0].TaskModels[0].hub.sigma_BNInit"))
```

The name of a dispersion is a path from the simulation object. A path contains attribute names and literal
indexes, for example `dynamics.spacecraft.hub.mHub` or `TaskList[0].TaskModels[2].RNGSeed`. A path cannot contain
calls such as `get_model()`. If a path contains other syntax, the run stops with `DispersionApplyError`.

If the path ends at a method, the controller calls the method with the value. If the value is a tuple, the
controller gives each item of the tuple as a different argument.

The controller saves each value as a string in `run<N>.json`. The string must be a Python literal that
`ast.literal_eval` can read, for example `1.5`, `[0.1, 0.2, 0.3]` or `'Sphere'`.

To make sure that a set of dispersions is correct without a Monte Carlo batch, use `MonteCarlo/PathWalk.py`:

| Function | Effect |
|---|---|
| `resolve_path(sim, path)` | Returns the object at the path. |
| `apply_modification(sim, path, value_str)` | Applies one value. |
| `generate_modifications(sim, dispersions)` | Returns a new value for each dispersion. |
| `apply_dispersions(sim, dispersions)` | Generates the values and applies them, as a worker does. |

## Retained data

A `RetentionPolicy` gives the messages and variables that the controller saves from each run. It can also have a
callback that uses the data of one run, for example to make a plot.

```python
retention_policy = RetentionPolicy()
retention_policy.add_message_log("inertial_state_output", ["r_BN_N", "v_BN_N"])

def plot_velocity(data, retention_policy):
    v_BN_N = data["messages"]["inertial_state_output.v_BN_N"]
    plt.plot(v_BN_N[:, 0], v_BN_N[:, 1])

retention_policy.set_data_callback(plot_velocity)
monte_carlo.add_retention_policy(retention_policy)
```

The controller reads each message from the recorder in `sim.msgRecList[name]`. Thus the simulation must create
that recorder. The first column of each retained array is the message time in nanoseconds.

## Execute the batch

```python
failures = monte_carlo.execute_simulations()
```

`execute_simulations` returns a list of `FailureRecord` objects, one for each run that stopped with an error. Each record has
`run_index`, `exception_type` and `traceback`. To get only the indexes, use `[f.run_index for f in failures]`.

## Directory layout

Each call to `execute_simulations` or `run_initial_conditions` makes a new run directory in `archive_dir`. The
controller does not remove the run directories that are already there.

```
<archive_dir>/
    mc_run_<YYYYmmdd-HHMMSS>/
        MonteCarlo.data                 the saved controller
        failures.txt                    the sorted indexes of the runs that stopped with an error
        failures.json                   the FailureRecord of each of these runs
        initial_conditions/run<N>.json  the dispersion values and seeds of each run
        results/run<N>.data             the retained data of each run
        results/run<N>mag.txt           the dispersion magnitudes, if enabled
        results/<message>.<field>.data  one pandas DataFrame for each retained field, for all runs
```

After a batch, `monte_carlo.mc_run_dir`, `monte_carlo.results_dir` and `monte_carlo.ic_directory` give these
paths.

## Load the data

Load the controller from a run directory. This can be in a different script.

```python
run_dir = Controller.latest_run_dir("mc_data")
monte_carlo = Controller.load(run_dir)

data = monte_carlo.get_retained_data(19)
data["messages"]["inertial_state_output.r_BN_N"]

parameters = monte_carlo.get_parameters(19)
```

The retained data of a run is a dictionary:

```
{
    "messages": {"messageName.fieldName": array},
    "variables": {"variableName": array},
    "custom": {...},
    "index": 19,
}
```

To execute the callbacks on all runs, or on some runs and policies only:

```python
monte_carlo.execute_callbacks()
monte_carlo.execute_callbacks(run_indexes=[4, 6, 27], retention_policies=[retention_policy])
```

For statistics and plots of many runs, use `McAnalysisBaseClass` in `MonteCarlo/AnalysisBaseClass.py`. Set its
`data_dir` to the `results` directory of the run.

## Run cases again

To execute runs again from their saved initial conditions, and retain the data in a new run directory:

```python
ic_directory = os.path.join(Controller.latest_run_dir("mc_data"), "initial_conditions")
monte_carlo.archive_dir = "mc_data"
failures = monte_carlo.run_initial_conditions([4, 6], ic_directory)
```

To execute runs again in the current process without retained data, for example to find the cause of an error, load
the run first:

```python
monte_carlo = Controller.load(run_dir)
failures = monte_carlo.re_run_cases([4, 6])
```
