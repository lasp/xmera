# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

r"""
Motivation
----------
This script is a basic demonstration of a script that can be used to plot Monte Carlo data with
bokeh and datashaders.   These tools are very efficient to plot large amounts of simulation data
that is likely to occur with Monte Carlo sensitivity analysis studies.  For example, running this script will
create an HTML interactive view of the simulation data.   Instead of seeing a fixed resolution, the user can
zoom into the data dynamically to see more detail.  This process recreates a newly render view of the simulation data.

The following two plots illustrate what this particular simulation setup will yield.

.. _scenarioAnalyzeMonteCarlo-ds0:
.. figure:: /_images/static/ds-0.png
    :align: center
    :scale: 50%

    Figure 1: Full view of the attitude error plot data

.. _scenarioAnalyzeMonteCarlo-ds1:
.. figure:: /_images/static/ds-1.png
    :align: center
    :scale: 50%

    Figure 2: Zoomed in and nearly rendered view of the attitude error data details

The next plot shows the output of ``scenario_AttFeedbackMC.py`` with more simulation runs.
This plot shows 40 runs.

.. _scenarioAnalyzeMonteCarlo-ds2:
.. figure:: /_images/static/ds-2.png
    :align: center
    :scale: 50%

    Figure 3: Larger Monte Carlo batch with 40 simulation runs

Configuring a Python Environment For this Script
------------------------------------------------
.. danger::

    Running this script is different from running other BSK scripts.  There are very particular python
    package requirements that must be carefully followed.  It is recommended the user create a
    virtual python environment as discussed in the installation setup.  This environment might have to be
    specific to running this script because of these dependency challenges.

The setup steps are as follows:

#. The datashaders etc. require that this script be run with Python 3.7, not higher
#. Create dedicated virtual environment and compile Xmera for this environment
#. Install this particular version of ``panel`` package first.  It must be done alone as it upgrades
   ``bokeh`` to a version that is too new::

        pip3 install --upgrade panel==1.4.4

#. Next, install the following particular python package versions::

        pip3 install --upgrade bokeh==3.4.2 holoviews==1.16.0 param==2.1.1 hvplot==0.10.0

How to Run the Script
---------------------
.. important::

    Read all three steps before advancing.

The next steps outline how to run this script.

1.  This script can only be run once there exists data produced by the ``scenario_AttFeedbackMC.py`` script.

2.  At the bottom of this script, comment out the name guard and associated ``run()`` statement,
    and un-comment the following ``run()`` statement before this script can run.
    These lines are provided in their commented/uncommented form
    to ensure that the sphinx documentation generation process does not
    run this script automatically.

3.  This script must be called from command line using::

        /$path2bin/panel serve --show /$path2script/scenarioAnalyzeMonteCarlo.py

This will process the data created with ``scenario_AttFeedbackMC.py`` and open a browser window showing
Figure 1 above.  To end the script you need to press the typical key strokes to interrupt a process as the
bokeh server will keep running until stopped.

"""

import inspect
import os

from bokeh.palettes import RdYlBu9

import xmera.utilities.macros as macros
from xmera.utilities.DS_Plot import DS_Plot
from xmera.utilities.MonteCarlo.AnalysisBaseClass import McAnalysisBaseClass
from xmera.utilities.dataframe_utilities import curve_per_df_column, pull_and_format_df


filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))
from xmera import __path__

bskPath = __path__[0]

def plotSuite(dataDir):
    """
    This is the function to populate with all of the plots to be generated using datashaders and bokeh.
    Each variable requires a call to ``pull_and_format_df()`` to ensure the dataframe will be compatible with
    the developed datashader utilities.

    Args:
        dataDir: (str) directory containing all of the dataframes created from the Monte Carlo run

    Returns: List of DS_Plots

    """
    plotList = []
    sigma_BR = pull_and_format_df(os.path.join(dataDir, "attGuidMsg.sigma_BR.data"), 3)
    sigmaPlot = DS_Plot(sigma_BR, title="Attitude Error",
                        xAxisLabel='time [s]', yAxisLabel='Sigma_BR',
                        macro_x=macros.NANO2SEC,
                        labels=['b1', 'b2', 'b3'], cmap=RdYlBu9,
                        plotFcn=curve_per_df_column)
    plotList.append(sigmaPlot)

    sigma_BR = pull_and_format_df(os.path.join(dataDir, "attGuidMsg.omega_BR_B.data"), 3)
    sigmaPlot = DS_Plot(sigma_BR, title="Attitude Rate Error",
                        xAxisLabel='time [s]', yAxisLabel='omega_BR_B',
                        macro_x=macros.NANO2SEC, macro_y=macros.R2D,
                        labels=['b1', 'b2', 'b3'], cmap=RdYlBu9,
                        plotFcn=curve_per_df_column)
    plotList.append(sigmaPlot)
    return plotList


def run(show_plots):
    """
    **This script is meant to be configured based on the user's needs. It can be configured using the following
    three booleans:**

    First, set ``show_all_data = True`` to get a broad view of the data and find a time window to investigate closer.

    When you know the behavior of the data, set ``show_extreme_data = True`` to look at specific runs
    in the window.

    Finally, set ``show_optional_data = True`` to look at extra data. This data can show why the extrema
    runs occur.

    :param show_all_data: plot all MC runs for the plots specified in the plotSuite method
    :param show_extreme_data: call plotSuite method for user-defined number of extrema MC runs
    :param optional_plots: plots additional user-defined plots
    """

    show_all_data = True
    show_extreme_data = True
    optional_plots = False

    plotList = []
    analysis = McAnalysisBaseClass()
    analysis.data_dir = os.path.join(path, "scenario_AttFeedbackMC")

    # save_as_static: save off static .html files of the plots generated into the static_dir directory.
    # The static_dir will be created inside the data_dir folder.
    # (Note: This inhibits dynamic plotting!
    analysis.save_as_static = True
    analysis.static_dir = "/plots/"

    if show_all_data:
        plotList.extend(plotSuite(analysis.data_dir))

    if show_extreme_data:
        analysis.variable_name = "attGuidMsg.sigma_BR"
        analysis.variable_dim = 1

        extrema_run_numbers = analysis.get_extrema_run_indices(num_extrema=1, window=[500 * 1e9, 550 * 1e9])

        analysis.extract_subset_of_runs(run_idx=extrema_run_numbers)
        plotList.extend(plotSuite(os.path.join(analysis.data_dir, "subset")))

    if optional_plots:
        # nominalRuns = analysis.getNominalRunIndices(50)
        # statPlots = analysis.generateStatPlots()

        shadowFactor = pull_and_format_df(os.path.join(analysis.data_dir, "eclipse_data_0.shadowFactor.data"), 1)
        shadowFactor = shadowFactor.dropna(axis=1)
        shadowFactorPlot = DS_Plot(shadowFactor, title="Optional Plots: Eclipse",
                                               xAxisLabel='time[s]', yAxisLabel='Eclipse Factor',
                                               macro_x=macros.NANO2SEC, macro_y=macros.R2D,
                                               cmap=RdYlBu9,
                                               plotFcn=curve_per_df_column)

        # plot_list.extend([statPlots])
        plotList.extend([shadowFactorPlot])

    analysis.render_plots(plotList)

# The following must be commented out before this script can run.  It is provided here
# to ensure that the sphinx documentation generation process does not run this script
# automatically.
if __name__ == "__main__":
    run(False)


# uncomment the following line to run this script.
# run(False)
