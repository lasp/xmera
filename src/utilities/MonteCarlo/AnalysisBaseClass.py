import glob
import os
import time

import holoviews as hv
import numpy as np
import pandas as pd

from xmera.utilities import macros
from xmera.utilities.dataframe_utilities import curve_per_df_component
from xmera.utilities.DS_Plot import DS_Plot


class McAnalysisBaseClass:
    def __init__(self):
        self.variable_name = ""
        self.variable_dim = 0
        self.data_dir = ""
        self.num_extrema = 0
        self.extrema_runs = []
        self.time_window = []
        self.data = None
        self.static_dir = None
        self.save_as_static = False

    def pull_and_format_df(self, path, var_idx_len):
        df = pd.read_pickle(path)
        if len(np.unique(df.columns.codes[1])) != var_idx_len:
            print("Warning: " + path + " not formatted correctly!")
            new_mult_index = pd.MultiIndex.from_product([df.columns.codes[0], range(var_idx_len)],
                                                        names=['runNum', 'varIdx'])
            indices = pd.Index([0, 1])  # Need multiple rows for curves
            df = df.reindex(columns=new_mult_index, index=indices)
        return df

    def get_nominal_run_indices(self, max_number=50):
        """
        Find the MC run indices of the most nominal runs.

        The method removes the runs with the largest standard deviation, step by step, until the number
        of runs is not more than ``max_number``.

        :param max_number: the maximum number of nominal runs that remain.
        :return: list of run indices
        """
        if self.data is None:
            self.data = pd.read_pickle(self.data_dir + "/" + self.variable_name + ".data")

        data_bar = self.data[np.abs(self.data - self.data.mean()) < 0.5 * self.data.std()]
        i = 5
        while len(data_bar.columns.codes[0]) > max_number * self.variable_dim:
            i += 1
            cols_to_delete = data_bar.columns[data_bar.isnull().sum() / len(data_bar) > 1. / np.sqrt(i)]
            data_bar.drop(cols_to_delete, axis=1, inplace=True)

        print("Nominal runs are ", list(dict.fromkeys(data_bar.columns.codes[0].tolist())))
        return data_bar.columns.codes[0]

    def get_extrema_run_indices(self, num_extrema, window):
        """
        Determine the MC run indices of the most deviant values within a particular time window
        Only compatible with curve_per_df_column

        :param num_extrema: number of extreme runs to collect
        :param window: window of time to search for the extremes in
        :return: list of run indices
        """
        if self.data is None:
            self.data = pd.read_pickle(self.data_dir + "/" + self.variable_name + ".data")
        times = self.data.index.tolist()

        # Find the closest indices to the time window requested
        ind_start = min(range(len(times)), key=lambda i: abs(times[i] - window[0]))
        ind_end = min(range(len(times)), key=lambda i: abs(times[i] - window[1]))
        self.time_window = [times[ind_start], times[ind_end]]

        # Find outliers based on largest deviation off of the mean
        mean = self.data.mean(axis=1)
        diff = self.data.abs().sub(mean, axis=0)
        self.extrema_runs = diff.transpose().nlargest(num_extrema, self.time_window).index
        print("Extrema runs are: ", list(dict.fromkeys(self.extrema_runs.tolist())))
        return self.extrema_runs

    def generate_stat_curves(self):
        """
        Generate curves that represent the mean, median, and standard deviation of a particular variable.
        Not Tested.
        """
        if self.data is None:
            self.data = pd.read_pickle(self.data_dir + "/" + self.variable_name + ".data")

        idx = pd.IndexSlice
        self.runs, self.var_num = self.data.columns.values[-1]
        self.runs += 1
        self.var_num += 1
        axes_mean = []
        axes_median = []
        axes_std = []
        for j in range(self.var_num):
            axis_mean = self.data.loc[idx[:], idx[:, j]].mean(axis=1)
            axis_median = self.data.loc[idx[:], idx[:, j]].median(axis=1)
            axis_std = self.data.loc[idx[:], idx[:, j]].std(axis=1)
            axes_mean.append(axis_mean)
            axes_median.append(axis_median)
            axes_std.append(axis_std)

        mean_run = pd.concat(axes_mean, axis=1)
        median_run = pd.concat(axes_median, axis=1)
        std_run = pd.concat(axes_std, axis=1)

        return [mean_run, median_run, std_run]

    def generate_stat_plots(self):
        """
        Generate plots for the mean, median, and mode.
        Not Tested.
        :return: list of stats plots
        """
        if self.data is None:
            self.data = self.pull_and_format_df(self.data_dir + "/" + self.variable_name + ".data", self.variable_dim)

        mean_run, median_run, std_run = self.generate_stat_curves()
        var_idx_list = range(self.variable_dim)

        mean_run.columns = pd.MultiIndex.from_product([['mean'], list(var_idx_list)], names=["stats", "varIdx"])
        median_run.columns = pd.MultiIndex.from_product([['median'], list(var_idx_list)], names=["stats", "varIdx"])
        std_run.columns = pd.MultiIndex.from_product([['std'], list(var_idx_list)], names=["stats", "varIdx"])

        mean_run_plot = DS_Plot(mean_run, title="Mean Plot: " + self.variable_name,
                               xAxisLabel='time[s]', yAxisLabel= self.variable_name.split('.')[-1],
                               macro_x=macros.NANO2SEC,
                               labels=['1', '2', '3'],
                               plotFcn=curve_per_df_component)

        med_run_plot = DS_Plot(median_run, title="Median Plot: " + self.variable_name,
                              xAxisLabel='time[s]', yAxisLabel= self.variable_name.split('.')[-1],
                              macro_x=macros.NANO2SEC,
                              labels=['1', '2', '3'],
                              plotFcn=curve_per_df_component)

        std_run_plot = DS_Plot(std_run, title="Standard Dev Plot: " + self.variable_name,
                              xAxisLabel='time[s]', yAxisLabel= self.variable_name.split('.')[-1],
                              macro_x=macros.NANO2SEC,
                              labels=['1', '2', '3'],
                              plotFcn=curve_per_df_component)

        stat_run_plots = []
        stat_run_plots.append(mean_run_plot)
        stat_run_plots.append(med_run_plot)
        stat_run_plots.append(std_run_plot)

        return stat_run_plots

    def extract_subset_of_runs(self, run_idx):
        """
        Create a separate folder in the data directory that contains the subset of data the user is looking to plot.
        If the ``/subset/`` directory already exists, check if it contains the data for the runs requested, if so skip.

        :param run_idx: list of run indices to extract
        :return: nothing
        """
        idx = pd.IndexSlice
        base_dir = self.data_dir
        new_list = []
        for run in run_idx:
            new_list.append(run[0])

        run_idx = new_list
        # check if a subset directory exists, and if it already contains all run_idx requested
        if not os.path.exists(base_dir + "/subset/"):
            os.mkdir(base_dir + "/subset/")
        else:
            file_paths = glob.glob(base_dir + "/subset" + "/*.data")
            for file_path in file_paths:
                if "MonteCarlo.data" in file_path:
                    continue
                if "run" in file_path and "overrun" not in file_path:
                    continue
                df = pd.read_pickle(file_path)
                singleton = list(dict.fromkeys(np.array(df.columns.codes[0]).tolist()))
                singleton_runs = list(dict.fromkeys(np.sort(np.array(run_idx)).tolist()))
                if len(singleton) == len(singleton_runs):
                    if singleton_runs == singleton:
                        print("Subset directory already contains run_idx values. Skipping extraction")
                        return
                    else:
                        break

        # If no data in subset (or the wrong data), extract and save the right data.
        print("Populating Subset Directory with Dataframes for runs: " + str(run_idx))
        # shutil.rmtree(data_dir + "/subset/")
        file_paths = glob.glob(base_dir + "/*.data")
        for file_path in file_paths:
            if "MonteCarlo.data" in file_path:
                continue
            if "run" in file_path and "overrun" not in file_path:
                continue
            df = pd.read_pickle(file_path)
            df_sub_set = df.loc[idx[:], idx[run_idx, :]]
            var_name = file_path.rsplit("/")
            pd.to_pickle(df_sub_set, base_dir + "/subset/" + var_name[-1])
        print("Finished Populating Subset Directory")

    def render_plots(self, plot_list):
        """
        Render all plots in plotList and print information about time taken, percent complete, which plot, etc.

        :param plot_list: List of plots to render
        :return: nothing.
        """
        hv.extension('bokeh')
        renderer = hv.renderer('bokeh').instance(mode='server')

        if self.save_as_static:
            print("Note: You requested to save static plots. This means no interactive python session will be generated.")
        print("Beginning the plotting")

        if not os.path.exists(self.data_dir + self.static_dir):
            os.mkdir(self.data_dir + self.static_dir)

        for i in range(len(plot_list)):
            start_time = time.time()
            image, title = plot_list[i].generateImage()
            try:
                if self.save_as_static:
                    # Save .html files of each of the plots into the static directory
                    hv.save(image, self.data_dir + self.static_dir + "/" + title + ".html")
                else:
                    renderer.server_doc(image)
                # Print information about the rendering process
                print("LOADED: " + title +"\t\t\t" +
                      "Percent Complete: " + str(round((i + 1) / len(plot_list) * 100, 2)) + "% \t\t\t"
                      "Time Elapsed: " + str( round(time.time() - start_time)) + " [s]")
            except Exception as e:
                print("Couldn't Plot " + title)
                print(e)
