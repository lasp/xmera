import multiprocessing as mp
import os
import pickle

import numpy as np
import pandas as pd


class DataWriter(mp.Process):
    """ Class to be launched as separate process to pull data from queue and write out to .csv dataFrames
        Args:
            q: queue object from multiprocessing.Manager.queue
        Returns:
            Nil
    """
    def __init__(self, q):
        super(DataWriter, self).__init__()
        self._queue = q
        self._end_token = None
        self._var_cast = None
        self._log_dir = ""
        self._data_files = set()

    def run(self):
        """ The process run loop. Gets data from a queue and writes it out to per message csv files
            Args:
                Nil
            Returns:
                Nil
        """
        while self._end_token is None:
            data, mc_sim_index, self._end_token = self._queue.get()
            print("Starting to log: " + str(mc_sim_index))
            if self._end_token:
                continue
            print("Logging Dataframes from run " + str(mc_sim_index))
            for dict_name, dict_data in data.items(): # Loops through Messages, Variables, Custom dictionaries in the retention policy
                for item_name, item_data in dict_data.items(): # Loop through all items and their data

                    if item_name == "OrbitalElements.Omega": # Protects from OS that aren't case sensitive.
                        item_name = "OrbitalElements.Omega_Capital"

                    file_path = self._log_dir + item_name + ".data"
                    self._data_files.add(file_path)

                    # Is the data a vector, scalar, or non-existant?
                    try:
                        vari_len = item_data[:,1:].shape[1]
                    except:
                        vari_len = 0

                    # Generate the MultiLabel
                    outer_label = [mc_sim_index]
                    inner_label = []

                    for i in range(vari_len):
                        inner_label.append(i)
                    if vari_len == 0:
                        inner_label.append(0) # May not be necessary, might be able to leave blank and get a None
                    labels = pd.MultiIndex.from_product([outer_label, inner_label], names=["runNum", "varIdx"])

                    # Generate the individual run's dataframe
                    if vari_len >= 2:
                        df = pd.DataFrame(item_data[:, 1:].tolist(), index=item_data[:,0], columns=labels)
                    elif vari_len == 1:
                        df = pd.DataFrame(item_data[:, 1].tolist(), index=item_data[:,0], columns=labels)
                    else:
                        df = pd.DataFrame([np.nan], columns=labels)

                    for i in range(0, vari_len):
                        try: # if the data is numeric reduce it to float32 rather than float64 to reduce storage footprint
                            # Note: You might think you can simplify these three lines into a single:
                            # df.iloc[:,i] = df.iloc[:,i].apply(pandas.to_numeric, downcast="float")
                            # but you'd be wrong.
                            var_comp = df.iloc[:,i]
                            if self._var_cast != None:
                                var_comp = pd.to_numeric(var_comp, downcast='float')
                            df.iloc[:,i] = var_comp
                        except:
                            pass

                    # If the .data file doesn't exist save the dataframe to create the file
                    # and skip the remainder of the loop
                    if not os.path.exists(file_path):
                        pickle.dump([df], open(file_path, "wb"))
                        continue

                    # If the .data file does exists, append the message's pickle.
                    with open(file_path, "a+b") as pkl:
                        pickle.dump([df], pkl)

            print("Finished logging dataframes from run" + str(mc_sim_index))

        # Sort by the MultiIndex (first by run number then by variable component)
        print("Starting to concatenate dataframes")
        for file_path in self._data_files:
            # We create a new index so that we populate any missing run data (in the case that a run breaks) with NaNs.
            all_data = []
            with open(file_path, 'rb') as pkl:
                try:
                    while True:
                        all_data.extend(pickle.load(pkl))
                except EOFError:
                    pass
            all_data = pd.concat(all_data, axis=1)
            new_mult_ind = pd.MultiIndex.from_product([list(range(all_data.columns.min()[0], all_data.columns.max()[0]+1)),
                                                         list(range(all_data.columns.min()[1], all_data.columns.max()[1]+1))],
                                                         names=["runNum", "varIdx"])
            #all_data = all_data.sort_index(axis=1, level=[0,1]) #TODO: When we dont lose MCs anymore, we should just use this call
            all_data = all_data.reindex(columns=new_mult_ind)
            all_data.index.name = 'time[ns]'
            all_data.to_pickle(file_path)
        print("Finished concatenating dataframes")

    def set_log_dir(self, log_dir):
        self._log_dir = log_dir

    def set_var_cast(self, var_cast):
        self._var_cast = var_cast
