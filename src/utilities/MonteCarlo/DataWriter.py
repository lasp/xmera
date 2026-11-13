import logging
import multiprocessing as mp
import os
import pickle

import numpy as np
import pandas as pd


logger = logging.getLogger("montecarlo_datawriter")


class DataWriter(mp.Process):
    """ Class to be launched as separate process to pull data from queue and write out to .csv dataFrames
        Args:
            q: queue object from multiprocessing.Manager.queue
        Returns:
            Nil
    """

    # Two names that are different only in case collide on a case-insensitive filesystem (HFS+ and
    # APFS in their default macOS configuration). Without a rename, the second file replaces the first
    # file on disk and no error occurs. Each key is the original name of the retained data. Each value
    # is the new file name. Add an entry to this map when you find a new collision.
    _CASE_INSENSITIVE_RENAMES = {
        # OrbitalElements has the two names "omega" (argument of periapsis) and "Omega" (longitude
        # of ascending node). These names collide on macOS.
        "OrbitalElements.Omega": "OrbitalElements.Omega_Capital",
    }

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
            logger.info(f"Starting to log: {str(mc_sim_index)}")
            if self._end_token:
                continue
            logger.info(f"Logging Dataframes from run {str(mc_sim_index)}")
            for dict_name, dict_data in data.items(): # Loops through Messages, Variables, Custom dictionaries in the retention policy
                for item_name, item_data in dict_data.items(): # Loop through all items and their data

                    item_name = self._CASE_INSENSITIVE_RENAMES.get(item_name, item_name)
                    file_path = os.path.join(self._log_dir, item_name + ".data")
                    self._data_files.add(file_path)

                    # The data can be a vector, a scalar, or missing.
                    try:
                        vari_len = item_data[:,1:].shape[1]
                    except (AttributeError, IndexError, TypeError):
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
                        except (ValueError, TypeError):
                            # Keep a column that is not numeric as it is.
                            pass

                    # If the .data file doesn't exist save the dataframe to create the file
                    # and skip the remainder of the loop
                    if not os.path.exists(file_path):
                        with open(file_path, "wb") as pkl:
                            pickle.dump([df], pkl)
                        continue

                    # If the .data file does exists, append the message's pickle.
                    with open(file_path, "a+b") as pkl:
                        pickle.dump([df], pkl)

            logger.info(f"Finished logging dataframes from run {str(mc_sim_index)}")

        logger.info("Starting to concatenate dataframes")
        for file_path in self._data_files:
            self.finalize(file_path)
        logger.info("Finished concatenating dataframes")

    @staticmethod
    def read_frames(file_path):
        """Yield the frames of one data file, one at a time.

        A file that the writer has not finalized holds one ``[df]`` pickle for each run. A finalized
        file holds one frame for all runs. If the writer stopped in the middle of a pickle, the
        incomplete pickle at the end of the file is dropped.
        """
        with open(file_path, "rb") as pkl:
            while True:
                try:
                    item = pickle.load(pkl)
                except EOFError:
                    return
                except Exception as e:
                    logger.warning(f"Dropping the incomplete end of {file_path} ({type(e).__name__})")
                    return
                if isinstance(item, pd.DataFrame):
                    yield item
                else:
                    yield from item

    @staticmethod
    def is_finalized(file_path):
        """True if the file holds one frame, as the writer leaves it after the shutdown sentinel."""
        with open(file_path, "rb") as pkl:
            try:
                return isinstance(pickle.load(pkl), pd.DataFrame)
            except Exception:
                return False

    @classmethod
    def finalize(cls, file_path):
        """Combine the frames of one data file into one frame with one column for each (runNum, varIdx).

        A run that is missing between the lowest and the highest run index gets NaN columns.
        """
        all_data = pd.concat(list(cls.read_frames(file_path)), axis=1)
        new_mult_ind = pd.MultiIndex.from_product([list(range(all_data.columns.min()[0], all_data.columns.max()[0]+1)),
                                                     list(range(all_data.columns.min()[1], all_data.columns.max()[1]+1))],
                                                     names=["runNum", "varIdx"])
        all_data = all_data.reindex(columns=new_mult_ind)
        all_data.index.name = 'time[ns]'
        all_data.to_pickle(file_path)

    @classmethod
    def runs_in(cls, file_path):
        """The run indexes that have columns in one data file, including the NaN columns of a finalized file."""
        runs = set()
        for frame in cls.read_frames(file_path):
            runs.update(int(run) for run in frame.columns.get_level_values(0))
        return runs

    @classmethod
    def keep_runs(cls, file_path, runs):
        """Rewrite one data file as an unfinalized stream that holds only the given runs, each one time.

        After this, the writer can append more runs to the file and finalize it as usual.
        """
        kept = set()
        temporary_path = file_path + ".partial"
        with open(temporary_path, "wb") as out:
            for frame in cls.read_frames(file_path):
                run_level = frame.columns.get_level_values(0)
                selected = frame.loc[:, run_level.isin(runs - kept)]
                if selected.shape[1]:
                    kept.update(int(run) for run in selected.columns.get_level_values(0))
                    pickle.dump([selected], out)
        os.replace(temporary_path, file_path)

    def set_log_dir(self, log_dir):
        self._log_dir = log_dir

    def set_var_cast(self, var_cast):
        self._var_cast = var_cast
