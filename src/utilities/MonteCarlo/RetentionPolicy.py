from dataclasses import dataclass
from xmera.utilities import unitTestSupport

@dataclass
class VariableRetentionParameters:
    """
    Represents a variable's logging parameters.
    """
    var_name:str
    var_rate:int
    start_index:int
    stop_index:int
    var_type:str

    def __init__(self, var_name, var_rate, start_index=0, stop_index=0, var_type='double'):
        self.var_name = var_name
        self.var_rate = var_rate
        self.start_index = start_index
        self.stop_index = stop_index
        self.var_type = var_type

@dataclass
class MessageRetentionParameters:
    """
    Represents a message's logging parameters.
    Args:
        name: name of the message recorder
        retained_vars: the message variable to record
    """
    msg_rec_name:str
    retained_vars:str

    def __init__(self, name, retained_vars):
        self.msg_rec_name = name
        self.retained_vars = retained_vars

class RetentionPolicy:
    """
    This policy controls what simulation data is saved and how it is stored.  Note that the simulation data
    array will have the message time prepended as the first column.
    """

    def __init__(self, rate=int(1E10)):
        self.log_rate = rate
        self.message_log_list = []
        self.var_log_list = []
        self.data_callback = None
        self.retention_functions = []

    def add_message_log(self, name, retained_vars):
        self.message_log_list.append(MessageRetentionParameters(name, retained_vars))

    def add_variable_log(self, variable_name, start_index=0, stop_index=0, var_type='double', log_rate=None):
        if log_rate is None:
            log_rate = self.log_rate
        var_container = VariableRetentionParameters(variable_name, log_rate, start_index, stop_index, var_type)
        self.var_log_list.append(var_container)

    def add_logs_to_sim(self, sim_instance):
        for variable in self.var_log_list:
            sim_instance.AddVariableForMultiProcessLogging(variable.var_name,
                                                           variable.var_rate,
                                                           variable.start_index,
                                                           variable.stop_index,
                                                           variable.var_type)



    def add_retention_function(self, function):
        self.retention_functions.append(function)

    def set_data_callback(self, data_callback):
        self.data_callback = data_callback

    def execute_callback(self, data):
        if self.data_callback is not None:
            self.data_callback(data, self)

    @staticmethod
    def add_retention_policies_to_sim(sim_instance, retention_policies):
        """ Adds logs for variables and messages to a sim_instance
        Args:
            sim_instance: The simulation instance to add logs to.
            retention_policies: RetentionPolicy[] list that defines the data to log.
        """

        for retention_policy in retention_policies:
            retention_policy.add_logs_to_sim(sim_instance)

        # TODO handle duplicates somehow?

    @staticmethod
    def get_data_for_retention(sim_instance, retention_policies):
        """ Returns the data that should be retained given a sim_instance and the retention_policies

        Args:
            sim_instance: The simulation instance to retrieve data from
            retention_policies: A list of RetentionPolicy objects defining the data to retain

        Returns:
            Retained Data in the form of a dictionary with two sub-dictionaries for messages and variables::

                {
                    "messages": {
                        "messageName": [value1,value2,value3]
                    },
                    "variables": {
                        "variable_name": [value1,value2,value3]
                    }
                }
        """
        data = {"messages": {}, "variables": {}, "custom": {} }

        for retention_policy in retention_policies:
            for msg_param in retention_policy.message_log_list:

                # record the message recording times
                msg_times = sim_instance.msgRecList[msg_param.msg_rec_name].times()

                # record the message variables
                for var_name in msg_param.retained_vars:
                    # To ensure the current datashaders utilities continue to work, the
                    # retained data is combined with the time information as it was in
                    # BSK1.x releases.
                    msg_data = getattr(sim_instance.msgRecList[msg_param.msg_rec_name], var_name)
                    msg_data = unitTestSupport.addTimeColumn(msg_times, msg_data)
                    data["messages"][msg_param.msg_rec_name + "." + var_name] = msg_data

            for variable in retention_policy.var_log_list:
                data["variables"][variable.var_name] = sim_instance.GetMultiProcessLoggerVariableData(variable.var_name)

            for func in retention_policy.retention_functions:
                tmp_module_data = func(sim_instance)
                for (key, value) in tmp_module_data.items():
                    data["custom"][key] = value
        return data
