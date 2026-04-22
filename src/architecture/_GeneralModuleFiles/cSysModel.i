%module(package="xmera.architecture", directors="1", threads="1") cSysModel
%{
   #include <architecture/_GeneralModuleFiles/sys_model.h>
%}

// threads="1" is required so SWIG wraps director callbacks with
// SWIG_PYTHON_THREAD_BEGIN_BLOCK (PyGILState_Ensure), but it also adds
// automatic Py_BEGIN/END_ALLOW_THREADS around every outbound C++ method
// wrapper. When Python code running inside a worker-thread director
// callback calls such a wrapper, the nested Py_BEGIN_ALLOW_THREADS
// double-releases the GIL and aborts the interpreter
// ("Fatal Python error: Aborted"). %nothreadallow disables just the
// outbound auto-release; director callbacks keep their GIL acquisition
// (which would also be disabled by the blunter %nothread).
%nothreadallow;

// Pull in stdint typemaps so C++ members like `int64_t moduleID` and
// `uint64_t currentSimNanos` are wrapped as Python ints rather than raw
// pointer wrappers. We intentionally don't %include swig_conly_data.i
// here — it would cause SWIG to mark swig_conly_data as already-imported
// for downstream filter .i files, silently skipping the %array_functions
// and STRUCTASLIST macros they depend on.
%include <stdint.i>
%include <std_string.i>
%include <architecture/utilities/bskLogging.h>

%feature("director") SysModel;
%feature("pythonappend") SysModel::SysModel %{
    self.__super_init_called__ = True%}
%rename("_SysModel") SysModel;

%include <architecture/_GeneralModuleFiles/sys_model.h>

%pythonbegin %{
from typing import Union, Iterable
%}

%extend SysModel
{
    %pythoncode %{
        def logger(self, variableNames: Union[str, Iterable[str]], recordingTime: int = 0):
            if isinstance(variableNames, str):
                variableNames = [variableNames]

            logging_functions = {
                variable_name: lambda _, vn=variable_name: getattr(self, vn)
                for variable_name in variableNames
            }

            for variable_name, log_fun in logging_functions.items():
                try:
                    log_fun(0)
                except AttributeError:
                    raise ValueError(f"Cannot log {variable_name} as it is not a "
                                    f"variable of {type(self).__name__}")

            from xmera.utilities import pythonVariableLogger
            return pythonVariableLogger.PythonVariableLogger(logging_functions, recordingTime)
    %}
}
