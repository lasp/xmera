// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module(directors="1", threads="1") sim_model
%{
   #include "sim_model.h"
%}

%include "std_vector.i"
%include "std_string.i"
%include "std_set.i"
%include "std_pair.i"
%include "stdint.i"
%include "carrays.i"
%include "exception.i"
%include "cdata.i"
%include "swig_eigen.i"

%array_functions(bool, boolArray);
%array_functions(uint8_t, cByteArray);

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int, allocator<int> >;
   %template(DoubleVector) vector<double, allocator<double> >;
   %template(MultiArray) vector<vector<double>>;
   %template(StringVector) vector<string, allocator<string> >;
   %template(StringSet) set<string>;
   %template(intSet) set<unsigned long>;
   %template(int64Set) set<long int>;
   %template(ConstCharVector) vector<const char*, allocator<const char*> >;
   %template() std::pair<long int, long int>;
   %template() std::pair<long long int, long long int>;
   %template() std::pair<int64_t, int64_t>;
   %template(exchangeSet) set<pair<long int, long int>>;
   %template(modelPriPair) vector<ModelPriorityPair, allocator<ModelPriorityPair> >;
   %template(procSchedList) vector<ModelScheduleEntry, allocator<ModelScheduleEntry> >;
   %template(simProcList) vector<SysProcess *, allocator<SysProcess *> >;
}

%inline %{
    uint64_t getObjectAddress(void *variable) {
        return (reinterpret_cast<uint64_t> (variable));
    }
%}

%exception {
    try {
        $action
    } catch (const std::exception& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
    } catch (const std::string& e) {
        SWIG_exception(SWIG_RuntimeError, e.c_str());
    }
}

%feature("director") SysModel;
%feature("pythonappend") SysModel::SysModel %{
    self.__super_init_called__ = True%}
%rename("_SysModel") SysModel;

// These three methods block waiting for child simulation threads (via std::binary_semaphore::acquire).
// They must release the GIL before blocking so that SWIG director callbacks in the child
// threads can acquire it. The catch blocks re-acquire the GIL (via Py_BLOCK_THREADS, which
// is valid inside a Py_BEGIN_ALLOW_THREADS scope) before SWIG_exception calls PyErr_SetString.
//
// %nothreadallow cancels the automatic Py_BEGIN/END_ALLOW_THREADS that threads="1" would
// otherwise add around $action. Without it, $action carries its own inner thread-allow
// block, and our outer Py_BEGIN_ALLOW_THREADS causes a double PyEval_SaveThread — calling
// it a second time when the GIL is already released, which is a fatal error in Python 3.12.
%nothreadallow SimModel::selfInitSimulation;
%nothreadallow SimModel::resetInitSimulation;
%nothreadallow SimModel::stepUntilStop;

%define %sim_blocking_exception(method)
%exception method {
    Py_BEGIN_ALLOW_THREADS
    try {
        $action
    } catch (const std::exception& e) {
        Py_BLOCK_THREADS;
        SWIG_exception(SWIG_RuntimeError, e.what());
    } catch (const std::string& e) {
        Py_BLOCK_THREADS;
        SWIG_exception(SWIG_RuntimeError, e.c_str());
    }
    Py_END_ALLOW_THREADS
}
%enddef

%sim_blocking_exception(SimModel::selfInitSimulation)
%sim_blocking_exception(SimModel::resetInitSimulation)
%sim_blocking_exception(SimModel::stepUntilStop)

%include "cSysModel.i"
%include "sys_model_task.h"
%include "sys_process.h"
%include "sim_model.h"
%include "architecture/utilities/bskLogging.h"

%pythoncode %{
class SuperInitChecker(type):

    def __call__(cls, *a, **kw):
        rv = super(SuperInitChecker, cls).__call__(*a, **kw)
        if not getattr(rv, "__super_init_called__", False):
            error_msg = (
               "Need to call parent __init__ in SysModel subclasses:\n"
               f"class {cls.__name__}(sim_model.SysModel):\n"
               "    def __init__(...):\n"
               "        super().__init__()"
            )
            raise SyntaxError(error_msg)
        return rv

class SysModel(_SysModel, metaclass=SuperInitChecker):
    bskLogger: BSKLogger = None
%}


%pythoncode %{
    from xmera.utilities import deprecated
%}

%extend SimModel{
    %pythoncode %{

        @property
        def CurrentNanos(self):
            deprecated.deprecationWarn(
                    "CurrentNanos",
                    "2025/08/01",
                    "Using CurrentNanos is deprecated. Use: getCurrentNanos()\n"
            )
            return self.getCurrentNanos()

        @property
        def NextTaskTime(self):
            deprecated.deprecationWarn(
                    "NextTaskTime",
                    "2025/08/01",
                    "Using NextTaskTime is deprecated. Use: getNextTaskTime()\n"
            )
            return self.getNextTaskTime()

        def getNextTime(self):
            deprecated.deprecationWarn(
                    "getNextTime()",
                    "2025/08/01",
                    "Using getNextTime() is deprecated. Use: getNextTaskTime()\n"
            )
            return self.getNextTaskTime()

        @property
        def nextTaskTime(self):
            deprecated.deprecationWarn(
                    "nextTaskTime",
                    "2025/08/01",
                    "Using nextTaskTime is deprecated. Use: getNextTaskTime()\n"
            )
            return self.getNextTaskTime()

        @property
        def prevRouteTime(self):
            deprecated.deprecationWarn(
                    "prevRouteTime",
                    "2025/08/01",
                    "Using prevRouteTime is deprecated, and always returns 0.\n"
            )
            return 0
    %}
}

%extend SysModelTask {
    %pythoncode %{
        @property
        def NextStartTime(self):
            deprecated.deprecationWarn(
                "NextStartTime",
                "2025/08/01",
                "Using NextStartTime is deprecated. Use: getNextStartTime()\n"
            )
            return self.getNextStartTime()

        @NextStartTime.setter
        def NextStartTime(self, value):
            deprecated.deprecationWarn(
                "NextStartTime",
                "2025/08/01",
                "Using NextStartTime is deprecated. Use: setNextStartTime()\n"
            )
            self.setNextStartTime(value)

        @property
        def TaskPeriod(self):
            deprecated.deprecationWarn(
                "TaskPeriod",
                "2025/08/01",
                "Using TaskPeriod is deprecated. Use: getTaskPeriod()\n"
            )
            return self.getTaskPeriod()

        @TaskPeriod.setter
        def TaskPeriod(self, value):
            deprecated.deprecationWarn(
                "TaskPeriod",
                "2025/08/01",
                "Using TaskPeriod is deprecated. Use: setTaskPeriod()\n"
            )
            self.setTaskPeriod(value)

        @property
        def FirstTaskTime(self):
            deprecated.deprecationWarn(
                "FirstTaskTime",
                "2025/08/01",
                "Using FirstTaskTime is deprecated. Use: getFirstTaskTime()\n"
            )
            return self.getFirstTaskTime()

        @FirstTaskTime.setter
        def FirstTaskTime(self, value):
            deprecated.deprecationWarn(
                "FirstTaskTime",
                "2025/08/01",
                "Using FirstTaskTime is deprecated. Use: setFirstTaskTime()\n"
            )
            self.setFirstTaskTime(value)
    %}
}
