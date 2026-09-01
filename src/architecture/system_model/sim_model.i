// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module(directors="1", threads="1") sim_model
%{
   #include "simulation.h"
   #include "architecture/utilities/logging.h"
%}

%include <attribute.i>
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
   %template(task_step_list) vector<SysModel*, allocator<SysModel*> >;
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

%include "cSysModel.i"
%include "simulation.h"

%inline %{
    void _setGlobalLogLevel(std::string const& level) {
        xmera::logging::setLevel(xmera::logging::parseLevel(level));
    }

    void _setNamedLogLevel(std::string const& name, std::string const& level) {
        xmera::logging::setLevel(name, xmera::logging::parseLevel(level));
    }

    void reset_log_levels() {
        xmera::logging::resetLevels();
    }

    void set_log_sink(std::string const& spec) {
        xmera::logging::setSink(xmera::logging::parseSink(spec));
    }

    void _log(std::string const& name, std::string const& level, std::string const& message) {
        xmera::logging::get(name)->log(xmera::logging::parseLevel(level), message);
    }
%}

%pythoncode %{
def set_log_level(level, name=None):
    """Set the log level: trace, debug, info, warning, error, critical or off.

    If name is None, the level applies to all loggers that do not have a level set by name.
    Otherwise it applies to the logger with this name, usually a module modelTag.
    """
    if name is None:
        _setGlobalLogLevel(level)
    else:
        _setNamedLogLevel(name, level)


def log(level, message, name=""):
    """Write a message at a level (trace, debug, info, warning, error or critical) to the logger with this name."""
    _log(name, level, message)
%}

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

    def log(self, level, message):
        """Write a message at a level to the logger named after the modelTag of this module."""
        log(level, message, self.modelTag)
%}
