// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module(directors="1", threads="1") sim_model
%{
   #include "simulation.h"
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
