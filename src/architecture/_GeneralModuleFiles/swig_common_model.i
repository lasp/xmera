%module swig_common_model

%include "std_vector.i"
%include "std_string.i"
%include "std_set.i"
%include "std_pair.i"
%include "swig_conly_data.i"
%feature("copyctor");
%array_functions(bool, boolArray);

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int, allocator<int> >;
   %template(DoubleVector) vector<double, allocator<double> >;
   %template(StringVector) vector<string, allocator<string> >;
   %template(StringSet) set<string>;
   %template(intSet) set<unsigned long>;
   %template(ConstCharVector) vector<const char*, allocator<const char*> >;
   %template(MultiArray) vector < vector <double> >;
   %template(MultiArray3d) vector < vector < vector <double> > >;
}

%include "swig_eigen.i"
