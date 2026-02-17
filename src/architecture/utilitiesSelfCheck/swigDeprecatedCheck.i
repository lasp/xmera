// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

%module swigDeprecatedCheck

// The following lines show how to deprecate a stand-alone C++ function,
// an entire C++ class, two C++ class functions, and finally a C++ class variable
%include "swig_deprecated.i"
%deprecated_function(test1, "2023/01/01", "test1 Msg")
%deprecated_function(SwigDeprecatedTestClass::SwigDeprecatedTestClass, "2099/01/01", "class Msg")
%deprecated_function(SwigDeprecatedTestClass::test2, "2023/01/01", "test2 Msg")
%deprecated_function(SwigDeprecatedTestClass::test3, "2099/01/01", "test3 Msg")
%deprecated_variable(SwigDeprecatedTestClass, test4, "2099/01/01", "test4 Msg")

// The following just defines a class and function for us to deprecate
%inline {

void test1(int, double) {};

struct SwigDeprecatedTestClass
{
    void test2() {};
    void test3() {};

    int test4;
};

}
