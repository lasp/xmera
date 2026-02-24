// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module swig_deprecated

/** Used to deprecate a function in C++ that is exposed to Python through SWIG.

'function' is the SWIG identifier of the function. If it is a standalone function,
then this is simply its name. If it's a class function, then this should be
[CLASS_NAME]::[FUNCTION_NAME]

'removal_date' is the expected removal date in the format 'YYYY/MM/DD'. Think of
an amount of time that would let users update their code, and then add that duration
to today's date to find a reasonable removal date.

'message' is a text that is directly shown to the users. Here, you may explain
why the function is deprecated, alternative functions, links to documentation
or scenarios that show how to translate deprecated code...

See src\architecture\utilitiesSelfCheck\swigDeprecatedCheck.i
*/
%define %deprecated_function(function, removal_date, message)
%pythonprepend function %{
    from xmera.utilities import deprecated
    deprecated.deprecationWarn(f"{__name__}.function".replace("::","."), `removal_date`, `message`)
%}
%enddef

/** Used to deprecate a public class variable in C++ that is exposed to Python through SWIG.

'class' is the SWIG identifier of the class.

'variable' is the name of the variable.

'removal_date' is the expected removal date in the format 'YYYY/MM/DD'. Think of
an amount of time that would let users update their code, and then add that duration
to today's date to find a reasonable removal date.

'message' is a text that is directly shown to the users. Here, you may explain
why the variable is deprecated, alternative variables, links to documentation
or scenarios that show how to translate deprecated code...

See src\architecture\utilitiesSelfCheck\swigDeprecatedCheck.i
*/
%define %deprecated_variable(class, variable, removal_date, message)
%extend class {
    %pythoncode %{
    from xmera.utilities import deprecated
    variable = deprecated.DeprecatedProperty(`removal_date`, `message`, variable)
    %}
}
%enddef
