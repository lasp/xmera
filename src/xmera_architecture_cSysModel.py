# cSysModel is a SWIG module whose contents are compiled into _sim_model.pyd.
# This shim re-exports those classes so that cross-module SWIG-generated code
# (e.g. `class GravityEffector(cSysModel.SysModel)`) can find them.
#
# IMPORTANT: export _SysModel (the raw SWIG director class), NOT sim_model.SysModel.
# sim_model.SysModel adds the SuperInitChecker metaclass, which requires Python
# subclasses to call super().__init__(). SWIG-generated subclasses never do this —
# they call the C++ constructor directly via _module.new_ClassName(). The real
# cSysModel SWIG module would have exposed the raw class without SuperInitChecker.
from xmera.architecture.sim_model import _SysModel as SysModel, BSKLogger