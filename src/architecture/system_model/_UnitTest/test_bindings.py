from xmera.architecture import sim_model

def test_SimModel_bindings():
    """
    Check that the expected FFI bindings for SimModel are exposed
    """
    sim = sim_model.SimModel()

    assert type(sim.addNewProcess("process name")) is sim_model.SysProcess

    assert sim.resetSimulation() is None
    assert sim.singleStepProcesses() is None
    assert sim.stepUntilStop(0) is None

    assert type(sim.getNextTaskTime()) is int
    assert type(sim.getCurrentNanos()) is int
    assert type(sim.getNextProcPriority()) is int
