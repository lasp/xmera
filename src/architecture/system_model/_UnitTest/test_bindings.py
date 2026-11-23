from xmera.architecture.sim_model import (
    simulation, group_id, task_id, task_step_list, task_description,
    step_until, step_next_update,
    DEFAULT_GROUP
)

def test_SimModel_bindings():
    """
    Check that the expected FFI bindings for SimModel are exposed
    """
    sim = simulation()
    task = sim.add_task(task_description(task_step_list()))

    assert type(sim.add_task_group()) is group_id
    assert type(task) is task_id

    assert sim.disable(DEFAULT_GROUP) is None
    assert sim.enable(DEFAULT_GROUP) is None
    assert sim.disable(task) is None
    assert sim.enable(task) is None
    assert sim.set_update_period(task, 100) is None
    assert sim.reset() is None
    assert sim.step() is None
    assert step_until(sim, 0) is None
    assert step_next_update(sim) is None

    assert type(sim.next_update()) is int
    assert type(sim.next_priority()) is int
    assert type(sim.priority(DEFAULT_GROUP)) is int
    assert type(sim.priority(task)) is int
