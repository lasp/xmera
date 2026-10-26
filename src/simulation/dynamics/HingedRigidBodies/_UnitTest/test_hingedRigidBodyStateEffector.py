# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os

import numpy
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from xmera.simulation import spacecraft
from xmera.simulation import hingedRigidBodyStateEffector
from xmera.utilities import macros
from xmera.utilities import pythonVariableLogger
from xmera.simulation import gravityEffector
from xmera.simulation import extForceTorque
from xmera.architecture import messaging


def test_hinged_rigid_body_gravity(show_plots):
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.001)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Create two hinged rigid bodies
    unit_test_sim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unit_test_sim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unit_test_sim.panel1.mass = 100.0
    unit_test_sim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel1.d = 1.5
    unit_test_sim.panel1.k = 100.0
    unit_test_sim.panel1.c = 0.0
    unit_test_sim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unit_test_sim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unit_test_sim.panel1.thetaInit = 5*numpy.pi/180.0
    unit_test_sim.panel1.thetaDotInit = 0.0
    unit_test_sim.panel1.modelTag = "Panel1"

    # Define Variables for panel 2
    unit_test_sim.panel2.mass = 100.0
    unit_test_sim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel2.d = 1.5
    unit_test_sim.panel2.k = 100.0
    unit_test_sim.panel2.c = 0.0
    unit_test_sim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unit_test_sim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unit_test_sim.panel2.thetaInit = 0.0
    unit_test_sim.panel2.thetaDotInit = 0.0
    unit_test_sim.panel2.modelTag = "Panel2"

    # Add panels to spaceCraft
    sc_object.addStateEffector(unit_test_sim.panel1)
    sc_object.addStateEffector(unit_test_sim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    sc_object.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel1)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel2)

    # Add Earth gravity to the sim
    unit_test_sim.earthGravBody = gravityEffector.GravBodyData()
    unit_test_sim.earthGravBody.planetName = "earth_planet_data"
    unit_test_sim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unit_test_sim.earthGravBody.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([unit_test_sim.earthGravBody])

    # Log the spacecraft state message
    dat_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, dat_log)

    # Add energy and momentum variables to log
    panel1_log = unit_test_sim.panel1.logger("forceOnBody_B")
    panel2_log = unit_test_sim.panel2.logger("forceOnBody_B")
    sc_log = pythonVariableLogger.PythonVariableLogger({
        "totOrbEnergy": lambda _: sc_object.totOrbEnergy,
        "totOrbAngMomPntN_N": lambda _: sc_object.totOrbAngMomPntN_N,
        "totRotAngMomPntC_N": lambda _: sc_object.totRotAngMomPntC_N,
        "totRotEnergy": lambda _: sc_object.totRotEnergy,
    })
    unit_test_sim.AddModelToTask(unit_task_name, panel1_log)
    unit_test_sim.AddModelToTask(unit_task_name, panel2_log)
    unit_test_sim.AddModelToTask(unit_task_name, sc_log)

    # Initialize the simulation
    unit_test_sim.InitializeSimulation()

    stop_time = 2.5
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    sigma_out = dat_log.sigma_BN

    force_panel1 = unitTestSupport.addTimeColumn(panel1_log.times(), panel1_log.forceOnBody_B)
    force_panel2 = unitTestSupport.addTimeColumn(panel2_log.times(), panel2_log.forceOnBody_B)
    orb_energy = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totOrbEnergy)
    orb_ang_mom_N = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totOrbAngMomPntN_N)
    rot_ang_mom_N = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totRotAngMomPntC_N)
    rot_energy = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totRotEnergy)

    data_sigma = [sigma_out[-1]]

    true_sigma = [[0.06170318243240492, -0.07089090074412899, 0.06409500412692531]]

    initial_orb_ang_mom_N = [[orb_ang_mom_N[0,1], orb_ang_mom_N[0,2], orb_ang_mom_N[0,3]]]

    final_orb_ang_mom = [orb_ang_mom_N[-1]]

    initial_rot_ang_mom_N = [[rot_ang_mom_N[0,1], rot_ang_mom_N[0,2], rot_ang_mom_N[0,3]]]

    final_rot_ang_mom = [rot_ang_mom_N[-1]]

    initial_orb_energy = [[orb_energy[0,1]]]

    final_orb_energy = [orb_energy[-1]]

    initial_rot_energy = [[rot_energy[0,1]]]

    final_rot_energy = [rot_energy[-1]]

    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,1] - orb_ang_mom_N[0,1])/orb_ang_mom_N[0,1], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,2] - orb_ang_mom_N[0,2])/orb_ang_mom_N[0,2], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,3] - orb_ang_mom_N[0,3])/orb_ang_mom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInOrbitalAngularMomentumGravity"
    plot_title = "Change in Orbital Angular Momentum with Gravity"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orb_energy[:,0]*1e-9, (orb_energy[:,1] - orb_energy[0,1])/orb_energy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInOrbitalEnergyGravity"
    plot_title = "Change in Orbital Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,1] - rot_ang_mom_N[0,1])/rot_ang_mom_N[0,1], rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,2] - rot_ang_mom_N[0,2])/rot_ang_mom_N[0,2], rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,3] - rot_ang_mom_N[0,3])/rot_ang_mom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInRotationalAngularMomentumGravity"
    plot_title = "Change In Rotational Angular Momentum with Gravity"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rot_energy[:,0]*1e-9, (rot_energy[:,1] - rot_energy[0,1])/rot_energy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInRotationalEnergyGravity"
    plot_title = "Change In Rotational Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(force_panel1[:,0]*1e-9, force_panel1[:,1], force_panel1[:,0]*1e-9, force_panel1[:,2], force_panel1[:,0]*1e-9, force_panel1[:,3])
    plt.xlabel('time (s)')
    plt.ylabel('Force about Point B')

    plt.figure()
    plt.clf()
    plt.plot(force_panel2[:,0]*1e-9, force_panel2[:,1], force_panel2[:,0]*1e-9, force_panel2[:,2], force_panel2[:,0]*1e-9, force_panel2[:,3])
    plt.xlabel('time (s)')
    plt.ylabel('Force about Point B')

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 1e-10
    numpy.testing.assert_allclose(numpy.array(true_sigma),
                                  numpy.array(data_sigma),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed gravity attitude test")

    final_orb_ang_mom = numpy.delete(final_orb_ang_mom, 0, axis=1)  # remove time column
    final_rot_ang_mom = numpy.delete(final_rot_ang_mom, 0, axis=1)  # remove time column
    final_rot_energy = numpy.delete(final_rot_energy, 0, axis=1)  # remove time column
    final_orb_energy = numpy.delete(final_orb_energy, 0, axis=1)  # remove time column

    numpy.testing.assert_allclose(numpy.array(initial_orb_ang_mom_N),
                                 numpy.array(final_orb_ang_mom),
                                 rtol=accuracy,
                                 err_msg="Hinged Rigid Body integrated test failed gravity orbital angular momentum "
                                         "unit test")

    numpy.testing.assert_allclose(numpy.array(initial_rot_ang_mom_N),
                                 numpy.array(final_rot_ang_mom),
                                 rtol=accuracy,
                                 err_msg="Hinged Rigid Body integrated test failed gravity rotational angular momentum "
                                       "unit test")

    numpy.testing.assert_allclose(numpy.array(initial_rot_energy),
                                 numpy.array(final_rot_energy),
                                 rtol=accuracy,
                                 err_msg="Hinged Rigid Body integrated test failed gravity rotational energy unit test")

    numpy.testing.assert_allclose(numpy.array(initial_orb_energy),
                                 numpy.array(final_orb_energy),
                                 rtol=accuracy,
                                 err_msg="Hinged Rigid Body integrated test failed gravity orbital energy unit test")


@pytest.mark.xfail(strict=True, reason="Gravity cancellation in the back-substitution leaves round-off")
def test_hinged_rigid_body_gravity_invariance(show_plots):
    """A spacecraft at rest in free fall must stay at rest: uniform gravity must not
    rotate the hub or swing the panels."""
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess("TestProcess")
    test_proc.addTask(unit_test_sim.CreateNewTask("unitTask", macros.sec2nano(1.0)))

    sc_object = spacecraft.Spacecraft()
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    unit_test_sim.AddModelToTask("unitTask", sc_object)

    panels = []
    for r_HB_B, dcm_HB in [([[0.5], [0.0], [1.0]], [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]),
                           ([[-0.5], [0.0], [1.0]], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])]:
        panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
        panel.mass = 100.0
        panel.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
        panel.d = 1.5
        panel.k = 100.0
        panel.r_HB_B = r_HB_B
        panel.dcm_HB = dcm_HB
        sc_object.addStateEffector(panel)
        unit_test_sim.AddModelToTask("unitTask", panel)
        panels.append(panel)

    earth_grav_body = gravityEffector.GravBodyData()
    earth_grav_body.planetName = "earth_planet_data"
    earth_grav_body.mu = 0.3986004415E+15  # meters!
    earth_grav_body.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earth_grav_body])

    sc_log = sc_object.scStateOutMsg.recorder()
    panel_logs = [panel.hingedRigidBodyOutMsg.recorder() for panel in panels]
    for log in [sc_log] + panel_logs:
        unit_test_sim.AddModelToTask("unitTask", log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1000.0))
    unit_test_sim.ExecuteSimulation()

    times = sc_log.times() * macros.NANO2SEC
    states = {
        "sigma_BN": sc_log.sigma_BN,
        "omega_BN_B": sc_log.omega_BN_B,
        "theta": numpy.column_stack([log.theta for log in panel_logs]),
        "thetaDot": numpy.column_stack([log.thetaDot for log in panel_logs]),
    }

    plt.close("all")
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
    for ax, (name, values) in zip(axes, states.items()):
        ax.plot(times, values)
        ax.set_ylabel(name)
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    if show_plots:
        plt.show()
    plt.close("all")

    for name, values in states.items():
        numpy.testing.assert_array_equal(values, numpy.zeros_like(values), err_msg=name)


def test_hinged_rigid_body_no_gravity(show_plots):
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.001)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    unit_test_sim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unit_test_sim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unit_test_sim.panel1.mass = 100.0
    unit_test_sim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel1.d = 1.5
    unit_test_sim.panel1.k = 100.0
    unit_test_sim.panel1.c = 0.0
    unit_test_sim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unit_test_sim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unit_test_sim.panel1.thetaInit = 5*numpy.pi/180.0
    unit_test_sim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unit_test_sim.panel2.mass = 100.0
    unit_test_sim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel2.d = 1.5
    unit_test_sim.panel2.k = 100.0
    unit_test_sim.panel2.c = 0.0
    unit_test_sim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unit_test_sim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unit_test_sim.panel2.thetaInit = 0.0
    unit_test_sim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    sc_object.addStateEffector(unit_test_sim.panel1)
    sc_object.addStateEffector(unit_test_sim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    sc_object.hub.r_CN_NInit = [[0.1], [-0.4], [0.3]]
    sc_object.hub.v_CN_NInit = [[-0.2], [0.5], [0.1]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel1)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel2)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    sc_log = pythonVariableLogger.PythonVariableLogger({
        "totOrbEnergy": lambda _: sc_object.totOrbEnergy,
        "totOrbAngMomPntN_N": lambda _: sc_object.totOrbAngMomPntN_N,
        "totRotAngMomPntC_N": lambda _: sc_object.totRotAngMomPntC_N,
        "totRotEnergy": lambda _: sc_object.totRotEnergy,
    })
    unit_test_sim.AddModelToTask(unit_task_name, sc_log)

    unit_test_sim.InitializeSimulation()

    stop_time = 2.5
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    sigma_out = data_log.sigma_BN
    r_out_BN_N = data_log.r_BN_N
    v_out_CN_N = data_log.v_CN_N

    orb_energy = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totOrbEnergy)
    orb_ang_mom_N = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totOrbAngMomPntN_N)
    rot_ang_mom_N = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totRotAngMomPntC_N)
    rot_energy = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totRotEnergy)

    # Get the last sigma and position
    data_sigma = [sigma_out[-1]]
    data_pos = [r_out_BN_N[-1]]

    true_pos = [[-0.15832794740648992, 1.122481716747217, -0.37975995949382907]]

    true_sigma = [[0.06170318243240492, -0.07089090074412899, 0.06409500412692531]]

    initial_orb_ang_mom_N = [[orb_ang_mom_N[0,1], orb_ang_mom_N[0,2], orb_ang_mom_N[0,3]]]

    final_orb_ang_mom = [orb_ang_mom_N[-1]]

    initial_rot_ang_mom_N = [[rot_ang_mom_N[0,1], rot_ang_mom_N[0,2], rot_ang_mom_N[0,3]]]

    final_rot_ang_mom = [rot_ang_mom_N[-1]]

    initial_orb_energy = [[orb_energy[0,1]]]

    final_orb_energy = [orb_energy[-1]]

    initial_rot_energy = [[rot_energy[0,1]]]

    final_rot_energy = [rot_energy[-1]]

    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,1] - orb_ang_mom_N[0,1])/orb_ang_mom_N[0,1], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,2] - orb_ang_mom_N[0,2])/orb_ang_mom_N[0,2], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,3] - orb_ang_mom_N[0,3])/orb_ang_mom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInOrbitalAngularMomentumNoGravity"
    plot_title = "Change in Orbital Angular Momentum No Gravity"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orb_energy[:,0]*1e-9, (orb_energy[:,1] - orb_energy[0,1])/orb_energy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInOrbitalEnergyNoGravity"
    plot_title = "Change in Orbital Energy No Gravity"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,1] - rot_ang_mom_N[0,1])/rot_ang_mom_N[0,1], rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,2] - rot_ang_mom_N[0,2])/rot_ang_mom_N[0,2], rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,3] - rot_ang_mom_N[0,3])/rot_ang_mom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInRotationalAngularMomentumNoGravity"
    plot_title = "Change In Rotational Angular Momentum No Gravity"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rot_energy[:,0]*1e-9, (rot_energy[:,1] - rot_energy[0,1])/rot_energy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInRotationalEnergyNoGravity"
    plot_title = "Change In Rotational Energy No Gravity"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(data_log.times()*1e-9, v_out_CN_N[:,0], data_log.times()*1e-9, v_out_CN_N[:,1], data_log.times()*1e-9, v_out_CN_N[:,2])
    plt.xlabel('time (s)')
    plt.ylabel('m/s')
    plot_name = "VelocityOfCenterOfMassNoGravity"
    plot_title = "Velocity Of Center Of Mass No Gravity"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(data_log.times()*1e-9, (v_out_CN_N[:,0] - v_out_CN_N[:,0])/v_out_CN_N[:,0], data_log.times()*1e-9,
             (v_out_CN_N[:,1] - v_out_CN_N[:,1])/v_out_CN_N[:,1], data_log.times()*1e-9,
             (v_out_CN_N[:,2] - v_out_CN_N[:,2])/v_out_CN_N[:,2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInVelocityOfCenterOfMassNoGravity"
    plot_title = "Change In Velocity Of Center Of Mass No Gravity"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 1e-10
    numpy.testing.assert_allclose(numpy.array(true_pos),
                                  numpy.array(data_pos),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed position test")

    numpy.testing.assert_allclose(numpy.array(true_sigma),
                                  numpy.array(data_sigma),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed attitude test")

    final_orb_ang_mom = numpy.delete(final_orb_ang_mom, 0, axis=1)  # remove time column
    final_rot_ang_mom = numpy.delete(final_rot_ang_mom, 0, axis=1)  # remove time column
    final_rot_energy = numpy.delete(final_rot_energy, 0, axis=1)  # remove time column
    final_orb_energy = numpy.delete(final_orb_energy, 0, axis=1)  # remove time column


    numpy.testing.assert_allclose(numpy.array(initial_orb_ang_mom_N),
                                 numpy.array(final_orb_ang_mom),
                                 rtol=accuracy,
                                 err_msg="Hinged Rigid Body integrated test failed orbital angular momentum unit "
                                       "test")

    numpy.testing.assert_allclose(numpy.array(initial_rot_ang_mom_N),
                                  numpy.array(final_rot_ang_mom),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed rotational angular momentum unit "
                                          "test")

    numpy.testing.assert_allclose(numpy.array(initial_rot_energy),
                                  numpy.array(final_rot_energy),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed rotational energy unit test")

    numpy.testing.assert_allclose(numpy.array(initial_orb_energy),
                                  numpy.array(final_orb_energy),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed orbital energy unit test")


def test_hinged_rigid_body_no_gravity_damping(show_plots):
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.001)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    unit_test_sim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unit_test_sim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unit_test_sim.panel1.mass = 100.0
    unit_test_sim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel1.d = 1.5
    unit_test_sim.panel1.k = 100.0
    unit_test_sim.panel1.c = 6.0
    unit_test_sim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unit_test_sim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unit_test_sim.panel1.thetaInit = 5*numpy.pi/180.0
    unit_test_sim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unit_test_sim.panel2.mass = 100.0
    unit_test_sim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel2.d = 1.5
    unit_test_sim.panel2.k = 100.0
    unit_test_sim.panel2.c = 7.0
    unit_test_sim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unit_test_sim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unit_test_sim.panel2.thetaInit = 0.0
    unit_test_sim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    sc_object.addStateEffector(unit_test_sim.panel1)
    sc_object.addStateEffector(unit_test_sim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    sc_object.hub.r_CN_NInit = [[0.1], [-0.4], [0.3]]
    sc_object.hub.v_CN_NInit = [[-0.2], [0.5], [0.1]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel1)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel2)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    sc_log = pythonVariableLogger.PythonVariableLogger({
        "totOrbEnergy": lambda _: sc_object.totOrbEnergy,
        "totOrbAngMomPntN_N": lambda _: sc_object.totOrbAngMomPntN_N,
        "totRotAngMomPntC_N": lambda _: sc_object.totRotAngMomPntC_N,
    })
    unit_test_sim.AddModelToTask(unit_task_name, sc_log)

    unit_test_sim.InitializeSimulation()

    stop_time = 2.5
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    v_out_CN_N = data_log.v_CN_N

    orb_energy = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totOrbEnergy)
    orb_ang_mom_N = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totOrbAngMomPntN_N)
    rot_ang_mom_N = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totRotAngMomPntC_N)

    initial_orb_ang_mom_N = [[orb_ang_mom_N[0,1], orb_ang_mom_N[0,2], orb_ang_mom_N[0,3]]]

    final_orb_ang_mom = [orb_ang_mom_N[-1]]

    initial_rot_ang_mom_N = [[rot_ang_mom_N[0,1], rot_ang_mom_N[0,2], rot_ang_mom_N[0,3]]]

    final_rot_ang_mom = [rot_ang_mom_N[-1]]

    initial_orb_energy = [[orb_energy[0,1]]]

    final_orb_energy = [orb_energy[-1]]

    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,1] - orb_ang_mom_N[0,1])/orb_ang_mom_N[0,1], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,2] - orb_ang_mom_N[0,2])/orb_ang_mom_N[0,2], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,3] - orb_ang_mom_N[0,3])/orb_ang_mom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInOrbitalAngularMomentumNoGravityDamping"
    plot_title = "Change in Orbital Angular Momentum No Gravity with Damping"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orb_energy[:,0]*1e-9, (orb_energy[:,1] - orb_energy[0,1])/orb_energy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInOrbitalEnergyNoGravityDamping"
    plot_title = "Change in Orbital Energy No Gravity with Damping"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,1] - rot_ang_mom_N[0,1])/rot_ang_mom_N[0,1], rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,2] - rot_ang_mom_N[0,2])/rot_ang_mom_N[0,2], rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,3] - rot_ang_mom_N[0,3])/rot_ang_mom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInRotationalAngularMomentumNoGravityDamping"
    plot_title = "Change In Rotational Angular Momentum No Gravity with Damping"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(data_log.times()*1e-9, v_out_CN_N[:, 0],
             data_log.times()*1e-9, v_out_CN_N[:, 1],
             data_log.times()*1e-9, v_out_CN_N[:, 2])
    plt.xlabel('time (s)')
    plt.ylabel('m/s')
    plot_name = "VelocityOfCenterOfMassNoGravityDamping"
    plot_title = "Velocity Of Center Of Mass No Gravity with Damping"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(data_log.times()*1e-9, (v_out_CN_N[:, 0] - v_out_CN_N[:, 0])/v_out_CN_N[:, 0],
             data_log.times()*1e-9, (v_out_CN_N[:, 1] - v_out_CN_N[:, 1])/v_out_CN_N[:, 1],
             data_log.times()*1e-9, (v_out_CN_N[:, 2] - v_out_CN_N[:, 2])/v_out_CN_N[:, 2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plot_name = "ChangeInVelocityOfCenterOfMassNoGravityDamping"
    plot_title = "Change In Velocity Of Center Of Mass No Gravity with Damping"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    if show_plots:
        plt.show()
    plt.close("all")

    final_orb_ang_mom = numpy.delete(final_orb_ang_mom, 0, axis=1)  # remove time column
    final_rot_ang_mom = numpy.delete(final_rot_ang_mom, 0, axis=1)  # remove time column
    final_orb_energy = numpy.delete(final_orb_energy, 0, axis=1)  # remove time column

    accuracy = 1e-10
    numpy.testing.assert_allclose(numpy.array(initial_orb_ang_mom_N),
                                  numpy.array(final_orb_ang_mom),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test with damping failed orbital angular momentum unit test")

    numpy.testing.assert_allclose(numpy.array(initial_rot_ang_mom_N),
                                  numpy.array(final_rot_ang_mom),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test with damping failed rotational angular momentum unit test")

    numpy.testing.assert_allclose(numpy.array(initial_orb_energy),
                                  numpy.array(final_orb_energy),
                                  rtol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test with damping failed orbital energy unit test")


def test_hinged_rigid_body_theta_ss(show_plots):
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    step_size = 0.1
    test_process_rate = macros.sec2nano(step_size)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    unit_test_sim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unit_test_sim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unit_test_sim.panel1.mass = 100.0
    unit_test_sim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel1.d = 1.5
    unit_test_sim.panel1.k = 100.0
    unit_test_sim.panel1.c = 75
    unit_test_sim.panel1.r_HB_B = [[0.5], [1.0], [0.0]]
    unit_test_sim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]
    unit_test_sim.panel1.thetaInit = 0.0
    unit_test_sim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unit_test_sim.panel2.mass = 100.0
    unit_test_sim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel2.d = 1.5
    unit_test_sim.panel2.k = 100.0
    unit_test_sim.panel2.c = 75
    unit_test_sim.panel2.r_HB_B = [[-0.5], [1.0], [0.0]]
    unit_test_sim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    unit_test_sim.panel2.thetaInit = 0.0
    unit_test_sim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    sc_object.addStateEffector(unit_test_sim.panel1)
    sc_object.addStateEffector(unit_test_sim.panel2)

    # Add external force and torque
    ext_ft_object = extForceTorque.ExtForceTorque()
    ext_ft_object.modelTag = "externalDisturbance"
    ext_ft_object.extTorquePntB_B = [[0], [0], [0]]
    ext_ft_object.extForce_B = [[0], [1], [0]]
    sc_object.addDynamicEffector(ext_ft_object)
    unit_test_sim.AddModelToTask(unit_task_name, ext_ft_object)

    # Define mass properties of the rigid part of the spacecraft
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    sc_object.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel1)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel2)

    state_log = pythonVariableLogger.PythonVariableLogger({
        "theta1": lambda _: sc_object.dynManager.getStateObject(unit_test_sim.panel1.nameOfThetaState).getState(),
        "theta2": lambda _: sc_object.dynManager.getStateObject(unit_test_sim.panel2.nameOfThetaState).getState(),
    })
    unit_test_sim.AddModelToTask(unit_task_name, state_log)

    unit_test_sim.InitializeSimulation()

    stop_time = 60.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    theta1_out = unitTestSupport.addTimeColumn(state_log.times(), state_log.theta1)
    theta2_out = unitTestSupport.addTimeColumn(state_log.times(), state_log.theta2)

    # Developing the lagrangian result
    # Define initial values
    spacecraft_data = SpacecraftClass()
    spacecraft_data.hub.mass = sc_object.hub.mHub
    spacecraft_data.hub.inertia = sc_object.hub.IHubPntBc_B[2][2]
    # Define variables for panel1
    spacecraft_data.panel1.mass = unit_test_sim.panel1.mass
    spacecraft_data.panel1.inertia = unit_test_sim.panel1.IPntS_S[1][1]
    spacecraft_data.panel1.r_hinge = numpy.linalg.norm(numpy.asarray(unit_test_sim.panel1.r_HB_B))
    spacecraft_data.panel1.beta = numpy.arctan2(unit_test_sim.panel1.r_HB_B[1][0],unit_test_sim.panel1.r_HB_B[0][0])
    spacecraft_data.panel1.theta_h = 0.0
    spacecraft_data.panel1.d = unit_test_sim.panel1.d
    spacecraft_data.panel1.k = unit_test_sim.panel1.k
    spacecraft_data.panel1.c = unit_test_sim.panel1.c
    # Define variables for panel2
    spacecraft_data.panel2.mass = unit_test_sim.panel2.mass
    spacecraft_data.panel2.inertia = unit_test_sim.panel2.IPntS_S[1][1]
    spacecraft_data.panel2.r_hinge = numpy.linalg.norm(numpy.asarray(unit_test_sim.panel2.r_HB_B))
    spacecraft_data.panel2.beta = numpy.arctan2(unit_test_sim.panel2.r_HB_B[1][0],unit_test_sim.panel2.r_HB_B[0][0])
    spacecraft_data.panel2.theta_h = numpy.pi
    spacecraft_data.panel2.d = unit_test_sim.panel2.d
    spacecraft_data.panel2.k = unit_test_sim.panel2.k
    spacecraft_data.panel2.c = unit_test_sim.panel2.c
    # Define body force and torque
    spacecraft_data.x_thrust_B = 0.0
    spacecraft_data.y_thrust_B = ext_ft_object.extForce_B[1][0]
    spacecraft_data.torque = 0.0

    # Define initial conditions of the sim
    time = numpy.arange(0.0,stop_time + step_size,step_size).flatten()
    x0 = numpy.zeros(10)
    x0[3] = unit_test_sim.panel1.thetaInit
    x0[4] = -unit_test_sim.panel2.thetaInit

    X = numpy.zeros((len(x0),len(time)))
    X[:,0] = x0
    for j in range (1,(len(time))):
        X[:, j] = rk4(planar_flex_function, X[:, j-1], step_size, time[j-1], spacecraft_data)

    # Find steady state value
    variables_in = BoxAndWingParameters()
    variables_in.k = spacecraft_data.panel1.k
    variables_in.d = spacecraft_data.panel1.d
    variables_in.F = spacecraft_data.y_thrust_B
    variables_in.m_sc = spacecraft_data.hub.mass + spacecraft_data.panel1.mass + spacecraft_data.panel2.mass
    variables_in.m_sp = spacecraft_data.panel1.mass
    theta_ss_guess = -0.01
    tolerance = 1e-10
    theta_ss = newton_rapshon(box_and_wings_f_and_f_prime,theta_ss_guess,tolerance,variables_in)

    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(time, X[3,:],'-b',label = "Lagrangian")
    plt.plot(theta1_out[:,0]*1e-9, theta1_out[:,1],'-r',label = "Xmera")
    plt.plot(theta1_out[-1,0]*1e-9, theta_ss,'ok',label = "BOE Calculation")
    plt.xlabel('time (s)')
    plt.ylabel('theta 1 (rad)')
    plt.legend(loc ='upper right',numpoints = 1)
    plot_name = "BOECalculationForSteadyStateTheta1DeflectionVsSimulation"
    plot_title = "BOE Calculation for Steady State Theta 1 Deflection vs Simulation"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, -X[4,:],'-b',label = "Lagrangian")
    plt.plot(theta2_out[:,0]*1e-9, theta2_out[:,1],'-r',label = "Xmera")
    plt.plot(theta2_out[-1,0]*1e-9, theta_ss,'ok',label = "BOE Calculation")
    plt.xlabel('time (s)')
    plt.ylabel('theta 2 (rad)')
    plt.legend(loc ='upper right',numpoints = 1)
    plot_name = "BOECalculationForSteadyStateTheta2DeflectionVsSimulation"
    plot_title = "BOE Calculation for Steady State Theta 2 Deflection vs Simulation"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    if show_plots:
        plt.show()
    plt.close("all")


    accuracy = 1e-6
    assert abs(theta1_out[-1,1] - theta_ss) <= accuracy, ("Hinged Rigid Body integrated steady state test failed theta 1 "
                                                        "comparison ")

    assert abs(theta2_out[-1,1] - theta_ss) <= accuracy, ("Hinged Rigid Body integrated steady state test failed theta 2 "
                                                        "comparison ")


def test_hinged_rigid_body_frequency_amp(show_plots):
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    step_size = 0.1
    test_process_rate = macros.sec2nano(step_size)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    unit_test_sim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unit_test_sim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unit_test_sim.panel1.mass = 100.0
    unit_test_sim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel1.d = 1.5
    unit_test_sim.panel1.k = 300.0
    unit_test_sim.panel1.c = 0.0
    unit_test_sim.panel1.r_HB_B = [[0.5], [1.0], [0.0]]
    unit_test_sim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]
    unit_test_sim.panel1.thetaInit = 0.0
    unit_test_sim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unit_test_sim.panel2.mass = 100.0
    unit_test_sim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel2.d = 1.5
    unit_test_sim.panel2.k = 300.0
    unit_test_sim.panel2.c = 0.0
    unit_test_sim.panel2.r_HB_B = [[-0.5], [1.0], [0.0]]
    unit_test_sim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    unit_test_sim.panel2.thetaInit = 0.0
    unit_test_sim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    sc_object.addStateEffector(unit_test_sim.panel1)
    sc_object.addStateEffector(unit_test_sim.panel2)

    # Add external force and torque
    ext_ft_object = extForceTorque.ExtForceTorque()
    ext_ft_object.modelTag = "externalDisturbance"
    ext_ft_object.extTorquePntB_B = [[0], [0], [0]]
    force = 1
    ext_ft_object.extForce_B = [[0], [force], [0]]
    sc_object.addDynamicEffector(ext_ft_object)
    unit_test_sim.AddModelToTask(unit_task_name, ext_ft_object)

    # Define mass properties of the rigid part of the spacecraft
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    sc_object.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel1)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel2)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    state_log = pythonVariableLogger.PythonVariableLogger({
        "theta1": lambda _: sc_object.dynManager.getStateObject(unit_test_sim.panel1.nameOfThetaState).getState(),
        "theta2": lambda _: sc_object.dynManager.getStateObject(unit_test_sim.panel2.nameOfThetaState).getState(),
    })
    unit_test_sim.AddModelToTask(unit_task_name, state_log)

    unit_test_sim.InitializeSimulation()

    stop_time = 58
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time/2))
    unit_test_sim.ExecuteSimulation()

    ext_ft_object.extTorquePntB_B = [0.0, 0.0, 0.0]
    ext_ft_object.extForce_B = [0.0, 0.0, 0.0]

    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    r_out_BN_N = data_log.r_BN_N
    sigma_out_BN = data_log.sigma_BN
    theta_out = 4.0*numpy.arctan(sigma_out_BN[:,2])

    theta1_out = unitTestSupport.addTimeColumn(state_log.times(), state_log.theta1)
    theta2_out = unitTestSupport.addTimeColumn(state_log.times(), state_log.theta2)

    # Developing the lagrangian result
    # Define initial values
    spacecraft_data = SpacecraftClass()
    spacecraft_data.hub.mass = sc_object.hub.mHub
    spacecraft_data.hub.inertia = sc_object.hub.IHubPntBc_B[2][2]
    # Define variables for panel1
    spacecraft_data.panel1.mass = unit_test_sim.panel1.mass
    spacecraft_data.panel1.inertia = unit_test_sim.panel1.IPntS_S[1][1]
    spacecraft_data.panel1.r_hinge = numpy.linalg.norm(numpy.asarray(unit_test_sim.panel1.r_HB_B))
    spacecraft_data.panel1.beta = numpy.arctan2(unit_test_sim.panel1.r_HB_B[1][0],unit_test_sim.panel1.r_HB_B[0][0])
    spacecraft_data.panel1.theta_h = 0.0
    spacecraft_data.panel1.d = unit_test_sim.panel1.d
    spacecraft_data.panel1.k = unit_test_sim.panel1.k
    spacecraft_data.panel1.c = unit_test_sim.panel1.c
    # Define variables for panel2
    spacecraft_data.panel2.mass = unit_test_sim.panel2.mass
    spacecraft_data.panel2.inertia = unit_test_sim.panel2.IPntS_S[1][1]
    spacecraft_data.panel2.r_hinge = numpy.linalg.norm(numpy.asarray(unit_test_sim.panel2.r_HB_B))
    spacecraft_data.panel2.beta = numpy.arctan2(unit_test_sim.panel2.r_HB_B[1][0],unit_test_sim.panel2.r_HB_B[0][0])
    spacecraft_data.panel2.theta_h = numpy.pi
    spacecraft_data.panel2.d = unit_test_sim.panel2.d
    spacecraft_data.panel2.k = unit_test_sim.panel2.k
    spacecraft_data.panel2.c = unit_test_sim.panel2.c
    # Define body force and torque

    # Define initial conditions of the sim
    check = 0
    time = numpy.arange(0.0,stop_time + step_size,step_size).flatten()
    x0 = numpy.zeros(10)
    x0[3] = unit_test_sim.panel1.thetaInit
    x0[4] = -unit_test_sim.panel2.thetaInit

    X = numpy.zeros((len(x0),len(time)))
    X[:,0] = x0
    for j in range (1,(len(time))):
        if time[j-1] < stop_time/2:
            spacecraft_data.x_thrust_B = 0.0
            spacecraft_data.y_thrust_B = force
            spacecraft_data.torque = 0.0
        else:
            spacecraft_data.x_thrust_B = 0.0
            spacecraft_data.y_thrust_B = 0.0
            spacecraft_data.torque = 0.0
        X[:, j] = rk4(planar_flex_function, X[:, j-1], step_size, time[j-1], spacecraft_data)
        if check == 0 and X[3,j] < X[3,j-1]:
            check = 1
        if check ==1 and X[3,j] > X[3,j-1]:
            check = 2
        if check == 2 and X[3,j] < X[3,j-1]:
            check = 3
            index_first_peak = j-1
        if check == 3 and X[3,j] > X[3,j-1]:
            check = 4
        if check==4 and X[3,j] < X[3,j-1]:
            check = 5
            index_second_peak = j-1
        if check == 5 and X[3,j] > X[3,j-1]:
            check = 6
        if check==6 and X[3,j] < X[3,j-1]:
            check = 7
            index_third_peak = j-1

    # Find the period
    T1 = time[index_second_peak] - time[index_first_peak]
    T2 = time[index_third_peak] - time[index_second_peak]
    freq_hz = 1/((T1 + T2)/2)
    matrix_m = numpy.zeros([6,6])
    matrix_m[0,0] = 1.0
    matrix_m[1,1] = spacecraft_data.hub.mass + spacecraft_data.panel1.mass + spacecraft_data.panel2.mass
    matrix_m[2,2] = 1.0
    matrix_m[3,3] = spacecraft_data.panel1.inertia + spacecraft_data.panel1.mass*spacecraft_data.panel1.d**2
    matrix_m[4,4] = 1.0
    matrix_m[5,5] = spacecraft_data.panel2.inertia + spacecraft_data.panel2.mass*spacecraft_data.panel2.d**2
    # Define off diagonal terms
    matrix_m[1,3] = spacecraft_data.panel1.mass*spacecraft_data.panel1.d
    matrix_m[1,5] = spacecraft_data.panel2.mass*spacecraft_data.panel2.d
    matrix_m[3,1] = spacecraft_data.panel1.mass*spacecraft_data.panel1.d
    matrix_m[5,1] = spacecraft_data.panel2.mass*spacecraft_data.panel2.d
    # Define A matrix
    matrix_a = numpy.zeros([6,6])
    matrix_a[0,1] = 1.0
    matrix_a[2,3] = 1.0
    matrix_a[4,5] = 1.0
    matrix_a[3,2] = -spacecraft_data.panel1.k
    matrix_a[3,3] = -spacecraft_data.panel1.c
    matrix_a[5,4] = -spacecraft_data.panel2.k
    matrix_a[5,5] = -spacecraft_data.panel2.c
    # Define Atilde
    matrix_a_tilde = numpy.dot(numpy.linalg.inv(matrix_m),matrix_a)
    eigen_values = numpy.linalg.eigvals(matrix_a_tilde)
    omega_analytical = numpy.imag(eigen_values[2])
    omega_analytical_hz = omega_analytical/(2*numpy.pi)
    diff_freq = (freq_hz-omega_analytical_hz)/omega_analytical_hz

    # Find theta_max - the max deflection while the force is being applied
    variables_in = BoxAndWingParameters()
    variables_in.k = spacecraft_data.panel1.k
    variables_in.d = spacecraft_data.panel1.d
    variables_in.F = force
    variables_in.m_sc = spacecraft_data.hub.mass + spacecraft_data.panel1.mass + spacecraft_data.panel2.mass
    variables_in.m_sp = spacecraft_data.panel1.mass
    theta_ss_guess = -0.01
    tolerance = 1e-14
    theta_ss = newton_rapshon(box_and_wings_f_and_f_prime,theta_ss_guess,tolerance,variables_in)
    theta_max = 2*theta_ss
    # Pull theta_max from the sim
    theta_max_sim = min(X[3,:])
    diff_theta_max = abs((theta_max-theta_max_sim)/theta_max)

    # Find energy to find theta_max2 - the max deflection while the force is not being applied
    mass_total = spacecraft_data.hub.mass + 2.0*spacecraft_data.panel1.mass

    y_hub_dot_off = X[6, int(stop_time/2/step_size)]
    theta1_off = X[3, int(stop_time/2/step_size)]
    theta1_off_dot = X[8, int(stop_time/2/step_size)]
    r_sp1_dot_off = numpy.array([-spacecraft_data.panel1.d*theta1_off_dot*numpy.sin(theta1_off), y_hub_dot_off + spacecraft_data.panel1.d*theta1_off_dot*numpy.cos(theta1_off)])
    v_ycm_off =  1.0/mass_total*(spacecraft_data.hub.mass*y_hub_dot_off + 2*spacecraft_data.panel1.mass*r_sp1_dot_off[1])
    energy_off = 0.5*spacecraft_data.hub.mass*y_hub_dot_off**2 + 2*(0.5*spacecraft_data.panel1.mass*numpy.dot(r_sp1_dot_off,r_sp1_dot_off) + 0.5*spacecraft_data.panel1.inertia*theta1_off_dot**2 + 0.5*spacecraft_data.panel1.k*theta1_off**2)
    energy_final_without_spring = 0.5*mass_total*v_ycm_off**2
    energy_in_spring_final = energy_off - energy_final_without_spring
    theta_max2 = numpy.sqrt(energy_in_spring_final/spacecraft_data.panel1.k)
    # Pull theta_max2 from the sim
    theta_max2_sim = max(X[3,:])
    diff_theta2_max = abs((theta_max2-theta_max2_sim)/theta_max2)

    plt.figure()
    plt.clf()
    plt.plot(time, X[3,:],'-b',label = "Lagrangian")
    plt.plot(theta1_out[:,0]*1e-9, theta1_out[:,1],'-r',label = "Xmera")
    plt.plot([theta1_out[0,0]*1e-9, theta1_out[-1,0]*1e-9], [2*theta_ss, 2*theta_ss],'-g',label = "Theta Max")
    plt.plot([theta1_out[0,0]*1e-9, theta1_out[-1,0]*1e-9], [theta_max2, theta_max2],'-k',label = "Theta Max 2")
    plt.xlabel('time (s)')
    plt.ylabel('theta (rad)')
    plt.legend(loc ='upper left',numpoints = 1)
    plot_name = "MaxThetaWhileForcing"
    plot_title = "Max Theta While Forcing"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    # Write Results to tex snippet
    snippet_name = "FrequencyResults"
    tex_snippet =  str(omega_analytical_hz) + " & " + str(freq_hz) +  " & " + str(diff_freq)
    unitTestSupport.writeTeXSnippet(snippet_name, tex_snippet, path)
    snippet_name = "Theta1Results"
    tex_snippet =  str(theta_max) + " & " + str(theta_max_sim) +  " & " + str(diff_theta_max)
    unitTestSupport.writeTeXSnippet(snippet_name, tex_snippet, path)
    snippet_name = "Theta2Results"
    tex_snippet =  str(theta_max2) + " & " + str(theta_max2_sim) +  " & " + str(diff_theta2_max)
    unitTestSupport.writeTeXSnippet(snippet_name, tex_snippet, path)

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 5e-3
    assert abs((freq_hz - omega_analytical_hz)/omega_analytical_hz) <= accuracy, ("Hinged Rigid Body integrated theta max "
                                                                             "test failed frequency comparison ")

    assert abs((theta_max - theta_max_sim)/theta_max) <= accuracy, ("Hinged Rigid Body integrated theta max test failed "
                                                                "max comparison ")

    assert abs((theta_max2 - theta_max2_sim)/theta_max2) <= accuracy, ("Hinged Rigid Body integrated theta max test failed "
                                                                   "max 2 comparison ")


def test_hinged_rigid_body_motor_torque(show_plots):
    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"
    sc_object.spacecraftName = sc_object.modelTag

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.01)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    unit_test_sim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unit_test_sim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unit_test_sim.panel1.mass = 100.0
    unit_test_sim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel1.d = 1.5
    unit_test_sim.panel1.k = 0.0
    unit_test_sim.panel1.c = 0.0
    unit_test_sim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unit_test_sim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unit_test_sim.panel1.thetaInit = 0 * numpy.pi / 180.0
    unit_test_sim.panel1.thetaDotInit = 0.0
    unit_test_sim.panel1.modelTag = "panel1"

    # set a fixed motor torque message
    motor_msg_data = messaging.ArrayMotorTorqueMsgPayload()
    motor_msg_data.motorTorque = [2.0]
    motor_msg = messaging.ArrayMotorTorqueMsg().write(motor_msg_data)
    unit_test_sim.panel1.motorTorqueInMsg.subscribeTo(motor_msg)

    # Define Variables for panel 2
    unit_test_sim.panel2.mass = 100.0
    unit_test_sim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel2.d = 1.5
    unit_test_sim.panel2.k = 0.0
    unit_test_sim.panel2.c = 0.0
    unit_test_sim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unit_test_sim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unit_test_sim.panel2.thetaInit = 0.0 * macros.D2R
    unit_test_sim.panel2.thetaDotInit = 0.0
    unit_test_sim.panel2.modelTag = "panel2"

    # Add panels to spaceCraft
    sc_object_primary = sc_object

    sc_object_primary.addStateEffector(unit_test_sim.panel1)
    sc_object_primary.addStateEffector(unit_test_sim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    sc_object_primary.hub.mHub = 750.0
    sc_object_primary.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    sc_object_primary.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    sc_object_primary.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object_primary.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object_primary.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object_primary.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel1)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel2)

    sc_state_msg = sc_object.scStateOutMsg
    data_log = sc_state_msg.recorder()
    data_panel1 = unit_test_sim.panel1.hingedRigidBodyOutMsg.recorder()
    data_panel2 = unit_test_sim.panel2.hingedRigidBodyOutMsg.recorder()
    data_panel1_log = unit_test_sim.panel1.hingedRigidBodyConfigLogOutMsg.recorder()
    data_panel2_log = unit_test_sim.panel2.hingedRigidBodyConfigLogOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)
    unit_test_sim.AddModelToTask(unit_task_name, data_panel1)
    unit_test_sim.AddModelToTask(unit_task_name, data_panel2)
    unit_test_sim.AddModelToTask(unit_task_name, data_panel1_log)
    unit_test_sim.AddModelToTask(unit_task_name, data_panel2_log)

    sc_log = sc_object.logger("totRotAngMomPntC_N")
    unit_test_sim.AddModelToTask(unit_task_name, sc_log)

    unit_test_sim.InitializeSimulation()

    stop_time = 10.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    r_out_CN_N = data_log.r_CN_N
    v_out_CN_N = data_log.v_CN_N
    sigma_BN = data_log.sigma_BN
    theta1 = data_panel1.theta
    theta2 = data_panel2.theta

    r_B1N = data_panel1_log.r_BN_N[0]
    v_B1N = data_panel1_log.v_BN_N[0]
    s_B1N = data_panel1_log.sigma_BN[0]
    o_B1N = data_panel1_log.omega_BN_B[0]
    r_B2N = data_panel2_log.r_BN_N[0]
    v_B2N = data_panel2_log.v_BN_N[0]
    s_B2N = data_panel2_log.sigma_BN[0]
    o_B2N = data_panel2_log.omega_BN_B[0]

    rot_ang_mom_N = unitTestSupport.addTimeColumn(sc_log.times(), sc_log.totRotAngMomPntC_N)

    # Get the last sigma and position
    data_pos = [r_out_CN_N[-1]]

    true_pos = [[0., 0., 0.]]

    initial_rot_ang_mom_N = [[rot_ang_mom_N[0, 1], rot_ang_mom_N[0, 2], rot_ang_mom_N[0, 3]]]

    final_rot_ang_mom = [rot_ang_mom_N[-1]]

    plt.close("all")

    plt.figure()
    plt.clf()
    plt.plot(rot_ang_mom_N[:, 0] * 1e-9, (rot_ang_mom_N[:, 1] - rot_ang_mom_N[0, 1]) ,
             rot_ang_mom_N[:, 0] * 1e-9, (rot_ang_mom_N[:, 2] - rot_ang_mom_N[0, 2]) ,
             rot_ang_mom_N[:, 0] * 1e-9, (rot_ang_mom_N[:, 3] - rot_ang_mom_N[0, 3]) )
    plt.xlabel('time (s)')
    plt.ylabel('Ang. Momentum Difference')

    plt.figure()
    plt.clf()
    plt.plot(data_log.times() * 1e-9, v_out_CN_N[:, 0],
             data_log.times() * 1e-9, v_out_CN_N[:, 1],
             data_log.times() * 1e-9, v_out_CN_N[:, 2])
    plt.xlabel('time (s)')
    plt.ylabel('m/s')

    plt.figure()
    plt.clf()
    plt.plot(data_log.times() * macros.NANO2SEC, sigma_BN[:, 0],
             color=unitTestSupport.getLineColor(0, 3),
             label=r'$\sigma_{1}$')
    plt.plot(data_log.times() * macros.NANO2SEC, sigma_BN[:, 1],
             color=unitTestSupport.getLineColor(1, 3),
             label=r'$\sigma_{2}$')
    plt.plot(data_log.times() * macros.NANO2SEC, sigma_BN[:, 2],
             color=unitTestSupport.getLineColor(2, 3),
             label=r'$\sigma_{3}$')
    plt.legend(loc='lower right')
    plt.xlabel('time (s)')
    plt.ylabel(r'MRP $\sigma_{B/N}$')

    plt.figure()
    plt.clf()
    plt.plot(data_panel1.times() * macros.NANO2SEC, theta1*macros.R2D,
             color=unitTestSupport.getLineColor(0, 3),
             label=r'$\theta_{1}$')
    plt.plot(data_panel2.times() * macros.NANO2SEC, theta2*macros.R2D,
             color=unitTestSupport.getLineColor(1, 3),
             label=r'$\theta_{2}$')
    plt.legend(loc='lower right')
    plt.xlabel('time (s)')
    plt.ylabel('Hinge Angles [deg]')

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 1e-10
    numpy.testing.assert_allclose(numpy.array(data_pos),
                                  numpy.array(true_pos),
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed position test")

    final_rot_ang_mom = numpy.delete(final_rot_ang_mom, 0, axis=1)  # remove time column
    numpy.testing.assert_allclose(numpy.array(final_rot_ang_mom),
                                  numpy.array(initial_rot_ang_mom_N),
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed rotational angular momentum unit test")

    # check config log messages
    numpy.testing.assert_allclose(r_B1N,
                                  [2.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed panel 1 r_BN_N config log test")

    numpy.testing.assert_allclose(v_B1N,
                                  [0.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed panel 1 v_BN_N config log test")

    numpy.testing.assert_allclose(s_B1N,
                                  [0.0, 0, 1.0],
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed panel 1 sigma_BN config log test")

    numpy.testing.assert_allclose(o_B1N,
                                  [0.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed panel 1 omega_BN_B config log test")

    numpy.testing.assert_allclose(r_B2N,
                                  [-2.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed panel 2 r_BN_N config log test")

    numpy.testing.assert_allclose(v_B2N,
                                  [0.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed panel 2 v_BN_N config log test")

    numpy.testing.assert_allclose(s_B2N,
                                  [0.0, 0, 0.0],
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed panel 2 sigma_BN config log test")

    numpy.testing.assert_allclose(o_B2N,
                                  [0.0, 0, 0],
                                  atol=accuracy,
                                  err_msg="Hinged Rigid Body integrated test failed panel 2 omega_BN_B config log test")


def test_hinged_rigid_body_lagrang_vs_xmera(show_plots):
    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    step_size = 0.1
    test_process_rate = macros.sec2nano(step_size)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    unit_test_sim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unit_test_sim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unit_test_sim.panel1.mass = 100.0
    unit_test_sim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel1.d = 1.5
    unit_test_sim.panel1.k = 5000.0
    unit_test_sim.panel1.c = 75
    unit_test_sim.panel1.r_HB_B = [[0.5], [1.0], [0.0]]
    unit_test_sim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]
    unit_test_sim.panel1.thetaInit = 0.0
    unit_test_sim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unit_test_sim.panel2.mass = 100.0
    unit_test_sim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unit_test_sim.panel2.d = 1.5
    unit_test_sim.panel2.k = 5000.0
    unit_test_sim.panel2.c = 75
    unit_test_sim.panel2.r_HB_B = [[-0.5], [1.0], [0.0]]
    unit_test_sim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    unit_test_sim.panel2.thetaInit = 0.0
    unit_test_sim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    sc_object.addStateEffector(unit_test_sim.panel1)
    sc_object.addStateEffector(unit_test_sim.panel2)

    # Define force and torque
    moment_arm1_B = numpy.array([0.05, 0.0, 0.0])
    force1_B = numpy.array([0.2, 0.7, 0.0])
    torque1_B = numpy.cross(moment_arm1_B,force1_B)
    moment_arm2_B = numpy.array([-0.03, 0.0, 0.0])
    force2_B = numpy.array([0.0, 1.0, 0.0])
    torque2_B = numpy.cross(moment_arm2_B,force2_B)

    # Add external force and torque
    ext_ft_object = extForceTorque.ExtForceTorque()
    ext_ft_object.modelTag = "externalDisturbance"
    ext_ft_object.extForce_B = [[force1_B[0]], [force1_B[1]], [force1_B[2]]]
    ext_ft_object.extTorquePntB_B = [[torque1_B[0]], [torque1_B[1]], [torque1_B[2]]]
    sc_object.addDynamicEffector(ext_ft_object)
    unit_test_sim.AddModelToTask(unit_task_name, ext_ft_object)

    # Define mass properties of the rigid part of the spacecraft
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    sc_object.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel1)
    unit_test_sim.AddModelToTask(unit_task_name, unit_test_sim.panel2)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    state_log = pythonVariableLogger.PythonVariableLogger({
        "theta1": lambda _: sc_object.dynManager.getStateObject(unit_test_sim.panel1.nameOfThetaState).getState(),
        "theta2": lambda _: sc_object.dynManager.getStateObject(unit_test_sim.panel2.nameOfThetaState).getState(),
    })
    unit_test_sim.AddModelToTask(unit_task_name, state_log)

    unit_test_sim.InitializeSimulation()

    # Define times that the new forces will be applies
    force1_off_time = 5.0
    force2_on_time = 11.0
    force2_off_time = 18.0
    stop_time = 20.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(force1_off_time))
    unit_test_sim.ExecuteSimulation()

    # Turn force1 off
    ext_ft_object.extForce_B = [[0.0], [0.0], [0.0]]
    ext_ft_object.extTorquePntB_B = [[0.0], [0.0], [0.0]]

    unit_test_sim.ConfigureStopTime(macros.sec2nano(force2_on_time))
    unit_test_sim.ExecuteSimulation()

    # Turn force2 on
    ext_ft_object.extForce_B = [[force2_B[0]], [force2_B[1]], [force2_B[2]]]
    ext_ft_object.extTorquePntB_B = [[torque2_B[0]], [torque2_B[1]], [torque2_B[2]]]

    unit_test_sim.ConfigureStopTime(macros.sec2nano(force2_off_time))
    unit_test_sim.ExecuteSimulation()

    # Turn force2 off and finish sim
    ext_ft_object.extForce_B = [[0.0], [0.0], [0.0]]
    ext_ft_object.extTorquePntB_B = [[0.0], [0.0], [0.0]]

    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    theta1_out = unitTestSupport.addTimeColumn(state_log.times(), state_log.theta1)
    theta2_out = unitTestSupport.addTimeColumn(state_log.times(), state_log.theta2)

    r_out_BN_N = data_log.r_BN_N
    sigma_out_BN = data_log.sigma_BN
    theta_out = 4.0*numpy.arctan(sigma_out_BN[:,2])

    # Developing the lagrangian result
    # Define initial values
    spacecraft_data = SpacecraftClass()
    spacecraft_data.hub.mass = sc_object.hub.mHub
    spacecraft_data.hub.inertia = sc_object.hub.IHubPntBc_B[2][2]
    # Define variables for panel1
    spacecraft_data.panel1.mass = unit_test_sim.panel1.mass
    spacecraft_data.panel1.inertia = unit_test_sim.panel1.IPntS_S[1][1]
    spacecraft_data.panel1.r_hinge = numpy.linalg.norm(numpy.asarray(unit_test_sim.panel1.r_HB_B))
    spacecraft_data.panel1.beta = numpy.arctan2(unit_test_sim.panel1.r_HB_B[1][0],unit_test_sim.panel1.r_HB_B[0][0])
    spacecraft_data.panel1.theta_h = 0.0
    spacecraft_data.panel1.d = unit_test_sim.panel1.d
    spacecraft_data.panel1.k = unit_test_sim.panel1.k
    spacecraft_data.panel1.c = unit_test_sim.panel1.c
    # Define variables for panel2
    spacecraft_data.panel2.mass = unit_test_sim.panel2.mass
    spacecraft_data.panel2.inertia = unit_test_sim.panel2.IPntS_S[1][1]
    spacecraft_data.panel2.r_hinge = numpy.linalg.norm(numpy.asarray(unit_test_sim.panel2.r_HB_B))
    spacecraft_data.panel2.beta = numpy.arctan2(unit_test_sim.panel2.r_HB_B[1][0],unit_test_sim.panel2.r_HB_B[0][0])
    spacecraft_data.panel2.theta_h = numpy.pi
    spacecraft_data.panel2.d = unit_test_sim.panel2.d
    spacecraft_data.panel2.k = unit_test_sim.panel2.k
    spacecraft_data.panel2.c = unit_test_sim.panel2.c

    # Define initial conditions of the sim
    time = numpy.arange(0.0,stop_time + step_size,step_size).flatten()
    x0 = numpy.zeros(10)
    x0[3] = unit_test_sim.panel1.thetaInit
    x0[4] = -unit_test_sim.panel2.thetaInit

    X = numpy.zeros((len(x0),len(time)))
    X[:,0] = x0
    for j in range (1,(len(time))):
        if time[j-1] < force1_off_time:
            spacecraft_data.x_thrust_B = force1_B[0]
            spacecraft_data.y_thrust_B = force1_B[1]
            spacecraft_data.torque = torque1_B[2]
        elif time[j-1] >= force2_on_time and time[j-1] < force2_off_time:
            spacecraft_data.x_thrust_B = force2_B[0]
            spacecraft_data.y_thrust_B = force2_B[1]
            spacecraft_data.torque = torque2_B[2]
        else:
            spacecraft_data.x_thrust_B = 0.0
            spacecraft_data.y_thrust_B = 0.0
            spacecraft_data.torque = 0.0
        X[:, j] = rk4(planar_flex_function, X[:, j-1], step_size, time[j-1], spacecraft_data)

    plt.figure()
    plt.clf()
    plt.plot(time, X[0,:],'-b',label = "Lagrangian")
    plt.plot(data_log.times()*1e-9, (r_out_BN_N[:,0]-r_out_BN_N[:,0]),'-r',label = "Xmera")
    plt.plot([time[25], time[75], time[125], time[175]], [X[0,25], X[0,75], X[0,125], X[0,175],],'ok',label = "Test Points")
    plt.xlabel('time (s)')
    plt.ylabel('x position (m)')
    plt.legend(loc ='upper left',numpoints = 1)
    plot_name = "XPositionLagrangianVsXmera"
    plot_title = "X Position Lagrangian Vs Xmera"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, X[1,:],'-b',label = "Lagrangian")
    plt.plot(data_log.times()*1e-9, (r_out_BN_N[:,1]-r_out_BN_N[:,1]),'r',label = "Xmera")
    plt.plot([time[25], time[75], time[125], time[175]], [X[1,25], X[1,75], X[1,125], X[1,175],],'ok',label = "Test Points")
    plt.xlabel('time (s)')
    plt.ylabel('y position (m)')
    plt.legend(loc ='upper left',numpoints = 1)
    plot_name = "YPositionLagrangianVsXmera"
    plot_title = "Y Position Lagrangian Vs Xmera"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, X[2,:],'-b',label = "Lagrangian")
    plt.plot(data_log.times()*1e-9, theta_out,'-r',label = "Xmera")
    plt.plot([time[25], time[75], time[125], time[175]], [X[2,25], X[2,75], X[2,125], X[2,175],],'ok',label = "Test Points")
    plt.xlabel('time (s)')
    plt.ylabel('theta (rad)')
    plt.legend(loc ='upper left',numpoints = 1)
    plot_name = "ThetaLagrangianVsXmera"
    plot_title = "Theta Lagrangian Vs Xmera"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, X[3,:],'-b',label = "Lagrangian")
    plt.plot(theta1_out[:,0]*1e-9, theta1_out[:,1],'-r',label = "Xmera")
    plt.plot([time[25], time[75], time[125], time[175]], [X[3,25], X[3,75], X[3,125], X[3,175],],'ok',label = "Test Points")
    plt.xlabel('time (s)')
    plt.ylabel('theta 1 (rad)')
    plt.legend(loc ='upper left',numpoints = 1)
    plot_name = "Theta1LagrangianVsXmera"
    plot_title = "Theta 1 Position Lagrangian Vs Xmera"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(time, -X[4,:],'-b',label = "Lagrangian")
    plt.plot(theta2_out[:,0]*1e-9, theta2_out[:,1],'-r',label = "Xmera")
    plt.plot([time[25], time[75], time[125], time[175]], [-X[4,25], -X[4,75], -X[4,125], -X[4,175],],'ok',label = "Test Points")
    plt.xlabel('time (s)')
    plt.ylabel('theta 2 (rad)')
    plt.legend(loc ='lower left',numpoints = 1)
    plot_name = "Theta2LagrangianVsXmera"
    plot_title = "Theta 2 Lagrangian Vs Xmera"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    if show_plots:
        plt.show()
    plt.close("all")

    accuracy = 1e-10
    time_list = [25, 75, 125, 175]

    for i in time_list:
        assert abs(X[0,i] - (r_out_BN_N[i,0]-r_out_BN_N[0,0])) <= accuracy, ("Hinged Rigid Body integrated test "
                                                                          "Lagrangian vs. Xmera failed x position "
                                                                          "comparison ")
        assert abs(X[1,i] - (r_out_BN_N[i,1]-r_out_BN_N[0,1])) <= accuracy, ("Hinged Rigid Body integrated test "
                                                                          "Lagrangian vs. Xmera failed y position "
                                                                          "comparison ")
        assert abs(X[2,i] - theta_out[i]) <= accuracy, ("Hinged Rigid Body integrated test Lagrangian vs. Xmera failed "
                                                      "theta comparison ")
        assert abs(X[3,i] - theta1_out[i,1]) <= accuracy, ("Hinged Rigid Body integrated test Lagrangian vs. Xmera "
                                                         "failed theta 1 comparison ")
        assert abs(-X[4,i] - theta2_out[i,1]) <= accuracy, ("Hinged Rigid Body integrated test Lagrangian vs. Xmera "
                                                          "failed theta 2 comparison ")


def planar_flex_function(x, t, variables):
    theta = x[2]
    theta1 = x[3]
    theta2 = x[4]
    x_hub_dot = x[5]
    y_hub_dot = x[6]
    theta_dot = x[7]
    theta1_dot = x[8]
    theta2_dot = x[9]
    # Define variables for hub
    m_hub = variables.hub.mass
    IHub = variables.hub.inertia
    # Define variables for panel1
    m_sp1 = variables.panel1.mass
    ISP1 = variables.panel1.inertia
    r_hinge1 = variables.panel1.r_hinge
    beta1 = variables.panel1.beta
    theta_h1 = variables.panel1.theta_h
    d1 = variables.panel1.d
    k1 = variables.panel1.k
    c1 = variables.panel1.c
    # Define variables for panel2
    m_sp2 = variables.panel2.mass
    ISP2 = variables.panel2.inertia
    r_hinge2 = variables.panel2.r_hinge
    beta2 = variables.panel2.beta
    theta_h2 = variables.panel2.theta_h
    d2 = variables.panel2.d
    k2 = variables.panel2.k
    c2 = variables.panel2.c
    Tx_B = variables.x_thrust_B
    Ty_B = variables.y_thrust_B
    torque = variables.torque

    # Convert Tx_B and Ty_B to the inertial frame
    dcm_BN = numpy.array([[numpy.cos(theta), numpy.sin(theta)],
                          [-numpy.sin(theta), numpy.cos(theta)]])
    Thrust_N = numpy.dot(dcm_BN.transpose(),numpy.array([[Tx_B],[Ty_B]]))
    Tx = Thrust_N[0,0]
    Ty = Thrust_N[1,0]

    matrix_a = numpy.zeros((5,5))
    vector_b = numpy.zeros(5)
    # Populate X Translation Equation
    matrix_a[0,0] = 1.0
    matrix_a[0,1] = 0.0
    matrix_a[0,2] = -1/(m_hub + m_sp1 + m_sp2)*(m_sp1*r_hinge1*numpy.sin(beta1 + theta) + m_sp2*r_hinge2*numpy.sin(beta2 + theta) + d1*m_sp1*numpy.sin(theta_h1 + theta + theta1) + d2*m_sp2*numpy.sin(theta_h2 + theta + theta2))
    matrix_a[0,3] = -1/(m_hub + m_sp1 + m_sp2)*(d1*m_sp1*numpy.sin(theta_h1 + theta + theta1))
    matrix_a[0,4] = -1/(m_hub + m_sp1 + m_sp2)*(d2*m_sp2*numpy.sin(theta_h2 + theta + theta2))
    vector_b[0] = 1/(m_hub + m_sp1 + m_sp2)*(Tx + m_sp1*r_hinge1*numpy.cos(beta1 + theta)*theta_dot**2 + m_sp2*r_hinge2*numpy.cos(beta2 + theta)*theta_dot**2 + d1*m_sp1*numpy.cos(theta_h1 + theta + theta1)*theta_dot**2 + d2*m_sp2*numpy.cos(theta_h2 + theta + theta2)*theta_dot**2
               + 2*d1*m_sp1*numpy.cos(theta_h1 + theta + theta1)*theta_dot*theta1_dot + d1*m_sp1*numpy.cos(theta_h1 + theta + theta1)*theta1_dot**2 + 2*d2*m_sp2*numpy.cos(theta_h2 + theta + theta2)*theta_dot*theta2_dot
               + d2*m_sp2*numpy.cos(theta_h2 + theta + theta2)*theta2_dot**2)
    # Populate Y Translation Equation
    matrix_a[1,0] = 0.0
    matrix_a[1,1] = 1.0
    matrix_a[1,2] = 1/(m_hub + m_sp1 + m_sp2)*(m_sp1*r_hinge1*numpy.cos(beta1 + theta) + m_sp2*r_hinge2*numpy.cos(beta2 + theta) + d1*m_sp1*numpy.cos(theta_h1 + theta + theta1) + d2*m_sp2*numpy.cos(theta_h2 + theta + theta2))
    matrix_a[1,3] = 1/(m_hub + m_sp1 + m_sp2)*(d1*m_sp1*numpy.cos(theta_h1 + theta + theta1))
    matrix_a[1,4] = 1/(m_hub + m_sp1 + m_sp2)*(d2*m_sp2*numpy.cos(theta_h2 + theta + theta2))
    vector_b[1] = 1/(m_hub + m_sp1 + m_sp2)*(Ty + m_sp1*r_hinge1*numpy.sin(beta1 + theta)*theta_dot**2 + m_sp2*r_hinge2*numpy.sin(beta2 + theta)*theta_dot**2 + d1*m_sp1*numpy.sin(theta_h1 + theta + theta1)*theta_dot**2 + d2*m_sp2*numpy.sin(theta_h2 + theta + theta2)*theta_dot**2
               + 2*d1*m_sp1*numpy.sin(theta_h1 + theta + theta1)*theta_dot*theta1_dot + d1*m_sp1*numpy.sin(theta_h1 + theta + theta1)*theta1_dot**2 + 2*d2*m_sp2*numpy.sin(theta_h2 + theta + theta2)*theta_dot*theta2_dot
               + d2*m_sp2*numpy.sin(theta_h2 + theta + theta2)*theta2_dot**2)
    # Populate theta Equation
    matrix_a[2,0] = -1/(IHub + ISP1 + ISP2 + d1**2*m_sp1 + d2**2*m_sp2 + m_sp1*r_hinge1**2 + m_sp2*r_hinge2**2 + 2*d1*m_sp1*r_hinge1*numpy.cos(beta1 - theta_h1 - theta1) + 2*d2*m_sp2*r_hinge2*numpy.cos(beta2 - theta_h2 - theta2))*(m_sp1*r_hinge1*numpy.sin(beta1 + theta)
                 + m_sp2*r_hinge2*numpy.sin(beta2 + theta) + d1*m_sp1*numpy.sin(theta_h1 + theta + theta1) + d2*m_sp2*numpy.sin(theta_h2 + theta + theta2))
    matrix_a[2,1] = 1/(IHub + ISP1 + ISP2 + d1**2*m_sp1 + d2**2*m_sp2 + m_sp1*r_hinge1**2 + m_sp2*r_hinge2**2 + 2*d1*m_sp1*r_hinge1*numpy.cos(beta1 - theta_h1 - theta1) + 2*d2*m_sp2*r_hinge2*numpy.cos(beta2 - theta_h2 - theta2))*(m_sp1*r_hinge1*numpy.cos(beta1 + theta)
                 + m_sp2*r_hinge2*numpy.cos(beta2 + theta) + d1*m_sp1*numpy.cos(theta_h1 + theta + theta1) + d2*m_sp2*numpy.cos(theta_h2 + theta + theta2))
    matrix_a[2,2] = 1.0
    matrix_a[2,3] = 1/(IHub + ISP1 + ISP2 + d1**2*m_sp1 + d2**2*m_sp2 + m_sp1*r_hinge1**2 + m_sp2*r_hinge2**2 + 2*d1*m_sp1*r_hinge1*numpy.cos(beta1 - theta_h1 - theta1) + 2*d2*m_sp2*r_hinge2*numpy.cos(beta2 - theta_h2 - theta2))*(ISP1
                 + d1**2*m_sp1 + d1*m_sp1*r_hinge1*numpy.cos(beta1 - theta_h1 - theta1))
    matrix_a[2,4] = 1/(IHub + ISP1 + ISP2 + d1**2*m_sp1 + d2**2*m_sp2 + m_sp1*r_hinge1**2 + m_sp2*r_hinge2**2 + 2*d1*m_sp1*r_hinge1*numpy.cos(beta1 - theta_h1 - theta1) + 2*d2*m_sp2*r_hinge2*numpy.cos(beta2 - theta_h2 - theta2))*(ISP2
                 + d2**2*m_sp2 + d2*m_sp2*r_hinge2*numpy.cos(beta2 - theta_h2 - theta2))
    vector_b[2] = 1/(IHub + ISP1 + ISP2 + d1**2*m_sp1 + d2**2*m_sp2 + m_sp1*r_hinge1**2 + m_sp2*r_hinge2**2 + 2*d1*m_sp1*r_hinge1*numpy.cos(beta1 - theta_h1 - theta1) + 2*d2*m_sp2*r_hinge2*numpy.cos(beta2 - theta_h2 - theta2))*(torque
              - 2*d1*m_sp1*r_hinge1*numpy.sin(beta1 - theta_h1 - theta1)*theta_dot*theta1_dot - d1*m_sp1*r_hinge1*numpy.sin(beta1 - theta_h1 - theta1)*theta1_dot**2
              - 2*d2*m_sp2*r_hinge2*numpy.sin(beta2 - theta_h2 - theta2)*theta_dot*theta2_dot - d2*m_sp2*r_hinge2*numpy.sin(beta2 - theta_h2 - theta2)*theta2_dot**2)
    # Populate theta1 Equation
    matrix_a[3,0] = -1/(ISP1 + d1**2*m_sp1)*(d1*m_sp1*numpy.sin(theta_h1 + theta + theta1))
    matrix_a[3,1] = 1/(ISP1 + d1**2*m_sp1)*(d1*m_sp1*numpy.cos(theta_h1 + theta + theta1))
    matrix_a[3,2] = 1/(ISP1 + d1**2*m_sp1)*(ISP1 + d1**2*m_sp1 + d1*m_sp1*r_hinge1*numpy.cos(beta1 - theta_h1 - theta1))
    matrix_a[3,3] = 1.0
    matrix_a[3,4] = 0.0
    vector_b[3] = 1/(ISP1 + d1**2*m_sp1)*(-k1*theta1 + d1*m_sp1*r_hinge1*numpy.sin(beta1 - theta_h1 - theta1)*theta_dot**2 - c1*theta1_dot)
    # Populate theta2 Equation
    matrix_a[4,0] = -1/(ISP2 + d2**2*m_sp2)*(d2*m_sp2*numpy.sin(theta_h2 + theta + theta2))
    matrix_a[4,1] = 1/(ISP2 + d2**2*m_sp2)*(d2*m_sp2*numpy.cos(theta_h2 + theta + theta2))
    matrix_a[4,2] = 1/(ISP2 + d2**2*m_sp2)*(ISP2 + d2**2*m_sp2 + d2*m_sp2*r_hinge2*numpy.cos(beta2 - theta_h2 - theta2))
    matrix_a[4,3] = 0.0
    matrix_a[4,4] = 1.0
    vector_b[4] = 1/(ISP2 + d2**2*m_sp2)*(-k2*theta2 + d2*m_sp2*r_hinge2*numpy.sin(beta2 - theta_h2 - theta2)*theta_dot**2 - c2*theta2_dot)

    Xdot = numpy.zeros(len(x))
    # Populate Trivial derivatives
    Xdot[0] = x_hub_dot
    Xdot[1] = y_hub_dot
    Xdot[2] = theta_dot
    Xdot[3] = theta1_dot
    Xdot[4] = theta2_dot
    # Calculate nontrivial derivatives
    result = numpy.dot(numpy.linalg.inv(matrix_a),vector_b)
    Xdot[5:10] = result

    return Xdot


def rk4(Fn, X, h, t, varargin):
    k1 = h*Fn(X, t, varargin)
    k2 = h*Fn(X+k1/2, t+h/2, varargin)
    k3 = h*Fn(X+k2/2, t+h/2, varargin)
    k4 = h*Fn(X+k3, t+h, varargin)
    Z = X + (k1 + 2*k2 + 2*k3 + k4)/6.0
    return Z


class SolarPanel:
    mass = 0.0
    inertia = 0.0
    r_hinge = 0.0
    beta = 0.0
    theta_h = 0.0
    d = 0.0
    k = 0.0
    c = 0.0


class HubClass:
    mass = 0.0
    inertia = 0.0


class SpacecraftClass:
    panel1 = SolarPanel()
    panel2 = SolarPanel()
    hub = HubClass()
    x_thrust_B = 0.0
    y_thrust_B = 0.0
    torque = 0.0


def newton_rapshon(func_and_dervi,guess,tolerance,variables):
    x_old = guess
    for i in range(1,101):
        fx, f_prime_x = func_and_dervi(x_old, variables)
        x_new = x_old - fx/f_prime_x
        if abs(x_new - x_old) < tolerance:
            break
        x_old = x_new
    return x_new


def box_and_wings_f_and_f_prime(theta,variables):
    # Define variables
    F = variables.F
    m_sc = variables.m_sc
    k = variables.k
    m_sp = variables.m_sp
    d = variables.d
    a_sp = F/m_sc
    f_x = k*theta + m_sp*a_sp*d*numpy.cos(theta)
    f_prime_x = k - m_sp*a_sp*d*numpy.sin(theta)
    return f_x, f_prime_x


class BoxAndWingParameters:
    F = 0
    m_sc = 0
    k = 0
    m_sp = 0
    d = 0

if __name__ == "__main__":
    # test_hinged_rigid_body_gravity(True)
    # test_hinged_rigid_body_gravity_invariance(True)
    # test_hinged_rigid_body_no_gravity(True)
    # test_hinged_rigid_body_no_gravity_damping(True)
    # test_hinged_rigid_body_theta_ss(True)
    # test_hinged_rigid_body_frequency_amp(True)
    # test_hinged_rigid_body_motor_torque(True, True)
    test_hinged_rigid_body_lagrang_vs_xmera(True)
