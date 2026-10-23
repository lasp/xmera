# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os

import numpy
import pytest
from random import random

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from xmera.simulation import spacecraft
from xmera.utilities import macros
from xmera.simulation import gravityEffector
from xmera.simulation import extForceTorque
from xmera.utilities import RigidBodyKinematics
from xmera.utilities import simIncludeGravBody
from xmera.simulation import GravityGradientEffector
from xmera.architecture import messaging

def add_time_column(time, data):
    return numpy.transpose(numpy.vstack([[time], numpy.transpose(data)]))

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_


@pytest.mark.parametrize("function", ["sc_translation"
                                      , "sc_trans_and_rotation"
                                      , "sc_rotation"
                                      , "sc_trans_boe"
                                      , "sc_point_b_vs_point_c"
                                      , "sc_optional_ref"
                                      , "sc_accum_dv"
                                      , "sc_accum_dv_ext_force"
                                      ])
def test_spacecraft_all_test(show_plots, function):
    """Module Unit Test"""
    if function == "sc_optional_ref":
        [test_results, test_message] = eval(function + '(show_plots, 1e-3)')
    elif function == "sc_accum_dv" or function == "sc_accum_dv_ext_force":
        [test_results, test_message] = eval(function + '()')
    else:
        [test_results, test_message] = eval(function + '(show_plots)')
    assert test_results < 1, test_message


def sc_translation(show_plots):
    """Module Unit Test"""
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.01)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    unit_test_sim.earthGravBody = gravityEffector.GravBodyData()
    unit_test_sim.earthGravBody.planetName = "earth_planet_data"
    unit_test_sim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unit_test_sim.earthGravBody.isCentralBody = True

    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([unit_test_sim.earthGravBody])

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Define initial conditions of the spacecraft
    sc_object.hub.mHub = 100
    sc_object.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]

    sc_object_log = sc_object.logger(["totOrbAngMomPntN_N", "totOrbEnergy"])
    unit_test_sim.AddModelToTask(unit_task_name, sc_object_log)

    unit_test_sim.InitializeSimulation()
    accuracy = 1e-3
    if not unitTestSupport.isArrayEqual(sc_object.scStateOutMsg.read().r_BN_N,
                                        [item for sublist in sc_object.hub.r_CN_NInit for item in sublist],
                                        3, accuracy):
        test_fail_count += 1
        test_messages.append("FAILED: SCHub Translation test failed init pos msg unit test")
    if not unitTestSupport.isArrayEqual(sc_object.scStateOutMsg.read().v_BN_N,
                                        [item for sublist in sc_object.hub.v_CN_NInit for item in sublist],
                                        3, accuracy):
        test_fail_count += 1
        test_messages.append("FAILED: SCHub Translation test failed init pos msg unit test")


    stop_time = 10.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()
    orb_ang_mom_N = unitTestSupport.addTimeColumn(sc_object_log.times(), sc_object_log.totOrbAngMomPntN_N)
    orb_energy = unitTestSupport.addTimeColumn(sc_object_log.times(), sc_object_log.totOrbEnergy)

    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,1] - orb_ang_mom_N[0,1])/orb_ang_mom_N[0,1], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,2] - orb_ang_mom_N[0,2])/orb_ang_mom_N[0,2], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,3] - orb_ang_mom_N[0,3])/orb_ang_mom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInOrbitalAngularMomentumTranslationOnly", "Change in Orbital Angular Momentum Translation Only", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(orb_energy[:,0]*1e-9, (orb_energy[:,1] - orb_energy[0,1])/orb_energy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInOrbitalEnergyTranslationOnly", "Change in Orbital Energy Translation Only", plt, r"width=0.8\textwidth", path)
    if show_plots:
        plt.show()
        plt.close('all')

    module_output = data_log.r_BN_N

    true_pos = [
                [-4072255.7737936215, 7456050.4649078, 5258610.029627514]
                ]

    initial_orb_ang_mom_N = [
                [orb_ang_mom_N[0,1], orb_ang_mom_N[0,2], orb_ang_mom_N[0,3]]
                ]

    final_orb_ang_mom = [
                [orb_ang_mom_N[-1,1], orb_ang_mom_N[-1,2], orb_ang_mom_N[-1,3]]
                 ]

    initial_orb_energy = [
                [orb_energy[0,1]]
                ]

    final_orb_energy = [
                [orb_energy[-1,1]]
                 ]

    accuracy = 1e-10
    for i in range(0,len(true_pos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(module_output[-1,:],true_pos[i],3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: SCHub Translation test failed pos unit test")

    for i in range(0,len(initial_orb_ang_mom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(final_orb_ang_mom[i],initial_orb_ang_mom_N[i],3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: SCHub Translation test failed orbital angular momentum unit test")

    for i in range(0,len(initial_orb_energy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(final_orb_energy[i],initial_orb_energy[i],1,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: SCHub Translation test failed orbital energy unit test")

    if test_fail_count == 0:
        print("PASSED: " + " SCHub Translation Integrated Sim Test")

    assert test_fail_count < 1, test_messages

    # return fail count and join into a single string all messages in the list
    # test_message
    return [test_fail_count, ''.join(test_messages)]

def sc_trans_and_rotation(show_plots):
    """Module Unit Test"""
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

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    unit_test_sim.earthGravBody = gravityEffector.GravBodyData()
    unit_test_sim.earthGravBody.planetName = "earth_planet_data"
    unit_test_sim.earthGravBody.mu = 0.3986004415E+15  # meters!
    unit_test_sim.earthGravBody.isCentralBody = True

    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([unit_test_sim.earthGravBody])

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Define initial conditions of the spacecraft
    sc_object.hub.mHub = 100
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    sc_object_log = sc_object.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unit_test_sim.AddModelToTask(unit_task_name, sc_object_log)

    unit_test_sim.InitializeSimulation()

    stop_time = 10.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    orb_ang_mom_N = unitTestSupport.addTimeColumn(sc_object_log.times(), sc_object_log.totOrbAngMomPntN_N)
    rot_ang_mom_N = unitTestSupport.addTimeColumn(sc_object_log.times(), sc_object_log.totRotAngMomPntC_N)
    rot_energy = unitTestSupport.addTimeColumn(sc_object_log.times(), sc_object_log.totRotEnergy)
    orb_energy = unitTestSupport.addTimeColumn(sc_object_log.times(), sc_object_log.totOrbEnergy)

    r_BN_NOutput = data_log.r_BN_N
    sigma_BNOutput = data_log.sigma_BN

    true_pos = [
                [-4072255.7737936215, 7456050.4649078, 5258610.029627514]
                ]

    true_sigma = [
                [3.73034285e-01,  -2.39564413e-03,   2.08570797e-01]
                ]

    initial_orb_ang_mom_N = [
                [orb_ang_mom_N[0,1], orb_ang_mom_N[0,2], orb_ang_mom_N[0,3]]
                ]

    final_orb_ang_mom = [
                [orb_ang_mom_N[-1,1], orb_ang_mom_N[-1,2], orb_ang_mom_N[-1,3]]
                 ]

    initial_rot_ang_mom_N = [
                [rot_ang_mom_N[0,1], rot_ang_mom_N[0,2], rot_ang_mom_N[0,3]]
                ]

    final_rot_ang_mom = [
                [rot_ang_mom_N[-1,1], rot_ang_mom_N[-1,2], rot_ang_mom_N[-1,3]]
                 ]

    initial_orb_energy = [
                [orb_energy[0,1]]
                ]

    final_orb_energy = [
                [orb_energy[-1,1]]
                 ]

    initial_rot_energy = [
                [rot_energy[0,1]]
                ]

    final_rot_energy = [
                [rot_energy[-1,1]]
                 ]

    plt.close('all')
    plt.figure()
    plt.clf()
    plt.plot(orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,1] - orb_ang_mom_N[0,1])/orb_ang_mom_N[0,1], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,2] - orb_ang_mom_N[0,2])/orb_ang_mom_N[0,2], orb_ang_mom_N[:,0]*1e-9, (orb_ang_mom_N[:,3] - orb_ang_mom_N[0,3])/orb_ang_mom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInOrbitalAngularMomentumTranslationAndRotation", "Change in Orbital Angular Momentum Translation And Rotation", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(orb_energy[:,0]*1e-9, (orb_energy[:,1] - orb_energy[0,1])/orb_energy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInOrbitalEnergyTranslationAndRotation", "Change in Orbital Energy Translation And Rotation", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,1] - rot_ang_mom_N[0,1])/rot_ang_mom_N[0,1], rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,2] - rot_ang_mom_N[0,2])/rot_ang_mom_N[0,2], rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_N[:,3] - rot_ang_mom_N[0,3])/rot_ang_mom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInRotationalAngularMomentumTranslationAndRotation", "Change in Rotational Angular Momentum Translation And Rotation", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rot_energy[:,0]*1e-9, (rot_energy[:,1] - rot_energy[0,1])/rot_energy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInRotationalEnergyTranslationAndRotation", "Change in Rotational Energy Translation And Rotation", plt, r"width=0.8\textwidth", path)
    if show_plots:
        plt.show()
        plt.close('all')

    accuracy = 1e-8
    for i in range(0,len(true_pos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(r_BN_NOutput[-1,:],true_pos[i],3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed pos unit test")

    for i in range(0,len(true_sigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(sigma_BNOutput[-1,:],true_sigma[i],3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed attitude unit test")

    accuracy = 1e-10
    for i in range(0,len(initial_orb_ang_mom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(final_orb_ang_mom[i],initial_orb_ang_mom_N[i],3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed orbital angular momentum unit test")

    for i in range(0,len(initial_rot_ang_mom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(final_rot_ang_mom[i],initial_rot_ang_mom_N[i],3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational angular momentum unit test")

    for i in range(0,len(initial_rot_energy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(final_rot_energy[i],initial_rot_energy[i],1,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational energy unit test")

    for i in range(0,len(initial_orb_energy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(final_orb_energy[i],initial_orb_energy[i],1,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed orbital energy unit test")

    if test_fail_count == 0:
        print("PASSED: " + " Spacecraft Translation and Rotation Integrated Sim Test")

    assert test_fail_count < 1, test_messages

    # return fail count and join into a single string all messages in the list
    # test_message
    return [test_fail_count, ''.join(test_messages)]

def sc_rotation(show_plots):
    """Module Unit Test"""
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    time_step = 0.001
    test_process_rate = macros.sec2nano(time_step)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Define initial conditions of the spacecraft
    sc_object.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc_object.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    # BOE for rotational dynamics
    h = numpy.dot(numpy.asarray(sc_object.hub.IHubPntBc_B),numpy.asarray(sc_object.hub.omega_BN_BInit).flatten())
    H = numpy.linalg.norm(h)
    n3_B = -h/H

    # Find DCM
    n2_B = numpy.zeros(3)
    n2_B[1] = 0.1
    n2_B[0] = -n2_B[1]*n3_B[1]/n3_B[0]
    n2_B = n2_B/numpy.linalg.norm(n2_B)
    n1_B = numpy.cross(n2_B,n3_B)
    n1_B = n1_B/(numpy.linalg.norm(n1_B))
    dcm_BN = numpy.zeros([3,3])
    dcm_BN[:,0] = n1_B
    dcm_BN[:,1] = n2_B
    dcm_BN[:,2] = n3_B
    h3_N = numpy.array([0,0,-H])
    h3_B = numpy.dot(dcm_BN,h3_N)
    h3_Ncheck = numpy.dot(dcm_BN.transpose(),h3_B)
    sigma_calc = RigidBodyKinematics.C2MRP(dcm_BN)
    sc_object.hub.sigma_BNInit = [[sigma_calc[0]], [sigma_calc[1]], [sigma_calc[2]]]

    sc_object_log = sc_object.logger(["totRotAngMomPntC_N", "totRotEnergy"])
    unit_test_sim.AddModelToTask(unit_task_name, sc_object_log)

    unit_test_sim.InitializeSimulation()

    stop_time = 10.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    rot_ang_mom_N = unitTestSupport.addTimeColumn(sc_object_log.times(), sc_object_log.totRotAngMomPntC_N)
    rot_energy = unitTestSupport.addTimeColumn(sc_object_log.times(), sc_object_log.totRotEnergy)
    rot_ang_mom_mag = numpy.zeros(len(rot_ang_mom_N))
    for i in range(0,len(rot_ang_mom_N)):
        rot_ang_mom_mag[i] = numpy.linalg.norm(numpy.asarray(rot_ang_mom_N[i,1:4]))

    true_sigma = [
                [5.72693314e-01,   5.10734375e-01,  -3.07377611e-01]
                ]

    initial_rot_ang_mom_N = [
                [numpy.linalg.norm(numpy.asarray(rot_ang_mom_N[0,1:4]))]
                ]

    final_rot_ang_mom = [
                [numpy.linalg.norm(numpy.asarray(rot_ang_mom_N[-1,1:4]))]
                 ]

    initial_rot_energy = [
                [rot_energy[0,1]]
                ]

    final_rot_energy = [
                [rot_energy[-1,1]]
                 ]

    module_output = add_time_column(data_log.times(), data_log.sigma_BN)
    omega_BNOutput = add_time_column(data_log.times(), data_log.omega_BN_B)

    check = 0
    for i in range(0,len(module_output)):
        if check == 0 and module_output[i+1,2] < module_output[i,2]:
            check = 1
        if check == 1 and module_output[i+1,2] > module_output[i,2]:
            check = 2
            index = i+1
            break

    sigma_before_switch = module_output[index-1,1:4]
    sigma_before_before = module_output[index-2,1:4]
    sigma_after_switch = module_output[index,:]
    delta_t = (module_output[index-1,0] - module_output[index-2,0])*1e-9
    y_prime = (sigma_before_switch - sigma_before_before)/delta_t
    sigma_ghost = sigma_before_switch + y_prime*delta_t
    sigma_after_analytical = - sigma_ghost/numpy.dot(numpy.linalg.norm(numpy.asarray(sigma_ghost)),numpy.linalg.norm(numpy.asarray(sigma_ghost)))

    time_array = numpy.zeros(5)
    sigma_array = numpy.zeros([3,5])
    omega_analytical_array = numpy.zeros([3,5])
    omega_array = numpy.zeros([4,5])
    for i in range(0, 5):
        idx = int(stop_time/time_step*(i+1)/5)
        time_array[i] = module_output[idx, 0]
        sigma_array[:, i] = module_output[idx, 1:4]
        sigma = sigma_array[:, i]
        sigma_norm = numpy.linalg.norm(sigma)
        sigma1 = sigma[0]
        sigma2 = sigma[1]
        sigma3 = sigma[2]
        omega_array[:,i] = omega_BNOutput[idx, :]
        omega_analytical_array[0,i] = -H/(1 + sigma_norm**2)**2*(8*sigma1*sigma3 - 4*sigma2*(1 - sigma_norm**2))/sc_object.hub.IHubPntBc_B[0][0]
        omega_analytical_array[1,i] = -H/(1 + sigma_norm**2)**2*(8*sigma2*sigma3 + 4*sigma1*(1 - sigma_norm**2))/sc_object.hub.IHubPntBc_B[1][1]
        omega_analytical_array[2,i] = -H/(1 + sigma_norm**2)**2*(4*(-sigma1**2 - sigma2**2 + sigma3**2) + (1 - sigma_norm**2)**2)/sc_object.hub.IHubPntBc_B[2][2]

    plt.close("all")    # clear out earlier figures

    plt.figure()
    plt.clf()
    plt.plot(module_output[:,0]*1e-9, module_output[:,1], module_output[:,0]*1e-9, module_output[:,2], module_output[:,0]*1e-9, module_output[:,3])
    plt.plot(module_output[index,0]*1e-9, module_output[index,1],'bo')
    plt.plot(module_output[index,0]*1e-9, sigma_ghost[0],'yo')
    plt.plot(module_output[index-1,0]*1e-9, module_output[index-1,1],'bo')
    plt.xlabel("Time (s)")
    plt.ylabel("MRPs")
    unitTestSupport.writeFigureLaTeX("scPlusMRPs", "Attitude of Spacecraft in MRPs", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(module_output[index - 3: index + 3,0]*1e-9, module_output[index - 3: index + 3,1],"b")
    plt.plot(module_output[index-1,0]*1e-9, module_output[index-1,1],'bo', label = "Xmera " + r"$\sigma_{1,t-1}$")
    plt.plot(module_output[index,0]*1e-9, module_output[index,1],'ro', label = "Xmera " + r"$\sigma_{1,t}$")
    plt.plot(module_output[index,0]*1e-9, sigma_ghost[0],'ko', label = "Xmera " + r"$\sigma_{1,0}$")
    plt.plot([module_output[index-1,0]*1e-9, module_output[index,0]*1e-9], [module_output[index-1,1], sigma_ghost[0]],'--k')
    axes = plt.gca()
    axes.set_ylim([-0.5,0.5])
    plt.legend(loc ='upper right',numpoints = 1)
    plt.xlabel("Time (s)")
    plt.ylabel("MRPs")
    unitTestSupport.writeFigureLaTeX("scPlusMRPSwitching", "MRP Switching", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rot_ang_mom_N[:,0]*1e-9, (rot_ang_mom_mag - rot_ang_mom_mag[0])/rot_ang_mom_mag[0])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInRotationalAngularMomentumRotationOnly", "Change in Rotational Angular Momentum Rotation Only", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rot_energy[:,0]*1e-9, (rot_energy[:,1] - rot_energy[0,1])/rot_energy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInRotationalEnergyRotationOnly", "Change in Rotational Energy Rotation Only", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(omega_BNOutput[:,0]*1e-9,omega_BNOutput[:,1],label = r"$\omega_1$" + " Xmera")
    plt.plot(omega_BNOutput[:,0]*1e-9,omega_BNOutput[:,2],label = r"$\omega_2$" + " Xmera")
    plt.plot(omega_BNOutput[:,0]*1e-9,omega_BNOutput[:,3], label = r"$\omega_3$" + " Xmera")
    plt.plot(time_array*1e-9,omega_analytical_array[0,:],'bo', label = r"$\omega_1$" + " BOE")
    plt.plot(time_array*1e-9,omega_analytical_array[1,:],'go', label = r"$\omega_2$" + " BOE")
    plt.plot(time_array*1e-9,omega_analytical_array[2,:],'ro', label = r"$\omega_3$" + " BOE")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.legend(loc ='lower right',numpoints = 1, prop = {'size': 6.5})
    unitTestSupport.writeFigureLaTeX("scPlusXmeraVsBOECalcForRotation", "Xmera Vs BOE Calc For Rotation", plt, r"width=0.8\textwidth", path)
    if show_plots:
        plt.show()
        plt.close("all")

    module_output = data_log.sigma_BN
    accuracy = 1e-8
    for i in range(0,len(true_sigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(module_output[-1],true_sigma[i],3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed attitude unit test")

    accuracy = 1e-10
    for i in range(0,len(initial_rot_ang_mom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(final_rot_ang_mom[i],initial_rot_ang_mom_N[i],1,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational angular momentum unit test")

    for i in range(0,len(initial_rot_energy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(final_rot_energy[i],initial_rot_energy[i],1,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Rotation Integrated test failed rotational energy unit test")

    omega_array = (numpy.delete(omega_array, 0, 0)).transpose()
    omega_analytical_array = omega_analytical_array.transpose()
    for i in range(0,len(omega_analytical_array)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(omega_array[i],omega_analytical_array[i],3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Rotation Integrated test Rotational BOE unit test")

    accuracy = 1e-5
    if not unitTestSupport.isArrayEqualRelative(numpy.delete(sigma_after_switch, 0,), sigma_after_analytical,1,accuracy):
        test_fail_count += 1
        test_messages.append("FAILED: Spacecraft Rotation Integrated test failed MRP Switching unit test")

    if test_fail_count == 0:
        print("PASSED: " + "Spacecraft Rotation Integrated test")

    assert test_fail_count < 1, test_messages

    # return fail count and join into a single string all messages in the list
    # test_message
    return [test_fail_count, ''.join(test_messages)]

def sc_trans_boe(show_plots):
    """Module Unit Test"""
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    time_step = 0.1
    test_process_rate = macros.sec2nano(time_step)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    # Define conditions for the forces and times
    F1 = 3.
    F2 = -7.
    t1 = 3.
    t2 = 6.
    t3 = 10.

    # Add external force and torque
    ext_ft_object = extForceTorque.ExtForceTorque()
    ext_ft_object.modelTag = "externalDisturbance"
    ext_ft_object.extTorquePntB_B = [[0], [0], [0]]
    ext_ft_object.extForce_B = [[F1], [0], [0]]
    sc_object.addDynamicEffector(ext_ft_object)
    unit_test_sim.AddModelToTask(unit_task_name, ext_ft_object)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Define initial conditions of the spacecraft
    sc_object.hub.mHub = 100
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    # Set the initial values for the states
    sc_object.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    unit_test_sim.InitializeSimulation()

    stop_time = t1
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    ext_ft_object.extTorquePntB_B = [[0], [0], [0]]
    ext_ft_object.extForce_B = [[0], [0], [0]]

    stop_time = t2
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    ext_ft_object.extTorquePntB_B = [[0], [0], [0]]
    ext_ft_object.extForce_B = [[F2], [0], [0]]

    stop_time = t3
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    r_BN_NOutput = add_time_column(data_log.times(), data_log.r_BN_N)
    v_BN_NOutput = add_time_column(data_log.times(), data_log.v_BN_N)

    # BOE calcs
    a1 = F1/sc_object.hub.mHub
    a2 = F2/sc_object.hub.mHub
    v1 = a1*t1
    v2 = v1
    v3 = v2 + a2*(t3-t2)
    x1 = 0.5*v1*t1
    x2 = x1 + v2*(t2-t1)
    t0 = t2 - v2/a2
    x3 = x2 + 0.5*v2*(t0-t2) + 0.5*v3*(t3-t0)

    # truth and Xmera
    truth_v = [v1, v2, v3]
    truth_x = [x1, x2, x3]

    xmera_v = [v_BN_NOutput[int(t1/time_step), 1], v_BN_NOutput[int(t2/time_step), 1], v_BN_NOutput[int(t3/time_step), 1]]
    xmera_x = [r_BN_NOutput[int(t1/time_step), 1], r_BN_NOutput[int(t2/time_step), 1], r_BN_NOutput[int(t3/time_step), 1]]

    plt.close('all')
    plt.figure()
    plt.clf()
    plt.plot(r_BN_NOutput[:,0]*1e-9, r_BN_NOutput[:,1],'-b',label = "Xmera")
    plt.plot([t1, t2, t3], [x1, x2, x3],'ro',markersize = 6.5,label = "BOE")
    plt.xlabel('time (s)')
    plt.ylabel('X (m)')
    plt.legend(loc ='upper left',numpoints = 1)
    plot_name = "scPlusTranslationPositionBOE"
    plot_title = "Translation Position BOE"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(v_BN_NOutput[:,0]*1e-9, v_BN_NOutput[:,1],'-b',label = "Xmera")
    plt.plot([t1, t2, t3], [v1, v2, v3],'ro',markersize = 6.5,label = "BOE")
    plt.xlabel('time (s)')
    plt.ylabel('X velocity (m/s)')
    plt.legend(loc ='lower left',numpoints = 1)
    plot_name = "scPlusTranslationVelocityBOE"
    plot_title = "Translation Velocity BOE"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)
    if show_plots:
        plt.show()
        plt.close('all')

    accuracy = 1e-10
    for i in range(0,3):
        # check a vector values
        if abs((truth_x[i] - xmera_x[i])/truth_x[i]) > accuracy:
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation BOE Integrated test failed pos unit test")

    for i in range(0,3):
        # check a vector values
        if abs((truth_v[i] - xmera_v[i])/truth_v[i]) > accuracy:
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Translation BOE Integrated test failed velocity unit test")

    if test_fail_count == 0:
        print("PASSED: " + " Spacecraft Translation BOE Integrated Sim Test")

    assert test_fail_count < 1, test_messages

    # return fail count and join into a single string all messages in the list
    # test_message
    return [test_fail_count, ''.join(test_messages)]

def sc_point_b_vs_point_c(show_plots):
    """Module Unit Test"""
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.01)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    # Define location of force
    r_FBc_B = numpy.array([0.3, -0.7, 0.4])
    force_B = numpy.array([0.5, 0.6, -0.2])
    torque_pnt_C_B = numpy.cross(r_FBc_B,force_B)

    # Add external force and torque
    ext_ft_object = extForceTorque.ExtForceTorque()
    ext_ft_object.modelTag = "externalDisturbance"
    ext_ft_object.extTorquePntB_B = [[torque_pnt_C_B[0]], [torque_pnt_C_B[1]], [torque_pnt_C_B[2]]]
    ext_ft_object.extForce_B = [[force_B[0]], [force_B[1]], [force_B[2]]]
    sc_object.addDynamicEffector(ext_ft_object)
    unit_test_sim.AddModelToTask(unit_task_name, ext_ft_object)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Define initial conditions of the spacecraft
    sc_object.hub.mHub = 100
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc_object.hub.r_CN_NInit = [[0.0],	[0.0],	[0.0]]
    sc_object.hub.v_CN_NInit = [[0.0],	[0.0],	[0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    unit_test_sim.InitializeSimulation()

    stop_time = 10.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    r_CN_NOutput1 = add_time_column(data_log.times(), data_log.r_CN_N)
    sigma_BNOutput1 = add_time_column(data_log.times(), data_log.sigma_BN)

    ####################

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.01)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    # Define location of force
    r_BcB_B = numpy.array([0.4, 0.5, 0.2])
    r_FB_B = r_BcB_B + r_FBc_B
    torque_pnt_B_B = numpy.cross(r_FB_B,force_B)

    # Add external force and torque
    ext_ft_object = extForceTorque.ExtForceTorque()
    ext_ft_object.modelTag = "externalDisturbance"
    ext_ft_object.extTorquePntB_B = [[torque_pnt_B_B[0]], [torque_pnt_B_B[1]], [torque_pnt_B_B[2]]]
    ext_ft_object.extForce_B = [[force_B[0]], [force_B[1]], [force_B[2]]]
    sc_object.addDynamicEffector(ext_ft_object)
    unit_test_sim.AddModelToTask(unit_task_name, ext_ft_object)

    data_log2 = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log2)

    # Define initial conditions of the spacecraft
    sc_object.hub.mHub = 100
    sc_object.hub.r_BcB_B = [[r_BcB_B[0]], [r_BcB_B[1]], [r_BcB_B[2]]]
    sc_object.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc_object.hub.r_CN_NInit = [[0.0],	[0.0],	[0.0]]
    sc_object.hub.v_CN_NInit = [[0.0],	[0.0],	[0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    unit_test_sim.InitializeSimulation()

    stop_time = 10.0
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    r_CN_NOutput2 = add_time_column(data_log2.times(), data_log2.r_CN_N)
    sigma_BNOutput2 = add_time_column(data_log2.times(), data_log2.sigma_BN)

    plt.figure()
    plt.clf()
    plt.plot(r_CN_NOutput1[:,0]*1e-9, r_CN_NOutput1[:,1], 'k', label = 'Torque About Point C', linewidth=3.0)
    plt.plot(r_CN_NOutput1[:,0]*1e-9,r_CN_NOutput1[:,2], 'k', r_CN_NOutput1[:,0]*1e-9, r_CN_NOutput1[:,3], 'k', linewidth=3.0)
    plt.plot(r_CN_NOutput2[:,0]*1e-9, r_CN_NOutput2[:,1], '--c', label = 'Torque About Point B')
    plt.plot(r_CN_NOutput2[:,0]*1e-9,r_CN_NOutput2[:,2], '--c', r_CN_NOutput2[:,0]*1e-9, r_CN_NOutput1[:,3], '--c')
    plt.xlabel('Time (s)')
    plt.ylabel('Inertial Position (m)')
    plt.legend(loc ='upper left', handlelength=3.5)
    plot_name = "scPlusPointBVsPointCTranslation"
    plot_title = "PointB Vs PointC Translation"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(sigma_BNOutput1[:,0]*1e-9, sigma_BNOutput1[:,1], 'k', label = 'Torque About Point C', linewidth=3.0)
    plt.plot(sigma_BNOutput1[:,0]*1e-9, sigma_BNOutput1[:,2], 'k', sigma_BNOutput1[:,0]*1e-9, sigma_BNOutput1[:,3], 'k', linewidth=3.0)
    plt.plot(sigma_BNOutput2[:,0]*1e-9, sigma_BNOutput2[:,1], '--c', label = 'Torque About Point B')
    plt.plot(sigma_BNOutput2[:,0]*1e-9, sigma_BNOutput2[:,2], '--c', sigma_BNOutput2[:,0]*1e-9, sigma_BNOutput2[:,3], '--c')
    plt.xlabel('Time (s)')
    plt.ylabel('MRPs')
    plt.legend(loc ='upper right', handlelength=3.5)
    plot_name = "scPlusPointBVsPointCAttitude"
    plot_title = "PointB Vs PointC Attitude"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(plot_name, plot_title, plt, format, path)
    if show_plots:
        plt.show()
        plt.close('all')

    accuracy = 1e-8
    if not unitTestSupport.isArrayEqualRelative(r_CN_NOutput1[-1,1:4],r_CN_NOutput2[-1,1:4],3,accuracy):
        test_fail_count += 1
        test_messages.append("FAILED: Spacecraft Point B Vs Point C test failed pos unit test")

    if not unitTestSupport.isArrayEqualRelative(sigma_BNOutput1[-1,1:4],sigma_BNOutput2[-1,1:4],3,accuracy):
        test_fail_count += 1
        test_messages.append("FAILED: Spacecraft Point B Vs Point C test failed attitude unit test")

    if test_fail_count == 0:
        print("PASSED: " + " Spacecraft Point B Vs Point C Integrated Sim Test")

    assert test_fail_count < 1, test_messages

    # return fail count and join into a single string all messages in the list
    # test_message
    return [test_fail_count, ''.join(test_messages)]

@pytest.mark.parametrize("accuracy", [1e-3])
def sc_optional_ref(show_plots, accuracy):
    """Module Unit Test"""
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    time_step = 0.1
    test_process_rate = macros.sec2nano(time_step)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # add Earth
    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector(list(grav_factory.gravBodies.values()))

    # add gravity gradient effector
    gg_eff = GravityGradientEffector.GravityGradientEffector()
    gg_eff.modelTag = sc_object.modelTag
    gg_eff.addPlanetName(earth.planetName)
    sc_object.addDynamicEffector(gg_eff)
    unit_test_sim.AddModelToTask(unit_task_name, gg_eff)

    # Define initial conditions of the spacecraft
    sc_object.hub.mHub = 100
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc_object.hub.r_CN_NInit = [[7000000.0],	[0.0],	[0.0]]
    sc_object.hub.v_CN_NInit = [[7000.0],	[0.0],	[0.0]]
    sc_object.hub.sigma_BNInit = [[0.5], [0.4], [0.3]]
    sc_object.hub.sigma_BNInit = [[0.], [0.], [1.0]]
    sc_object.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    # write attitude reference message
    att_ref = messaging.AttRefMsgPayload()
    att_ref.sigma_RN = [0.0, 0.0, 1.0]
    att_ref.omega_RN_N = [0.0001, 0.0002, 0.0003]
    att_ref_msg = messaging.AttRefMsg().write(att_ref)
    sc_object.attRefInMsg.subscribeTo(att_ref_msg)

    # write translational reference message
    trans_ref = messaging.TransRefMsgPayload()
    trans_ref.r_RN_N = [1000000, 2000000, 3000000]
    trans_ref.v_RN_N = [2000, 3000, 4000]
    trans_ref_msg = messaging.TransRefMsg().write(trans_ref)
    sc_object.transRefInMsg.subscribeTo(trans_ref_msg)

    unit_test_sim.InitializeSimulation()

    stop_time = 0.2
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    omega_out = data_log.omega_BN_B
    sigma_out = data_log.sigma_BN
    r_RN_Out = data_log.r_BN_N
    v_RN_Out = data_log.v_BN_N

    true_sigma = [att_ref.sigma_RN]*3
    true_omega = [[-0.0001, -0.0002, 0.0003]]*3
    truer_RN_N = [trans_ref.r_RN_N]*3
    truev_RN_N = [trans_ref.v_RN_N]*3

    test_fail_count, test_messages = unitTestSupport.compareArray(true_sigma, sigma_out,
                                                               accuracy, "sigma_BN",
                                                               test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareArray(true_omega, omega_out,
                                                               accuracy, "omega_BN_B",
                                                               test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareArray(truer_RN_N, r_RN_Out,
                                                               accuracy, "r_RN_N",
                                                               test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareArray(truev_RN_N, v_RN_Out,
                                                               accuracy, "v_RN_N",
                                                               test_fail_count, test_messages)

    if test_fail_count == 0:
        print("PASSED: scPlus setting optional reference state input message")
    else:
        print("FAILED: scPlus setting optional reference state input message")

    assert test_fail_count < 1, test_messages

    return [test_fail_count, ''.join(test_messages)]

def sc_accum_dv():
    """Module Unit Test"""
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    time_step = 0.1
    test_process_rate = macros.sec2nano(time_step)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # add Earth
    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector(list(grav_factory.gravBodies.values()))

    # Define initial conditions of the spacecraft
    sc_object.hub.mHub = 100
    sc_object.hub.r_BcB_B = [[0.0], [100.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    sc_object.hub.r_CN_NInit = [[-7000000.0],	[0.0],	[0.0]]
    sc_object.hub.v_CN_NInit = [[0.0],	[7000.0],	[0.0]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [numpy.pi/180]]

    unit_test_sim.InitializeSimulation()

    stop_time = 0.5
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    data_accum_dv_CN_B = data_log.TotalAccumDVBdy
    data_accum_dv_BN_B = data_log.TotalAccumDV_BN_B
    data_accum_dv_CN_N = data_log.TotalAccumDV_CN_N

    accuracy = 1e-10
    truth_data_accum_dv_CN_B = [0.0, 0.0, 0.0]
    truth_data_accum_dv_CN_N = [0.0, 0.0, 0.0]
    v_r = numpy.cross(numpy.array(sc_object.hub.omega_BN_BInit).T, -numpy.array(sc_object.hub.r_BcB_B).T)[0]
    truth_data_accum_dv_BN_B = numpy.zeros(3)
    for i in range(len(data_log.times())-1):
        if not unitTestSupport.isArrayEqual(data_accum_dv_CN_B[i+1],truth_data_accum_dv_CN_B,3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Point C Accumulated DV test failed pos unit test")

        truth_data_accum_dv_BN_B += numpy.matmul(RigidBodyKinematics.MRP2C(data_log.sigma_BN[i+1]),
                                              numpy.matmul(RigidBodyKinematics.MRP2C(data_log.sigma_BN[i+1]).T,v_r) -
                                              numpy.matmul(RigidBodyKinematics.MRP2C(data_log.sigma_BN[i]).T,v_r))
        if not unitTestSupport.isArrayEqual(data_accum_dv_BN_B[i+1],truth_data_accum_dv_BN_B,3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Point B Accumulated DV test failed pos unit test")

        if not unitTestSupport.isArrayEqual(data_accum_dv_CN_N[i+1],truth_data_accum_dv_CN_N,3,accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Point C Accumulated DV in inertial frame test failed pos unit test")

    if test_fail_count == 0:
        print("PASSED: Spacecraft Accumulated DV tests with offset CoM")

    return [test_fail_count, ''.join(test_messages)]


def sc_accum_dv_ext_force():
    """Module Unit Test"""
    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    sc_object = spacecraft.Spacecraft()
    sc_object.modelTag = "spacecraftBody"

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    time_step = 0.1
    test_process_rate = macros.sec2nano(time_step)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, sc_object)

    # Add external force and torque
    ext_ft_object = extForceTorque.ExtForceTorque()
    ext_ft_object.modelTag = "externalDisturbance"
    ext_ft_object.extTorquePntB_B = [[0], [0], [0]]
    ext_force = numpy.array([random() for _ in range(3)])
    ext_ft_object.extForce_B = [[item] for item in ext_force]
    sc_object.addDynamicEffector(ext_ft_object)
    unit_test_sim.AddModelToTask(unit_task_name, ext_ft_object)

    # Define initial conditions of the spacecraft
    sc_object.hub.mHub = 100

    data_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    stop_time = 0.5
    unit_test_sim.ConfigureStopTime(macros.sec2nano(stop_time))
    unit_test_sim.ExecuteSimulation()

    data_accum_dv_CN_N = data_log.TotalAccumDV_CN_N
    time_array_sec = data_log.times() * macros.NANO2SEC

    accuracy = 1e-10
    for i in range(len(data_log.times())):
        truth_data_accum_dv_CN_N = ext_force * time_array_sec[i] / sc_object.hub.mHub
        if not unitTestSupport.isArrayEqual(data_accum_dv_CN_N[i], truth_data_accum_dv_CN_N, 3, accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: Spacecraft Point C Accumulated DV with external force test failed unit test")

    if test_fail_count == 0:
        print("PASSED: Spacecraft Accumulated DV tests with offset CoM")

    return [test_fail_count, ''.join(test_messages)]


def test_spacecraft_free_fall_delta_v(show_plots):
    """A spacecraft in free fall pushed by a small constant force: the accumulated delta-v and the non-conservative
    acceleration must come only from that force, with no round-off from the much larger orbital velocity.

    The mass and force are powers of two, so the acceleration is exact, and the force acts along the center of mass
    offset, so the hub never rotates. Each step must then add exactly the RK4 integral of that acceleration over the
    step the spacecraft integrates, which it finds from the simulation times in seconds."""
    time_step = 1.0
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_proc = unit_test_sim.CreateNewProcess("TestProcess")
    test_proc.addTask(unit_test_sim.CreateNewTask("unitTask", macros.sec2nano(time_step)))

    sc_object = spacecraft.Spacecraft()
    sc_object.hub.mHub = 512.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    unit_test_sim.AddModelToTask("unitTask", sc_object)

    ext_force = extForceTorque.ExtForceTorque()
    ext_force.extForce_N = [[0.0], [0.0], [0.5]]
    sc_object.addDynamicEffector(ext_force)
    unit_test_sim.AddModelToTask("unitTask", ext_force)

    earth_grav_body = gravityEffector.GravBodyData()
    earth_grav_body.planetName = "earth_planet_data"
    earth_grav_body.mu = 0.3986004415E+15  # meters!
    earth_grav_body.isCentralBody = True
    sc_object.gravField.gravBodies = spacecraft.GravBodyVector([earth_grav_body])

    sc_log = sc_object.scStateOutMsg.recorder()
    unit_test_sim.AddModelToTask("unitTask", sc_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1000.0))
    unit_test_sim.ExecuteSimulation()

    times = sc_log.times() * macros.NANO2SEC
    outputs = {
        "TotalAccumDVBdy": sc_log.TotalAccumDVBdy,
        "TotalAccumDV_BN_B": sc_log.TotalAccumDV_BN_B,
        "TotalAccumDV_CN_N": sc_log.TotalAccumDV_CN_N,
        "nonConservativeAccelpntB_B": sc_log.nonConservativeAccelpntB_B,
    }

    # Delta-v of each RK4 step of the constant acceleration, summed in the integrator's order
    accel = 0.5 / 512.0
    accum_dv = numpy.zeros((len(times), 3))
    non_conservative_accel = numpy.zeros((len(times), 3))
    for i in range(1, len(times)):
        step = times[i] - times[i - 1]
        step_dv = 0.0
        for weight in [step / 6.0, step / 3.0, step / 3.0, step / 6.0]:
            step_dv += weight * accel
        accum_dv[i, 2] = accum_dv[i - 1, 2] + step_dv
        non_conservative_accel[i, 2] = step_dv / step
    expected = {
        "TotalAccumDVBdy": accum_dv,
        "TotalAccumDV_BN_B": accum_dv,
        "TotalAccumDV_CN_N": accum_dv,
        "nonConservativeAccelpntB_B": non_conservative_accel,
    }

    plt.close("all")
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
    for ax, (name, values) in zip(axes, outputs.items()):
        ax.plot(times, values - expected[name])
        ax.set_ylabel(name + " error")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    if show_plots:
        plt.show()
    plt.close("all")

    for name, values in outputs.items():
        numpy.testing.assert_array_equal(values, expected[name], err_msg=name)


if __name__ == "__main__":
    # scAttRef(True, 1e-3)
    # sc_translation(True)
    # sc_trans_and_rotation(True)
    # sc_rotation(True)
    # sc_trans_boe(True)
    # sc_point_b_vs_point_c(True)
    sc_optional_ref(True, 0.001)
    # sc_accum_dv()
    # sc_optional_ref(True, 0.001)
    # sc_accum_dv()
    # sc_accum_dv_ext_force()
    # test_spacecraft_free_fall_delta_v(True)
