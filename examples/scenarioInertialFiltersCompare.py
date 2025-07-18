import matplotlib.pyplot as plt
import numpy as np
import time as pytime
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import inertialAttitudeUkf, inertialUKF
from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.utilities import RigidBodyKinematics as rbk

starOnly = inertialAttitudeUkf.AttitudeFilterMethod_StarOnly

def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))

def rk4(f, t, x0, Inertia=np.eye(3), mrpShadow=True):
    x = np.zeros([len(t), len(x0) + 1])
    h = (t[len(t) - 1] - t[0]) / len(t)
    x[0, 0] = t[0]
    x[0, 1:] = x0
    for i in range(len(t) - 1):
        h = t[i + 1] - t[i]
        x[i, 0] = t[i]
        k1 = h * f(t[i], x[i, 1:], Inertia)
        k2 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k1, Inertia)
        k3 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k2, Inertia)
        k4 = h * f(t[i] + h, x[i, 1:] + k3, Inertia)
        x[i + 1, 1:] = x[i, 1:] + (k1 + 2. * k2 + 2. * k3 + k4) / 6.
        if mrpShadow:
            s = np.linalg.norm(x[i + 1, 1:4])**2
            if s > 1:
                x[i + 1, 1:4] = - (x[i + 1, 1:4]) / s
        x[i + 1, 0] = t[i + 1]
    return x

def attitude_dynamics(t, x, I):
    dxdt = np.zeros(np.shape(x))
    mrp = x[:3]
    omega = x[3:]
    B = rbk.BmatMRP(mrp)
    dxdt[:3] = 0.25 * np.matmul(B, omega)
    dxdt[3:] = - np.dot(np.linalg.inv(I), np.cross(omega, np.dot(I, omega)))
    return dxdt

def setup_filter_data_attukf(filterObject):
    filterObject.setAlpha(0.02)
    filterObject.setBeta(2.0)

    filterObject.setInitialPosition([0.05, 0.5, 0.1])
    filterObject.setInitialVelocity([0.02, -0.005, 0.01])
    filterObject.setInitialCovariance([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.01]])
    filterObject.setGyroNoise([[1e-6,0,0],[0.,1e-6,0],[0,0,1e-6]])
    sigmaMrpSquare = (1E-3) ** 2
    sigmaRateSquare = (5E-4) ** 2
    filterObject.setProcessNoise([[sigmaMrpSquare, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, sigmaMrpSquare, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, sigmaMrpSquare, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, sigmaRateSquare, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, sigmaRateSquare, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, 0.0, sigmaRateSquare]])


def setup_filter_data_ukf(filterObject):
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0
    filterObject.switchMag = 1.2
    filterObject.stateInit = [0.05, 0.5, 0.1, 0.02, -0.005, 0.01]
    filterObject.covarInit = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
    sigmaMrpSquare = (1E-3) ** 2
    sigmaRateSquare = (5E-4) ** 2
    qNoise = np.identity(6)
    qNoise[0:3, 0:3] = qNoise[0:3, 0:3]*sigmaMrpSquare
    qNoise[3:6, 3:6] = qNoise[3:6, 3:6]*sigmaRateSquare
    filterObject.qNoise = qNoise.reshape(36).tolist()


def test_inertialAttitudeUKF(filter, showPlots, initialError):
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)
    np.random.seed(1)
    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    step_size = 1
    test_process_rate = macros.sec2nano(step_size)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Create filter
    if(filter == "InertialAttitudeUKF"):
        inertialAttFilter = inertialAttitudeUkf.InertialAttitudeUkf(starOnly)
    else:
        inertialAttFilter = inertialUKF.InertialUKF()
    unit_test_sim.AddModelToTask(unit_task_name, inertialAttFilter)

    # Setup filter data
    if(filter == "InertialAttitudeUKF"):
        setup_filter_data_attukf(inertialAttFilter)
    else:
        setup_filter_data_ukf(inertialAttFilter)

    rw_orientation_list = [
        0.70710678118654746, -0.5, 0.5,
        0.70710678118654746, -0.5, -0.5,
        0.70710678118654746, 0.5, -0.5,
        0.70710678118654746, 0.5, 0.5
    ]

    rw_inertia_list = [5, 10, 5, 10]

    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]

    vehicle_config_data = messaging.VehicleConfigMsgPayload()
    vehicle_config_data.ISCPntB_B = I
    vehicle_config = messaging.VehicleConfigMsg().write(vehicle_config_data)
    if(filter == "InertialAttitudeUKF"):
        inertialAttFilter.vehicleConfigMsg.subscribeTo(vehicle_config)
    else:
        inertialAttFilter.massPropsInMsg.subscribeTo(vehicle_config)

    rw_data_msg = messaging.RWArrayConfigMsgPayload()
    rw_data_msg.numRW = 4
    rw_data_msg.GsMatrix_B = rw_orientation_list
    rw_data_msg.JsList = rw_inertia_list
    rw_msg = messaging.RWArrayConfigMsg().write(rw_data_msg)
    if(filter == "InertialAttitudeUKF"):
        inertialAttFilter.rwArrayConfigMsg.subscribeTo(rw_msg)
    else:
        inertialAttFilter.rwParamsInMsg.subscribeTo(rw_msg)

    rw_speeds_data = messaging.RWSpeedMsgPayload()
    for i in range(rw_data_msg.numRW):
        rw_speeds_data.wheelSpeeds[i] = i%2*200
    rw_speeds = messaging.RWSpeedMsg().write(rw_speeds_data)
    if(filter == "InertialAttitudeUKF"):
        inertialAttFilter.rwSpeedMsg.subscribeTo(rw_speeds)
    else:
        inertialAttFilter.rwSpeedsInMsg.subscribeTo(rw_speeds)

    # Initial state of filter
    initial_condition = np.zeros(6)
    if(filter == "InertialAttitudeUKF"):
        initial_condition[:3] = np.array(inertialAttFilter.getInitialPosition()).reshape(3)
        initial_condition[3:] = np.array(inertialAttFilter.getInitialVelocity()).reshape(3)
    else:
        initial_condition = np.array(inertialAttFilter.stateInit).reshape(6)
    if initialError:
        if(filter == "InertialAttitudeUKF"):
            inertialAttFilter.setInitialPosition([0, 0, 0])
            inertialAttFilter.setInitialVelocity([0, 0, 0])
        else:
            inertialAttFilter.stateInit = [0, 0, 0, 0, 0, 0]

    # Setup two star tracker measurements
    st_1_data = messaging.STAttMsgPayload()
    st_1_data.MRP_BdyInrtl = initial_condition[:3]
    st_1_data.dcm_CB = np.eye(3).flatten()
    st_1_data.timeTag = 0
    star_tracker1 = inertialAttitudeUkf.StarTrackerMessage()
    st_1_msg = messaging.STAttMsg().write(st_1_data)

    st_2_data = messaging.STAttMsgPayload()
    st_2_data.MRP_BdyInrtl = initial_condition[:3]
    st_2_data.dcm_CB = np.eye(3).flatten()
    st_2_data.timeTag = 0
    star_tracker2 = inertialAttitudeUkf.StarTrackerMessage()
    st_2_msg = messaging.STAttMsg().write(st_2_data)

    st_sigma_2 = 1e-4
    if(filter == "InertialAttitudeUKF"):
        star_tracker1.starTrackerMsg.subscribeTo(st_1_msg)
        star_tracker1.measurementNoise_C = [[st_sigma_2, 0, 0], [0,st_sigma_2,0], [0,0,st_sigma_2]]
        inertialAttFilter.addStarTrackerInput(star_tracker1)
        star_tracker2.starTrackerMsg.subscribeTo(st_2_msg)
        star_tracker2.measurementNoise_C = [[st_sigma_2, 0, 0], [0,st_sigma_2,0], [0,0,st_sigma_2]]
        inertialAttFilter.addStarTrackerInput(star_tracker2)
    else:
        starTracker1 = inertialUKF.STMessage()
        starTracker1.noise = [st_sigma_2, 0.0, 0.0,
                               0.0, st_sigma_2, 0.0,
                               0.0, 0.0, st_sigma_2]
        starTracker2 = inertialUKF.STMessage()
        starTracker2.noise = [st_sigma_2, 0.0, 0.0,
                               0.0, st_sigma_2, 0.0,
                               0.0, 0.0, st_sigma_2]
        star_tracker_list = [starTracker1, starTracker2]
        inertialAttFilter.STDatasStruct.STMessages = star_tracker_list
        inertialAttFilter.STDatasStruct.numST = len(star_tracker_list)
        inertialAttFilter.STDatasStruct.STMessages[0].stInMsg.subscribeTo(st_1_msg)
        inertialAttFilter.STDatasStruct.STMessages[1].stInMsg.subscribeTo(st_2_msg)

    if(filter == "InertialAttitudeUKF"):
        imu_data = messaging.IMUSensorMsgPayload()
        imu_measurement = messaging.IMUSensorMsg().write(imu_data)
        inertialAttFilter.imuSensorDataInMsg.subscribeTo(imu_measurement)
    else:
        accelMeasurement = messaging.AccDataMsg()
        inertialAttFilter.gyrBuffInMsg.subscribeTo(accelMeasurement)

    # Log
    if(filter == "InertialAttitudeUKF"):
        filter_data_log = inertialAttFilter.inertialFilterOutputMsg.recorder()
    else:
        filter_data_log = inertialAttFilter.filtDataOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)

    # Start simulation
    sim_time = 500
    time = np.linspace(0, sim_time, sim_time+1)
    expected = np.zeros([len(time), 7])
    expected[:int(len(time)/2), :] = rk4(attitude_dynamics, time[:int(len(time)/2)], initial_condition,
                                         np.array(I).reshape([3,3]))
    kick = np.array([0.1, 0.01, -0.5, 1E-1, 1E-2, 1E-1])
    initial_condition = expected[int(len(time)/2) -1, 1:] + kick
    expected[int(len(time)/2):, :] = rk4(attitude_dynamics, time[int(len(time)/2):], initial_condition,
                                         np.array(I).reshape([3,3]))

    unit_test_sim.InitializeSimulation()
    runtimeStart = pytime.time()
    for i in range(len(time)-1):
        for k in range(rw_data_msg.numRW):
            rw_speeds_data.wheelSpeeds[k] = k%2*200
        rw_speeds.write(rw_speeds_data, int((i+1)*1E9))
        if (10 < i < sim_time / 4) or (sim_time / 2 < i):
            if time[i+1]%2 == 0:
                st_1_data.timeTag = time[i+1]
                st_1_data.valid = True
                st_1_data.MRP_BdyInrtl = expected[i+1, 1:4] + np.random.normal(0, np.sqrt(st_sigma_2), 3)
                st_1_msg.write(st_1_data, int(time[i+1]*1E9))

            if time[i+1]%2 == 1 and i:
                st_2_data.timeTag = time[i+1]
                st_2_data.valid = True
                st_2_data.MRP_BdyInrtl = expected[i+1, 1:4] + np.random.normal(0, np.sqrt(st_sigma_2), 3)
                st_2_msg.write(st_2_data, int(time[i+1]*1e9))

        unit_test_sim.ConfigureStopTime(macros.sec2nano((time[i+1])))
        unit_test_sim.ExecuteSimulation()
    runtimeEnd = pytime.time() - runtimeStart

    num_states = 6
    state_data_log = add_time_column(filter_data_log.times(), filter_data_log.state[:, :num_states])
    diff = np.copy(state_data_log)
    for i in range(sim_time+1):
        diff[i, 1:4] = rbk.subMRP(diff[i, 1:4], expected[i, 1:4])
        diff[i, 4:] -= expected[i, 4:]

    if(showPlots):
        plt.figure(1)
        plt.plot(diff[:, 1:])
        plt.ylabel('state estimation error')
        plt.xlabel('t, sec')
        plt.title(filter)
        plt.show()

    return runtimeEnd


if __name__ == "__main__":
    runtimeUKF = test_inertialAttitudeUKF("InertialUKF",True, True)
    runtimeAttUKF = test_inertialAttitudeUKF("InertialAttitudeUKF",True, True)
    # static width version
    colWidth = 20
    # Header
    print(f"{'Filter':<{colWidth}} | {'Runtime (s)':>12}")
    print("-" * (colWidth + 3 + 12))
    # Rows
    print(f"{'InertialUKF':<{colWidth}} | {runtimeUKF:12.3f}")
    print(f"{'inertialAttitudeUKF':<{colWidth}} | {runtimeAttUKF:12.3f}")
