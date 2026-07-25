# SPDX-License-Identifier: ISC
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import os
import tempfile

import numpy as np
from xmera.simulation import spacecraft
import spiceypy
from xmera.utilities import (
    SimulationBaseClass,
    macros,
    spice_utilities,
    simIncludeGravBody,
    RigidBodyKinematics as rbk
)


def test_ck_read_write(show_plots):
    simulation = SimulationBaseClass.SimBaseClass()

    process = simulation.CreateNewProcess("testProcess")
    taskName = "task"
    dynTaskRate = macros.sec2nano(1.0)
    process.addTask(taskName, dynTaskRate)

    scObject = spacecraft.Spacecraft()
    scObject.modelTag = "spacecraft"
    simulation.AddModelToTask(taskName, scObject)

    scObject.hub.mHub = 750.0
    scObject.hub.IHubPntBc_B = np.array([[900., 0., 0.],
                                         [0., 800., 0.],
                                         [0., 0., 600.]])

    scObject.hub.sigma_BNInit = [[0.1], [-0.2], [0.3]]
    scObject.hub.omega_BN_BInit = [[0.01], [-0.01], [0.03]]

    # Load up the leap second and spacecraft SPICE kernels
    gravFactory = simIncludeGravBody.gravBodyFactory()
    timeInit = 'FEB 01, 2021 12:00:00 (UTC)'
    spiceObject = gravFactory.createSpiceInterface(time=timeInit)
    spiceypy.furnsh(spiceObject.SPICEDataPath + 'naif0011.tls')  # leap second file
    spiceypy.furnsh(spiceObject.SPICEDataPath + 'MVN_SCLKSCET.00000.tsc')  # spacecraft clock file

    scObjectLogger = scObject.scStateOutMsg.recorder(dynTaskRate)
    simulation.AddModelToTask(taskName, scObjectLogger)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(59))  # run for 59 seconds for easy time logic
    simulation.ExecuteSimulation()

    # Write a CK file using the attitude data from the simulation
    timeWrite = scObjectLogger.times()
    sigmaWrite = scObjectLogger.sigma_BN
    omegaWrite = scObjectLogger.omega_BN_B

    with tempfile.TemporaryDirectory() as tempDirectory:
        tempFileName = os.path.join(tempDirectory, "test.bc")
        spice_utilities.ckWrite(tempFileName, timeWrite, sigmaWrite, omegaWrite, timeInit, spacecraft_id=-202)

        # Read the same CK file to check if the values are identical
        spice_utilities.ckInitialize(tempFileName)
        sigmaRead = np.empty_like(sigmaWrite)
        omegaRead = np.empty_like(omegaWrite)
        for idx in range(len(timeWrite)):
            # Change the time string to account for increasing time
            timeString = timeInit[:19] + f"{int(timeWrite[idx] * macros.NANO2SEC):02}" + timeInit[21:]
            _, kernQuat, kernOmega = spice_utilities.ckRead(timeString, spacecraft_id=-202)

            sigmaRead[idx, :] = - rbk.EP2MRP(kernQuat)  # Convert from JPL-style quaternion notation
            omegaRead[idx, :] = kernOmega

        # Compare the read and write data
        np.testing.assert_allclose(sigmaRead, sigmaWrite)
        np.testing.assert_allclose(omegaRead, omegaWrite)


if __name__ == "__main__":
    test_ck_read_write(True)
