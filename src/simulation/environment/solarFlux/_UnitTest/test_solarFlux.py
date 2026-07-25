# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.simulation import solarFlux
from xmera.utilities import SimulationBaseClass
from xmera.utilities import orbitalMotion as om


@pytest.mark.parametrize("positionFactor, shadowFactor, eclipseMsgName, relTol", [(np.sqrt(2), 0.5, "eclipse_data_0", 1e-8), (np.sqrt(2), 0.5, "", 1e-8)])
def test_solarFlux(show_plots, positionFactor, shadowFactor, eclipseMsgName, relTol):
    """
    **Test Description**

    Test that solar flux is appropriately modified depending on spacecraft distance from the sun.
    To test this, the module is asked to write the solar flux at 1 AU. Then it is asked to write
    the flux at ``positionFactor*AU`` and the flux is checked to be ``positionFactor**2`` of that at 1 AU to within
    a relative tolerance of relTol.
    The application of the shadowFactor is also checked as a multiple of the un-shadowed flux.

    Args:
        positionFactor (float): positive,
            a factor by which to multiply the original s/c position to check flux at a new position
        shadowFactor (float): between 0 and 1,
            the eclipse factor by which to multiple the solar flux at a position
        relTol (float): positive, the relative tolerance to which the result is checked.
    """

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    proc.addTask("task", int(1e9))

    sunPositionMessage = messaging.SpicePlanetStateMsgPayload()
    sunPositionMessage.PositionVector = [0., 0., 0.]
    sunMsg = messaging.SpicePlanetStateMsg().write(sunPositionMessage)

    scPositionMessage = messaging.SCStatesMsgPayload()
    scPositionMessage.r_BN_N = [0., 0., om.AU*1000]
    scMsg = messaging.SCStatesMsg().write(scPositionMessage)

    eclipseMessage = messaging.EclipseMsgPayload()
    eclipseMessage.shadowFactor = shadowFactor
    eclMsg = messaging.EclipseMsg().write(eclipseMessage)

    sf = solarFlux.SolarFlux()
    sim.AddModelToTask("task", sf)
    sf.sunPositionInMsg.subscribeTo(sunMsg)
    sf.spacecraftStateInMsg.subscribeTo(scMsg)
    sf.eclipseInMsg.subscribeTo(eclMsg)

    dataLog = sf.solarFluxOutMsg.recorder()
    sim.AddModelToTask("task", dataLog)

    sim.InitializeSimulation()
    sim.TotalSim.singleStepProcesses()

    fluxOutEarth = dataLog.flux
    scPositionMessage.r_BN_N = [0., 0., positionFactor * om.AU*1000]
    scMsg.write(scPositionMessage)

    sim.TotalSim.singleStepProcesses()
    fluxOutFurther = dataLog.flux

    assert fluxOutFurther[1] == pytest.approx(fluxOutEarth[0] / shadowFactor / (positionFactor**2) * shadowFactor, rel=relTol)


if __name__ == "__main__":
    test_solarFlux(False, np.sqrt(2.0), 0.5, "eclipse_data_0", 1e-8)
