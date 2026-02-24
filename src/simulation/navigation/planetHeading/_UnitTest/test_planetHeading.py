# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import pytest
from xmera.architecture import messaging
from xmera.simulation import planetHeading
from xmera.utilities import SimulationBaseClass
from xmera.utilities import orbitalMotion as om


def test_planetHeading(show_plots=False, relTol=1e-8):
    """
    **Test Description**

    Test that a planet heading is properly calculated from a spacecraft and planet position and spacecraft attitude.
    To test this, the earth is placed at the inertial origin. A spacecraft with inertial attitude is placed
    at 1AU in the z-direction.  The heading is checked to be [0, 0, -1].
    These values were chosen arbitrarily. They are checked to be accurate to within a relative tolerance of the
    input ``relTol``, 1e-8 by default.

    Args:
        relTol (float): positive, the relative tolerance to which the result is checked.

    **Variables Being Tested**

    This test checks that ``headingOut`` stores the pulled log of the module ``bodyHeadingOutMsg``.

"""
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    earthPositionMessage = messaging.SpicePlanetStateMsgPayload()
    earthPositionMessage.PositionVector = [0., 0., 0.]
    plMsg = messaging.SpicePlanetStateMsg().write(earthPositionMessage)

    scPositionMessage = messaging.SCStatesMsgPayload()
    scPositionMessage.r_BN_N = [0., 0., om.AU*1000]
    scMsg = messaging.SCStatesMsg().write(scPositionMessage)

    ph = planetHeading.PlanetHeading()
    ph.modelTag = "planetHeading"
    sim.AddModelToTask(task.Name, ph)

    ph.planetPositionInMsg.subscribeTo(plMsg)
    ph.spacecraftStateInMsg.subscribeTo(scMsg)

    dataLog = ph.planetHeadingOutMsg.recorder()
    sim.AddModelToTask(task.Name, dataLog)

    sim.InitializeSimulation()
    sim.TotalSim.singleStepProcesses()
    headingOut = dataLog.rHat_XB_B[-1]

    assert headingOut == pytest.approx([0., 0., -1.], rel=relTol)


if __name__ == "__main__":
    test_planetHeading()
