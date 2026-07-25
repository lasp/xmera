# SPDX-License-Identifier: ISC
# Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import spiceypy

from xmera.utilities import macros as mc, simIncludeGravBody
from xmera.simulation import ephemerisConverter

from xmera import __path__

bskPath = __path__[0]


class BSKEnvironmentModel:
    """Defines the Earth Environment."""
    def __init__(self, SimBase, envRate):
        # Define empty class variables
        self.mu = None
        self.planetRadius = None
        self.sun = None
        self.earth = None

        # Define process name, task name and task time-step
        self.envTaskName = "EnvironmentTask"
        processTasksTimeStep = mc.sec2nano(envRate)

        # Create task
        SimBase.envProc.addTask(self.envTaskName, processTasksTimeStep)

        # Instantiate Env modules as objects
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.ephemObject = ephemerisConverter.EphemerisConverter()

        # Initialize all modules and write init one-time messages
        self.InitAllEnvObjects()

        # Add modules to environment task
        SimBase.AddModelToTask(self.envTaskName, self.gravFactory.spiceObject, None, 200)
        SimBase.AddModelToTask(self.envTaskName, self.ephemObject, None, 200)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetGravityBodies(self):
        """
        Specify what gravitational bodies to include in the simulation.
        """
        # Create gravity bodies
        gravBodies = self.gravFactory.createBodies(['sun', 'earth'])
        gravBodies['sun'].isCentralBody = True
        self.mu = self.gravFactory.gravBodies['sun'].mu
        self.planetRadius = self.gravFactory.gravBodies['sun'].radEquator
        self.sun = 0
        self.earth = 1

        # Override information with SPICE
        timeInitString = "2021 MAY 04 07:47:48.965 (UTC)"
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                              timeInitString,
                                              epochInMsg=True)
        self.gravFactory.spiceObject.zeroBase = 'Earth'

        # Load spice kernels
        spiceypy.furnsh(self.gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
        spiceypy.furnsh(self.gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
        spiceypy.furnsh(self.gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
        spiceypy.furnsh(self.gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants

    def SetEpochObject(self):
        """
        Add the ephemeris object to use with the SPICE library.
        """

        # self.epochMsg = self.gravFactory.epochMsg
        self.ephemObject.modelTag = 'EphemData'
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.earth])

    # Global call to initialize every module
    def InitAllEnvObjects(self):
        self.SetGravityBodies()
        self.SetEpochObject()
