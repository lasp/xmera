import numpy as np
import spiceypy

from xmera import __path__
from xmera.simulation import ephemerisConverter, groundLocation, eclipse
from xmera.utilities import macros as mc, simIncludeGravBody

bskPath = __path__[0]


class BSKEnvironmentModel:
    """Defines the Earth Environment."""
    def __init__(self, SimBase, envRate):
        # Define empty class variables
        self.mu = None
        self.planetRadius = None
        self.sun = None
        self.earth = None
        self.moon = None

        # Define process name, task name and task time-step
        self.envTaskName = "EnvironmentTask"
        processTasksTimeStep = mc.sec2nano(envRate)

        # Create task
        SimBase.envProc.addTask(SimBase.CreateNewTask(self.envTaskName, processTasksTimeStep))

        # Instantiate Env modules as objects
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.ephemObject = ephemerisConverter.EphemerisConverter()
        self.eclipseObject = eclipse.Eclipse()
        self.groundStation = groundLocation.GroundLocation()

        # Initialize all modules and write init one-time messages
        self.InitAllEnvObjects()

        # Add modules to environment task
        SimBase.AddModelToTask(self.envTaskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.ephemObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.eclipseObject, 200)
        SimBase.AddModelToTask(self.envTaskName, self.groundStation, 200)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetGravityBodies(self):
        """
        Specify what gravitational bodies to include in the simulation.
        """
        # Create gravity bodies
        gravBodies = self.gravFactory.createBodies(['sun', 'earth', 'moon'])
        gravBodies['earth'].isCentralBody = True
        self.mu = self.gravFactory.gravBodies['earth'].mu
        self.planetRadius = self.gravFactory.gravBodies['earth'].radEquator
        self.sun = 0
        self.earth = 1
        self.moon = 2

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
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.moon])

    def SetEclipseObject(self):
        """
        Specify what celestial object is causing an eclipse message.
        """
        self.eclipseObject.modelTag = "eclipseObject"
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        # add all celestial objects in spiceObjects except for the sun (0th object)
        for item in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[item])

    def SetGroundLocations(self):
        """
        Specify which ground locations are of interest.
        """
        self.groundStation.modelTag = "BoulderGroundStation"
        self.groundStation.planetRadius = self.planetRadius
        self.groundStation.specifyLocation(np.radians(40.009971), np.radians(-105.243895), 1624)
        self.groundStation.minimumElevation = np.radians(10.)
        self.groundStation.maximumRange = 1e9

    # Global call to initialize every module
    def InitAllEnvObjects(self):
        self.SetGravityBodies()
        self.SetEpochObject()
        self.SetEclipseObject()
        self.SetGroundLocations()
