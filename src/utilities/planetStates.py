from xmera import __path__
import spiceypy

bskPath = __path__[0]

def planetPositionVelocity(planetName, time, ephemerisPath = '/supportData/EphemerisData/pck00010.tpc', observer = 'SSB', frame = 'J2000'):
    """
        A convenience function to get planet position from spice

        Parameters
        ----------
        planetName : name of planet to get position of
            planet name must be a valid SPICE celestial body string.
        time : UTC time as string
        ephemerisPath : a string path to ephemeris file if something other than the default is desired
        observer : observer to get vectors relative to

        Returns
        -------
        position and velocity vector of planet in Solar System Barycenter inertial frame as lists [m], [m/s]
    """

    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/de430.bsp')
    spiceypy.furnsh(bskPath + '/supportData/EphemerisData/naif0012.tls') #load leap seconds
    spiceypy.furnsh(bskPath + ephemerisPath)
    et = spiceypy.str2et(time)
    [state, _] = spiceypy.spkezr(planetName, et, frame, "NONE", observer)
    position = state[0:3] * 1000
    velocity = state[3:6] * 1000
    spiceypy.unload(bskPath + ephemerisPath)

    return position, velocity # [m], [m/s]
