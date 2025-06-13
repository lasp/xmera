
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


import os
from datetime import datetime

import numpy
import spiceypy
from Basilisk import __path__
from Basilisk.utilities import RigidBodyKinematics, macros

bskPath = __path__[0]
from Basilisk.architecture.messaging import EpochMsgPayload, EpochMsg

def ckWrite(handle, time, mrp_array, av_array, start_seg, spacecraft_id=-62, reference_frame="J2000"):
    """
    Purpose: Creates a CK kernel from a time_array, mrp_array, and an av_array. Assumes that the SCLK is furnshed

    .. warning::

        time stamps for the time_array, mrp_array, and av_array must line up exactly!!

    :param handle: What you would like the CK file to be named. Note, it must be in double quotes and end in .bc, ex: "moikernel.bc"
    :param time: numpy array of time stamps in nanoseconds
    :param mrp_array: array of modified Rodriguez parameters in column order x, y, z
    :param av_array: array of angular velocities about 3 axis in column order x, y, z
    :param start_seg: the SCLK time that the file begins at in UTC Gregorian ex: 'FEB 01,2021  12:00:55.9999 (UTC)'
    :param spacecraft_id: spacecraft ID ex:-62
    :param reference_frame: reference frame ex:"J2000"
    :return:
    """
    try:
        os.remove(handle)
    except OSError:
        pass

    # Open the CK file
    file_handle = spiceypy.ckopn(handle, "my-ckernel", 0)

    # Create empty containers for time, attitude and angular velocity
    num_data_points = len(time)
    times = numpy.zeros(num_data_points)
    omegas = numpy.zeros((num_data_points, 3))
    quaternions = numpy.zeros((num_data_points, 4))

    # Find the elapsed seconds between initial time and reference ephemeris
    ephemeris_time = spiceypy.str2et(start_seg)

    # Convert the initial time to number of spacecraft clock ticks
    start_ticks = numpy.zeros(1)
    start_ticks[0] = spiceypy.sce2c(spacecraft_id, ephemeris_time)

    # Process data for each timestep
    for i in range(num_data_points):
        # Process the attitude
        quat = RigidBodyKinematics.MRP2EP(mrp_array[i, -3:])  # Grab the last 3 elements in case the first column is time
        quat[1:4] = -quat[1:4]  # Convert to JPL-style quaternions
        quaternions[i,:] = quat

        # Process the angular velocity
        omegas[i,:] = av_array[i, -3:]  # Grab the last 3 elements in case the first column is time

        # Process time
        current_time = ephemeris_time + time[i] * macros.NANO2SEC  # Compute the current time in elapsed seconds from ephemeris
        current_ticks = spiceypy.sce2c(spacecraft_id, current_time)  # Convert from ephemeris seconds to spacecraft clock ticks
        times[i] = current_ticks

    # Get time into usable format
    encoded_start_time = times[0] - 1.0e-3  # Pad the beginning for roundoff
    encoded_end_time = times[num_data_points - 1] + 1.0e-3  # Pad the end for roundoff

    # Save the date into a CK file
    spiceypy.ckw03(file_handle,
                   encoded_start_time,
                   encoded_end_time,
                   spacecraft_id,
                   reference_frame,
                   True,
                   "InertialData",
                   num_data_points,
                   times,
                   quaternions,
                   omegas,
                   1,
                   start_ticks)

    # Close the CK file
    spiceypy.ckcls(file_handle)


def ckRead(time, spacecraft_id=-62, reference_frame="J2000"):
    """
    Purpose: Read information out of a CK Kernel for a single instance and returns a quaternion array
    and an angular velocity array

    .. warning::

        Assumes that SCLK and CK kernels are already loaded using furnsh because pyswice gets mad when loading the same files over and over again.

    :param time: Should be in UTC Gregorian, and passed in as a string, ex: 'FEB 01,2021  14:00:55.9999 (UTC)'
    :param spacecraft_id: Spacecraft ID -- Default: -62
    :param reference_frame: is a character string which specifies the, reference frame of the segment. Reference Frame, ex: "J2000"
    :return: None
    """
    # Find the elapsed seconds between initial time and reference ephemeris
    ephemeris_time = spiceypy.str2et(time)

    # Convert initial time to spacecraft clock tick
    tick = spiceypy.sce2c(spacecraft_id, ephemeris_time)

    # Get attitude and angular velocity for a specified spacecraft clock time
    [dcm, angular_velocity, sclk_time] = spiceypy.ckgpav(spacecraft_id, tick, 0, reference_frame)

    # Convert attitude to quaternions
    quat = RigidBodyKinematics.C2EP(dcm)
    quat[1:4] = - quat[1:4]  # Convert to JPL-style quaternions

    return ephemeris_time, quat, angular_velocity


def ckInitialize(ck_file_in):
    spiceypy.furnsh(ck_file_in)


def ckClose(ck_file_in):
    spiceypy.unload(ck_file_in)


def timeStringToGregorianUTCMsg(DateSpice, **kwargs):
    """convert a general time/date string to a gregoarian UTC msg object"""
    # set the data path
    if 'dataPath' in kwargs:
        dataPath = kwargs['dataPath']
        if not isinstance(dataPath, str):
            print('ERROR: dataPath must be a string argument')
            exit(1)
    else:
        dataPath = bskPath + '/supportData/EphemerisData/'  # default value

    # load spice kernel and convert the string into a UTC date/time string
    spiceypy.furnsh(dataPath + 'naif0012.tls')
    ephemeris_time = spiceypy.str2et(DateSpice)
    etEpoch = ephemeris_time
    ephemeris_time_epoch = spiceypy.et2utc(etEpoch, 'C', 6, 255)
    spiceypy.unload(dataPath + 'naif0012.tls')  # leap second file

    # convert UTC string to datetime object
    datetime_object = datetime.strptime(ephemeris_time_epoch, '%Y %b %d %H:%M:%S.%f')

    # populate the epochMsg with the gregorian UTC date/time information
    epochMsgStructure = EpochMsgPayload()
    epochMsgStructure.year = datetime_object.year
    epochMsgStructure.month = datetime_object.month
    epochMsgStructure.day = datetime_object.day
    epochMsgStructure.hours = datetime_object.hour
    epochMsgStructure.minutes = datetime_object.minute
    epochMsgStructure.seconds = datetime_object.second + datetime_object.microsecond / 1e6

    epochMsg = EpochMsg().write(epochMsgStructure)
    epochMsg.this.disown()

    return epochMsg
