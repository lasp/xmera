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


import numpy
from Basilisk.architecture import messaging

#
#   FSW Setup Utilities for RW
#
rw_list = []


def create(
        gs_hat_B,
        Js,
        u_max = numpy.NaN,
        torque_sat_speed_limit=4000 * 2 * numpy.pi / 60
    ):
    """
    Create a FSW RW object

    This function is called to setup a FSW RW device in python, and adds it to the of RW
    devices in rw_list[].  This list is accessible from the parent python script that
    imported this rw library script, and thus any particular value can be over-ridden
    by the user.
    """
    global rw_list

    # create the blank RW object
    RW = messaging.RWConfigElementMsgPayload()

    norm = numpy.linalg.norm(gs_hat_B)
    if norm > 1e-10:
        gs_hat_B = gs_hat_B / norm
    else:
        print('Error: RW gsHat input must be non-zero 3x1 vector')
        exit(1)

    RW.gsHat_B = gs_hat_B
    RW.uMax = u_max
    RW.Js = Js
    RW.torqueSatSpeedLimit = torque_sat_speed_limit

    # add RW to the list of RW devices
    rw_list.append(RW)

    return


def write_config_message():
    """
    Write FSW RW array msg

    This function should be called after all devices are created with create()
    It creates the C-class container for the array of RW devices, and attaches
    this container to the spacecraft object

    """
    global rw_list

    gs_matrix_b = []
    js_list = []
    u_max_list = []
    torque_sat_speed_limit_list = []

    for rw in rw_list:
        gs_matrix_b.extend(rw.gsHat_B)
        js_list.extend([rw.Js])
        u_max_list.extend([rw.uMax])
        torque_sat_speed_limit_list.extend([rw.torqueSatSpeedLimit])

    rw_config_params = messaging.RWArrayConfigMsgPayload()
    rw_config_params.GsMatrix_B = gs_matrix_b
    rw_config_params.JsList = js_list
    rw_config_params.uMax = u_max_list
    rw_config_params.numRW = len(rw_list)
    rw_config_params.torqueSatSpeedLimit = torque_sat_speed_limit_list
    rw_config_msg = messaging.RWArrayConfigMsg().write(rw_config_params)
    rw_config_msg.this.disown()

    return rw_config_msg


def clear_setup():
    global rw_list

    rw_list = []

    return


def get_num_of_devices():
    return len(rw_list)
