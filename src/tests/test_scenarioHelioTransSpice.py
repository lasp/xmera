# SPDX-License-Identifier: ISC
# Copyright (c) 2022, Autonomous Vehicle System Lab, University of Colorado at Boulder
#

"""
Integrated tests

Purpose:  Integrated test of loading custom Spice files to specificy a spacecraft's translational motion. No simulation values are returned and tested.
"""

import inspect
import os
import sys

import pytest

# Get the current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/')

import scenarioHelioTransSpice

@pytest.mark.scenarioTest

def test_scenarioHelioTransSpice():

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    try:
        scenarioHelioTransSpice.run()

    except OSError as err:
        testFailCount = testFailCount + 1
        testMessages.append("OS error: {0}".format(err))

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED")
    else:
        print("Failed: testFailCount is " + str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

if __name__ == "__main__":
    test_scenarioHelioTransSpice(
    )
