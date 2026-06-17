# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os
import shutil
import subprocess
import sys

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
print(path)

# remove the old report because we don't want stale data around, even without pytest-html
# for more see reportconf.py
if os.path.exists('tests/report/'):
    shutil.rmtree('tests/report/')


def pytest_addoption(parser):
    parser.addoption("--show_plots", action="store_true",
                     help="test(s) shall display plots")
    parser.addoption("--report", action="store_true",  # --report is easier, more controlled than --html=<pathToReport>
                         help="whether or not to gen a pytest-html report. The report is saved in ./tests/report")


@pytest.fixture(scope="module")
def show_plots(request):
    return request.config.getoption("--show_plots")


@pytest.fixture(autouse=True)
def _close_matplotlib_figures():
    """Close matplotlib figures left open by a test.

    Tests open figures with hard-coded numbers (``plt.figure(1, figsize=...)``).
    When a figure with that number survives into a later test, matplotlib ignores
    the size/dpi arguments and warns ("figure with num: N already exists"). Closing
    after each test keeps figure numbers from leaking across tests. Only acts when
    pyplot was actually imported, so non-plotting tests incur no cost.
    """
    yield
    plt = sys.modules.get("matplotlib.pyplot")
    if plt is not None:
        plt.close("all")


def pytest_make_parametrize_id(config, val, argname):
    return f"{argname}={val}"


# we don't want to reconfigure pytest per pytest-html unless we have it
# for more on this, see the reportconf.py file.
reqs = subprocess.check_output([sys.executable, '-m', 'pip', 'freeze'])
installed_packages = [r.decode().split('==')[0] for r in reqs.split()]

if ('--report' in sys.argv) and ('pytest-html' not in installed_packages):
    print('ERROR: you need to pip install pytest-html package to use the --report flag')
    quit()

if 'pytest-html' in installed_packages:
    exec(open(path + "/reportconf.py").read(), globals())
