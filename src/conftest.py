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


def pytest_collectstart(collector):
    if isinstance(collector, pytest.Module):
        directory = str(collector.path.parent)
        if directory not in sys.path:
            sys.path.insert(0, directory)

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
    parser.addoption("--log-worker-tests", action="store",
                     default=None,
                     help="Directory to write per-xdist-worker test execution logs (one file per worker). "
                          "Each line is the test nodeid as it begins. Use to identify which test ran "
                          "just before a worker crash.")


@pytest.hookimpl(tryfirst=True)
def pytest_runtest_setup(item):
    log_dir = item.config.getoption("--log-worker-tests")
    if not log_dir:
        return
    worker = os.environ.get("PYTEST_XDIST_WORKER", "main")
    os.makedirs(log_dir, exist_ok=True)
    with open(os.path.join(log_dir, f"worker-{worker}.log"), "a") as f:
        f.write(item.nodeid + "\n")
        f.flush()


@pytest.fixture(scope="module")
def show_plots(request):
    return request.config.getoption("--show_plots")


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
