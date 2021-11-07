from __future__ import print_function
import subprocess

import pytest

from pycrazyswarm import Crazyswarm


HAVE_ROS = subprocess.call("type roslaunch", shell=True) == 0


def pytest_runtest_setup(item):
    markers = [mark.name for mark in item.iter_markers()]
    if "ros" in markers and not HAVE_ROS:
        pytest.skip("ROS is not installed.")


def pytest_addoption(parser):
    parser.addoption("--simros", action="store_true", help="use ROS simulator")


@pytest.fixture
def crazyswarm_ctor(pytestconfig):
    if pytestconfig.getoption("simros"):
        # ROS simulator - start CSW server first
        assert HAVE_ROS, "cannot run --simros tests without ROS installed."
        # TODO Once we drop Python 2, this by-reference hack should be replaced
        # with a scalar variable and a `nonlocal` statement inside the closure.
        ros_process = [None]
        print("Defining ctor")
        def ctor(**kwargs):
            assert "crazyflies_yaml" in kwargs
            # TODO: It might be cleaner to use some kind of filesystem mocking,
            # but does it matter?
            print("Writing YAML")
            with open("../launch/crazyflies.yaml", "w") as f:
                f.write(kwargs["crazyflies_yaml"])
            print("Spawning ROS")
            ros_process[0] = subprocess.Popen(
                ["roslaunch", "crazyswarm", "hover_swarm.launch", "--sim"],
                shell=True
            )
            print("Spawned ROS")
            return Crazyswarm(**kwargs)
        print("Yielding ctor")
        yield ctor
        print("Done with yield")
        # Control will NOT return here immediately after the test calls
        # `ctor()`. Instead, pytest remembers that the fixture is in `yield`
        # state and returns control _after the test finishes_. For details, see
        # https://docs.pytest.org/en/latest/how-to/fixtures.html#yield-fixtures-recommended
        ros_process[0].terminate()
        print("Terminated")
    else:
        # Pycrazyswarm simulator
        def ctor(**kwargs):
            args = "--sim --vis null "
            if "args" in kwargs:
                args += kwargs["args"]
                kwargs = kwargs.copy()
                del kwargs["args"]
            return Crazyswarm(args=args, **kwargs)
        # The only reason we use `yield` here is for type consistency with the
        # ROS config -- see comment in `if` case.
        yield ctor