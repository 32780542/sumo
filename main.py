"""Compatibility entry point for the SUMO platoon simulation.

The implementation is split under the sim package. This file re-exports the
public functions used by old scripts/tests and keeps `python main.py` working.
"""

from sim.config import *
from sim.utils import *
from sim.outputs import *
from sim.scenario import *
from sim.sensing import *
from sim.control import *
from sim.simulation import main, resolve_sumo_binary, run


if __name__ == "__main__":
    main()
