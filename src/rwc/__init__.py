"""Rendezvous without coordinates: Dubins-car agents with a ternary windshield sensor."""

from .scenarios import initial_state, make_sim
from .sim import Sim

__all__ = ["Sim", "make_sim", "initial_state"]
__version__ = "1.0.0"
