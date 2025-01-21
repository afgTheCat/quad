from typing import List
from numpy.typing import NDArray
from gymnasium import spaces, Env
import numpy as np

class Observation:
    rotation: NDArray[np.float64]
    position: NDArray[np.float64]
    velocity: NDArray[np.float64]
    acceleration: NDArray[np.float64]
    angular_velocity: NDArray[np.float64]
    thrusts: NDArray[np.float64]
    bat_voltage: np.float64

    def flatten(self) -> NDArray[np.float64]: ...

class Environment:
    @classmethod
    def default(cls) -> Environment: ...
    def step(self, action: NDArray[np.float64], delta_t: np.float64) -> Observation: ...

class GymEnv(Env):
    def __init__(self) -> None: ...
    def step(
        self, action: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], float, bool, bool, dict]: ...
