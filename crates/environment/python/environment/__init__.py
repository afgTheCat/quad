# TODO: nothig really is set in stone

import numpy as np
from gymnasium import spaces, Env
from numpy.typing import NDArray
from .environment import Environment
from typing import Any


class GymEnv(Env):
    _rust_env: Environment
    _target_position: NDArray[np.float64]
    _accept_distance: np.float64
    _delta_t: np.float64
    _total_t: np.float64
    _max_t: np.float64
    # action_space: spaces.Box

    def __init__(self) -> None:
        self._rust_env = Environment.default()
        self._target_position = np.array([10, 10, 10])
        self._accept_distance = np.float64(1)
        self._delta_t = np.float64(0.01)
        self._total_t = np.float64(0)
        self._max_t = np.float64(20)
        self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float64)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(10,), dtype=np.float64
        )

    def step(
        self, action: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], float, bool, bool, dict]:
        assert self.action_space.contains(action)
        assert self._total_t < self._max_t

        self._total_t += self._delta_t

        obs = self._rust_env.step(action, self._delta_t)
        distance_to_target = np.float64(
            np.linalg.norm(self._target_position - obs.position)
        )

        # goal reached
        reward = float(1 / distance_to_target)
        terminated = bool(self._accept_distance > distance_to_target)
        truncuated = bool(self._total_t > self._max_t)
        return (obs.flatten(), reward, terminated, truncuated, {})

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[NDArray[np.float64], dict]:
        obs = self._rust_env.reset()
        return (obs.flatten, {})

    def render(self) -> None:
        pass
