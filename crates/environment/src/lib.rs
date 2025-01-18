//! From gymnasium's documentation, this crate enables the implementation of the following:
//!     - :meth:`step` - Updates an environment with actions returning the next agent observation, the reward for taking that actions,
//!       if the environment has terminated or truncated due to the latest action and information from the environment about the step, i.e. metrics, debug info.
//!     - :meth:`reset` - Resets the environment to an initial state, required before calling step.
//!       Returns the first agent observation for an episode and information, i.e. metrics, debug info.
//!     - :meth:`render` - Renders the environments to help visualise what the agent see, examples modes are "human", "rgb_array", "ansi" for text.
//!     - :meth:`close` - Closes the environment, important when external software is used, i.e. pygame for rendering, databases
//!
//!     Environments have additional attributes for users to understand the implementation
//!
//!     - :attr:`action_space` - The Space object corresponding to valid actions, all valid actions should be contained within the space.
//!     - :attr:`observation_space` - The Space object corresponding to valid observations, all valid observations should be contained within the space.
//!     - :attr:`spec` - An environment spec that contains the information used to initialize the environment from :meth:`gymnasium.make`
//!     - :attr:`metadata` - The metadata of the environment, e.g. `{"render_modes": ["rgb_array", "human"], "render_fps": 30}`. For Jax or Torch, this can be indicated to users with `"jax"=True` or `"torch"=True`.
//!     - :attr:`np_random` - The random number generator for the environment. This is automatically assigned during
//!       ``super().reset(seed=seed)`` and when assessing :attr:`np_random`.

use flight_controller::Channels;
use nalgebra::Vector3;
use pyo3::pyclass;
use simulator::{SimulationObservation, Simulator};
use std::{collections::HashMap, f64, time::Duration};

// TODO: we can implement a bunch more things on this
struct Target {
    position: Vector3<f64>,
    accept_distance: f64,
}

impl Target {
    fn new(position: Vector3<f64>, accept_distance: f64) -> Self {
        Self {
            position,
            accept_distance,
        }
    }

    fn distance_to_point(&self, drone_pos: Vector3<f64>) -> f64 {
        self.position.metric_distance(&drone_pos)
    }

    // Indicates wheter a target has been hit or not
    fn hit(&self, drone_pos: Vector3<f64>) -> bool {
        if self.distance_to_point(drone_pos) < self.accept_distance {
            true
        } else {
            false
        }
    }
}

#[pyclass]
pub struct Environment {
    /// Controls how the step function will act
    delta_t: Duration,
    /// The simulator
    simulator: Simulator,
    /// The target
    target: Target,
    /// Maximum time that we allow per simulation
    max_ep_len: Duration,
}

impl Environment {
    fn new() {}

    // Step method
    pub fn step(
        &mut self,
        action: Channels,
    ) -> (
        SimulationObservation,
        f64,
        bool,
        bool,
        HashMap<String, String>,
    ) {
        let observation = self.simulator.simulate_delta(self.delta_t, action);
        let terminated = self.target.hit(observation.position);
        let reward = 1. / self.target.distance_to_point(observation.position);
        let trauncuated = self.max_ep_len < observation.simulation_time;
        (
            observation,
            reward,
            terminated,
            trauncuated,
            HashMap::default(),
        )
    }

    // TODO: when we have the machinary we should do a real reset
    pub fn reset(&mut self) {
        todo!()
    }

    pub fn render(&self) {
        todo!()
    }

    pub fn close(&self) {
        todo!()
    }
}
