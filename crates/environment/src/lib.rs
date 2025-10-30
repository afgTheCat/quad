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

use db::AscentDb;
use flight_controller::Channels;
use nalgebra::UnitQuaternion;
use pyo3::prelude::*;
use simulator::{
    SimulationObservation, Simulator,
    loader::{SimLoader, SimulationLoader},
};
use std::{f64, sync::Arc, time::Duration};

// all that seems useful for now
#[pyclass]
pub struct Observation {
    rotation: Vec<f64>,
    position: Vec<f64>,
    velocity: Vec<f64>,
    acceleration: Vec<f64>,
    angular_velocity: Vec<f64>,
    thrusts: Vec<f64>,
    bat_voltage: f64,
}

impl Observation {
    fn from_sim_observation(observation: SimulationObservation) -> Observation {
        let rotation = UnitQuaternion::from(observation.rotation);
        Observation {
            rotation: vec![
                rotation.coords.x,
                rotation.coords.y,
                rotation.coords.z,
                rotation.coords.w,
            ],
            position: vec![
                observation.position[0],
                observation.position[1],
                observation.position[2],
            ],
            velocity: vec![
                observation.linear_velocity[0],
                observation.linear_velocity[1],
                observation.linear_velocity[2],
            ],
            acceleration: vec![
                observation.acceleration[0],
                observation.acceleration[1],
                observation.acceleration[2],
            ],
            angular_velocity: vec![
                observation.angular_velocity[0],
                observation.angular_velocity[1],
                observation.angular_velocity[2],
            ],
            thrusts: vec![
                observation.thrusts[0],
                observation.thrusts[1],
                observation.thrusts[2],
                observation.thrusts[3],
            ],
            bat_voltage: observation.bat_voltage,
        }
    }
}

#[pymethods]
impl Observation {
    // TODO: I super hate how I have to define all the getters here. Probably there are better
    // solutions like returning a hashmap. This whole thing is temporary though
    #[getter]
    fn position(&self) -> PyResult<Vec<f64>> {
        Ok(self.position.clone())
    }

    pub fn flatten(&self) -> Vec<f64> {
        self.rotation
            .iter()
            .chain(self.position.iter())
            .chain(self.velocity.iter())
            .chain(self.acceleration.iter())
            .chain(self.angular_velocity.iter())
            .chain(self.thrusts.iter())
            .chain([self.bat_voltage].iter())
            .cloned()
            .collect()
    }
}

#[pyclass]
pub struct Environment {
    /// Config id
    config_id: String,
    /// Loads the simulation
    sim_loader: Box<dyn SimulationLoader>,
    /// The simulator
    simulator: Simulator,
}

#[pymethods]
impl Environment {
    // just for testing
    #[staticmethod]
    fn default() -> Self {
        log::info!("Default environment constructed");
        let db = AscentDb::new("/home/gabor/ascent/quad/data.sqlite");
        let sim_loader = SimLoader::new(Arc::new(db));
        let config_id = format!("");
        let mut simulator = sim_loader.load_simulation(&config_id);
        simulator.init();
        Self {
            config_id,
            sim_loader: Box::new(sim_loader),
            simulator,
        }
    }

    pub fn step(&mut self, action: Vec<f64>, delta_t: f64) -> Observation {
        let action = Channels {
            throttle: action[0],
            roll: action[1],
            pitch: action[2],
            yaw: action[3],
        };
        let observation = self
            .simulator
            .simulate_delta(Duration::from_secs_f64(delta_t), action);
        Observation::from_sim_observation(observation)
    }

    // TODO: when we have the machinary we should do a real reset
    pub fn reset(&mut self) -> Observation {
        let mut simulator = self.sim_loader.load_simulation(&self.config_id);
        simulator.init();

        // swap the simulators
        std::mem::swap(&mut self.simulator, &mut simulator);

        // explicitly drop the old simulator just to be sure
        drop(simulator);

        let observation = self.simulator.simulation_info();
        Observation::from_sim_observation(observation)
    }

    // TODO: what we can do is log things with the logger
    pub fn render(&self) {}

    pub fn close(&self) {
        // Just drop everything
    }
}

#[pymodule]
fn environment(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init(); // I guess we could supress this in the future
    m.add_class::<Observation>()?;
    m.add_class::<Environment>()?;
    Ok(())
}
