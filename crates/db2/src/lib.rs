pub mod db_loader;
pub mod file_loader;

use db::simulation::DBFlightLog;
use simulator::{Drone, Simulator};

pub trait LoaderTrait: Send + Sync {
    // load a drone
    fn load_drone(&self, config_id: i64) -> Drone;

    // Load simulation
    fn load_simulation(&self, config_id: i64) -> Simulator;

    // Load replay
    fn load_replay(&self, sim_id: &str) -> Vec<DBFlightLog>;

    // Get simulation ids
    fn get_simulation_ids(&self) -> Vec<String>;

    // Get reservoir ids
    fn get_reservoir_ids(&self) -> Vec<String>;
}
