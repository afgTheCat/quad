pub mod db_loader;
pub mod file_loader;
pub mod file_logger;

use db::simulation::DBFlightLog;
use flight_controller::controllers::res_controller::ResController;
use simulator::{Drone, Simulator};

pub trait DataAccessLayer: Send + Sync {
    // load a drone
    fn load_drone(&self, config_id: &str) -> Drone;

    // Load simulation
    fn load_simulation(&self, config_id: &str) -> Simulator;

    // Load replay
    fn load_replay(&self, sim_id: &str) -> Vec<DBFlightLog>;

    // Get simulation ids
    fn get_replay_ids(&self) -> Vec<String>;

    // Get reservoir ids
    fn get_reservoir_controller_ids(&self) -> Vec<String>;

    // Load reservoir controller
    fn load_res_controller(&self, controller_id: &str) -> ResController;
}
