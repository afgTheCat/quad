pub mod db_loader;
pub mod default_laoder;
pub mod file_loader;

use drone::Drone;
// use flight_controller::controllers::res_controller::ResController;
use loggers::FlightLog;
use res_controller::DroneRc;

pub trait LoaderTrait: Send + Sync {
    // load a drone
    fn load_drone(&mut self, config_id: &str) -> Drone;

    // TODO: do we need this?
    // Load simulation
    // fn load_simulation(&mut self, config_id: &str) -> Simulator;

    // Load replay
    fn load_replay(&mut self, sim_id: &str) -> FlightLog;

    // Get simulation ids
    fn get_replay_ids(&mut self) -> Vec<String>;

    // Get reservoir ids
    fn get_reservoir_controller_ids(&mut self) -> Vec<String>;

    // Load reservoir controller
    fn load_res_controller(&mut self, controller_id: &str) -> DroneRc;

    // Insert a new reservoir
    fn insert_reservoir(&mut self, controller_id: &str, controller: DroneRc);
}
