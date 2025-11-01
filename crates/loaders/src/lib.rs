use drone::Drone;
use simulator::Simulator;
use loggers::FlightLog;
use flight_controller::controllers::res_controller::ResController;

mod db_loader;
mod file_loader;

pub trait DataAccessLayer: Send + Sync {
    // load a drone
    fn load_drone(&self, config_id: &str) -> Drone;

    // Load simulation
    fn load_simulation(&self, config_id: &str) -> Simulator;

    // Load replay
    fn load_replay(&self, sim_id: &str) -> FlightLog;

    // Get simulation ids
    fn get_replay_ids(&self) -> Vec<String>;

    // Get reservoir ids
    fn get_reservoir_controller_ids(&self) -> Vec<String>;

    // Load reservoir controller
    fn load_res_controller(&self, controller_id: &str) -> ResController;
}
