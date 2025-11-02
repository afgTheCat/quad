use drone::Drone;
use flight_controller::controllers::res_controller::ResController;
use loggers::FlightLog;
use simulator::Simulator;

mod db_loader;
mod file_loader;

pub trait DataAccessLayer: Send + Sync {
    // load a drone
    fn load_drone(&mut self, config_id: &str) -> Drone;

    // TODO: do we need this?
    // Load simulation
    fn load_simulation(&mut self, config_id: &str) -> Simulator;

    // Load replay
    fn load_replay(&mut self, sim_id: &str) -> FlightLog;

    // Get simulation ids
    fn get_replay_ids(&mut self) -> Vec<String>;

    // Get reservoir ids
    fn get_reservoir_controller_ids(&mut self) -> Vec<String>;

    // Load reservoir controller
    fn load_res_controller(&mut self, controller_id: &str) -> ResController;
}
