// NOTE: only loads the default drone config. This is for debugging and stuff

use drone::default_drone::default_7in_4s_drone;
use res_controller::DroneRc;

use crate::LoaderTrait;

#[derive(Debug, Default)]
pub struct DefaultLoader {}

impl LoaderTrait for DefaultLoader {
    fn load_drone(&mut self, _config_id: &str) -> drone::Drone {
        default_7in_4s_drone()
    }

    fn load_replay(&mut self, _sim_id: &str) -> loggers::FlightLog {
        unreachable!()
    }

    fn get_replay_ids(&mut self) -> Vec<String> {
        vec![]
    }

    fn get_reservoir_controller_ids(&mut self) -> Vec<String> {
        vec![]
    }

    fn load_res_controller(&mut self, _controller_id: &str) -> DroneRc {
        unreachable!()
    }

    fn insert_reservoir(&mut self, _controller_id: &str, _controller: DroneRc) {
        todo!()
    }
}
