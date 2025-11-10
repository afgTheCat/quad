// NOTE: only loads the default drone config. This is for debugging and stuff

use drone::default_drone::default_7in_4s_drone;
use simulator::Simulator;

use crate::DataAccessLayer;

#[derive(Debug, Default)]
pub struct DefaultLoader {}

impl DataAccessLayer for DefaultLoader {
    fn load_drone(&mut self, config_id: &str) -> drone::Drone {
        default_7in_4s_drone()
    }

    fn load_simulation(&mut self, config_id: &str) -> simulator::Simulator {
        let drone = self.load_drone(config_id);
        Simulator::default_from_drone(drone)
    }

    fn load_replay(&mut self, sim_id: &str) -> loggers::FlightLog {
        unreachable!()
    }

    fn get_replay_ids(&mut self) -> Vec<String> {
        vec![]
    }

    fn get_reservoir_controller_ids(&mut self) -> Vec<String> {
        vec![]
    }

    fn load_res_controller(
        &mut self,
        controller_id: &str,
    ) -> flight_controller::controllers::res_controller::ResController {
        unreachable!()
    }

    fn insert_reservoir(&mut self, res: db_common::NewDBRcModel) {
        unreachable!()
    }
}
