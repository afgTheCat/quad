use drone::Drone;
use flight_controller::controllers::res_controller::ResController;
use simulator::Simulator;
use std::{fs, path::PathBuf};

use crate::DataAccessLayer;

const LOADER_PATH: &str = "/home/gabor/.local/share/quad/";

#[derive(Debug, Default)]
pub struct FileLoader {}

impl DataAccessLayer for FileLoader {
    fn load_drone(&mut self, config_id: &str) -> Drone {
        let mut drone_path = PathBuf::from(LOADER_PATH);
        drone_path.push("drones/");
        fs::create_dir_all(&drone_path).unwrap();
        drone_path.push(format!("{config_id}.json"));
        let content = fs::read_to_string(drone_path).unwrap();
        serde_json::from_slice(content.as_bytes()).unwrap()
    }

    fn load_simulation(&mut self, config_id: &str) -> simulator::Simulator {
        let drone = self.load_drone(config_id);
        Simulator::default_from_drone(drone)
    }

    fn load_replay(&mut self, sim_id: &str) -> loggers::FlightLog {
        todo!()
    }

    // fn load_replay(&self, sim_id: &str) -> Vec<db::simulation::DBFlightLog> {
    //     let mut replay = PathBuf::from(LOADER_PATH);
    //     replay.push("replays/");
    //     fs::create_dir_all(&replay).unwrap();
    //     replay.push(sim_id);
    //     let content = fs::read_to_string(replay).unwrap();
    //     serde_json::from_slice(content.as_bytes()).unwrap()
    // }

    // just list the file names in the loader.
    fn get_replay_ids(&mut self) -> Vec<String> {
        let mut simulation_dir = PathBuf::from(LOADER_PATH);
        // TODO: are these the replays
        simulation_dir.push("replays/");
        fs::create_dir_all(&simulation_dir).unwrap();
        fs::read_dir(simulation_dir)
            .unwrap()
            .map(|res| res.map(|e| e.path()))
            .filter_map(|res| res.ok().map(|t| t.to_str().unwrap().to_owned()))
            .collect::<Vec<_>>()
    }

    fn get_reservoir_controller_ids(&mut self) -> Vec<String> {
        let mut reservoir_dir = PathBuf::from(LOADER_PATH);
        reservoir_dir.push("reservoirs/");
        fs::create_dir_all(&reservoir_dir).unwrap();
        fs::read_dir(reservoir_dir)
            .unwrap()
            .map(|res| res.map(|e| e.path()))
            .filter_map(|res| res.ok().map(|t| t.to_str().unwrap().to_owned()))
            .collect::<Vec<_>>()
    }

    fn load_res_controller(&mut self, controller_id: &str) -> ResController {
        todo!()
    }

    fn insert_reservoir(&mut self, res: db_common::NewDBRcModel) {
        todo!()
    }
}

#[cfg(test)]
mod test {
    use crate::file_loader::LOADER_PATH;
    use drone::default_drone::default_7in_4s_drone;
    use std::{fs, path::PathBuf};

    #[test]
    fn save_default_config_to_file() {
        let default_drone = default_7in_4s_drone();
        let serialized = serde_json::to_string(&default_drone).unwrap();
        let mut drone_path = PathBuf::from(LOADER_PATH);
        drone_path.push("drones/7in_4s_drone.json");
        fs::write(drone_path, serialized).unwrap();
    }
}
