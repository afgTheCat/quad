use crate::DataAccessLayer;
use drone::Drone;
use flight_controller::controllers::res_controller::ResController;
use simulator::Simulator;
use std::{fs, path::PathBuf};

const LOADER_PATH: &str = "/home/gabor/.local/share/quad/";

#[derive(Debug, Clone, Default)]
pub struct FileLoader {}

impl DataAccessLayer for FileLoader {
    fn load_drone(&self, config_id: &str) -> Drone {
        let mut simulation = PathBuf::from(LOADER_PATH);
        simulation.push("drones/");
        fs::create_dir_all(&simulation).unwrap();
        simulation.push(format!("{config_id}.json"));
        let content = fs::read_to_string(simulation).unwrap();
        serde_json::from_slice(content.as_bytes()).unwrap()
    }

    fn load_simulation(&self, config_id: &str) -> simulator::Simulator {
        let drone = self.load_drone(config_id);
        Simulator::default_from_drone(drone)
    }

    fn load_replay(&self, sim_id: &str) -> Vec<db::simulation::DBFlightLog> {
        let mut replay = PathBuf::from(LOADER_PATH);
        replay.push("replays/");
        fs::create_dir_all(&replay).unwrap();
        replay.push(sim_id);
        let content = fs::read_to_string(replay).unwrap();
        serde_json::from_slice(content.as_bytes()).unwrap()
    }

    // just list the file names in the loader.
    fn get_replay_ids(&self) -> Vec<String> {
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

    fn get_reservoir_controller_ids(&self) -> Vec<String> {
        let mut reservoir_dir = PathBuf::from(LOADER_PATH);
        reservoir_dir.push("reservoirs/");
        fs::create_dir_all(&reservoir_dir).unwrap();
        fs::read_dir(reservoir_dir)
            .unwrap()
            .map(|res| res.map(|e| e.path()))
            .filter_map(|res| res.ok().map(|t| t.to_str().unwrap().to_owned()))
            .collect::<Vec<_>>()
    }

    fn load_res_controller(&self, controller_id: &str) -> ResController {
        todo!()
    }
}

#[cfg(test)]
mod test {
    use crate::file_loader::LOADER_PATH;
    use drone::default_drone::default_7in_4s_drone;
    use std::{fs, path::PathBuf};

    #[test]
    fn insert_drone() {
        let defualt_drone = default_7in_4s_drone();
        let drone_str = serde_json::to_string(&defualt_drone).unwrap();
        let mut path = PathBuf::from(LOADER_PATH);
        path.push("drones/");
        path.push(format!("7in_4s_drone.json"));
        fs::write(path, drone_str).unwrap();
    }
}
