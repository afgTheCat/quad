use drone::Drone;
use res_controller::DroneRc;
use std::{fs, path::PathBuf};

use crate::LoaderTrait;

// const LOADER_PATH: &str = "/home/gabor/.local/share/quad/";
fn loader_path() -> PathBuf {
    PathBuf::from(std::env::var("HOME").unwrap()).join(".local/share/quad")
}

#[derive(Debug, Default)]
pub struct FileLoader {}

impl LoaderTrait for FileLoader {
    fn load_drone(&mut self, config_id: &str) -> Drone {
        let mut drone_path = loader_path();
        drone_path.push("drones/");
        fs::create_dir_all(&drone_path).unwrap();
        drone_path.push(format!("{config_id}.json"));
        let content = fs::read_to_string(drone_path).unwrap();
        serde_json::from_slice(content.as_bytes()).unwrap()
    }

    fn load_flight_log(&mut self, sim_id: &str) -> loggers::FlightLog {
        let mut replay = loader_path();
        replay.push("replays/");
        fs::create_dir_all(&replay).unwrap();
        replay.push(sim_id);
        let content = fs::read_to_string(replay).unwrap();
        serde_json::from_slice(content.as_bytes()).unwrap()
    }

    // just list the file names in the loader.
    fn get_replay_ids(&mut self) -> Vec<String> {
        let mut replays_dir = loader_path();
        replays_dir.push("replays/");
        fs::create_dir_all(&replays_dir).unwrap();
        fs::read_dir(replays_dir)
            .unwrap()
            .map(|res| res.map(|e| e.path()))
            .filter_map(|res| res.ok().map(|t| t.to_str().unwrap().to_owned()))
            .collect::<Vec<_>>()
    }

    fn get_reservoir_controller_ids(&mut self) -> Vec<String> {
        let mut reservoir_dir = loader_path();
        reservoir_dir.push("reservoirs/");
        fs::create_dir_all(&reservoir_dir).unwrap();
        fs::read_dir(reservoir_dir)
            .unwrap()
            .map(|res| res.map(|e| e.path()))
            .filter_map(|res| res.ok().map(|t| t.to_str().unwrap().to_owned()))
            .collect::<Vec<_>>()
    }

    fn load_res_controller(&mut self, controller_id: &str) -> DroneRc {
        let mut reservoir_dir = loader_path();
        reservoir_dir.push("reservoirs/");
        fs::create_dir_all(&reservoir_dir).unwrap();
        reservoir_dir.push(format!("{controller_id}"));
        let content = fs::read_to_string(reservoir_dir).unwrap();
        serde_json::from_slice(content.as_bytes()).unwrap()
    }

    fn insert_reservoir(&mut self, controller_id: &str, controller: DroneRc) {
        let mut reservoir_dir = loader_path();
        reservoir_dir.push("reservoirs/");
        fs::create_dir_all(&reservoir_dir).unwrap();
        reservoir_dir.push(format!("{controller_id}"));
        let serialized = serde_json::to_string(&controller).unwrap();
        fs::write(reservoir_dir, serialized).unwrap();
    }
}

#[cfg(test)]
mod test {
    use crate::file_loader::loader_path;
    use drone::default_drone::default_7in_4s_drone;
    use std::fs;

    #[test]
    fn save_default_config_to_file() {
        let default_drone = default_7in_4s_drone();
        let serialized = serde_json::to_string(&default_drone).unwrap();
        let mut drone_path = loader_path();
        drone_path.push("drones");
        fs::create_dir_all(&drone_path).unwrap();
        drone_path.push("7in_4s_drone.json");
        fs::write(drone_path, serialized).unwrap();
    }
}
