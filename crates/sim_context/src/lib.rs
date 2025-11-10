pub mod input_gen;

use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use db_common::{DBNewFlightLog, DBRcModel, NewDBRcModel};
use drone::Drone;
use flight_controller::{
    controllers::{
        bf_controller::BFController, null_controller::NullController, res_controller::ResController,
    },
    FlightController,
};
use loaders::{db_loader::DBLoader, DataAccessLayer};
use loaders::{default_laoder::DefaultLoader, file_loader::FileLoader};
use loggers::{
    db_logger::DBLogger, empty_logger::EmptyLogger, file_logger::FileLogger,
    rerun_logger::RerunLogger, FlightLog, Logger as LoggerTrait,
};
use simulator::Replayer;
use simulator::Simulator;
use uuid::Uuid;

#[derive(Default, Eq, PartialEq, Hash, Debug, Clone)]
pub enum LoggerType {
    File,
    Db,
    Rerun,
    #[default]
    Empty,
}

#[derive(Clone, Eq, PartialEq, Hash, Debug, Default)]
pub enum ControllerType {
    #[default]
    Betafligt, // no parameters
    Reservoir(String), // reservoir controller id
    NullController,    // no controller
}

#[derive(Debug)]
pub enum Loader {
    DBLoader(DBLoader),
    FileLoader(FileLoader),
    DefaultLoader(DefaultLoader),
}

impl Loader {
    pub fn load_drone(&mut self, config_id: &str) -> Drone {
        match self {
            Self::DBLoader(loader) => loader.load_drone(config_id),
            Self::FileLoader(loader) => loader.load_drone(config_id),
            Self::DefaultLoader(loader) => loader.load_drone(config_id),
        }
    }

    pub fn load_res_controller(&mut self, controller_id: &str) -> ResController {
        match self {
            Self::DBLoader(loader) => loader.load_res_controller(controller_id),
            Self::FileLoader(loader) => loader.load_res_controller(controller_id),
            Self::DefaultLoader(loader) => loader.load_res_controller(controller_id),
        }
    }

    pub fn load_replay(&mut self, replay_id: &str) -> FlightLog {
        match self {
            Self::DBLoader(loader) => loader.load_replay(replay_id),
            Self::FileLoader(loader) => loader.load_replay(replay_id),
            Self::DefaultLoader(loader) => loader.load_replay(replay_id),
        }
    }
}

impl Default for Loader {
    fn default() -> Self {
        Self::FileLoader(FileLoader::default())
    }
}

#[derive(Default, Clone, PartialEq)]
pub enum LoaderType {
    DB,
    File,
    #[default]
    DefaultLoader,
}

impl LoaderType {
    fn to_loader(&self) -> Loader {
        match self {
            Self::DB => Loader::DBLoader(DBLoader::default()),
            Self::File => Loader::FileLoader(FileLoader::default()),
            Self::DefaultLoader => Loader::DefaultLoader(DefaultLoader::default()),
        }
    }
}

#[derive(Debug)]
pub struct SimContext {
    // what kind of logger should be loaded
    pub logger_type: LoggerType,
    // what kind of flight controller we want to use
    pub controller: ControllerType,
    // loaders
    pub loader: Loader,
    // Replay ids
    pub replay_ids: Vec<String>,
    // Res controller ids
    pub reservoir_controller_ids: Vec<String>,
    // Selected replay id
    pub replay_id: Option<String>,
    // Config id
    pub config_id: Option<String>,
    // The id for the simulation that is about to be loaded
    pub simulation_id: Option<String>,
}

impl Default for SimContext {
    fn default() -> Self {
        let mut sim_context = SimContext {
            logger_type: Default::default(),
            controller: Default::default(),
            loader: Default::default(),
            replay_ids: Default::default(),
            reservoir_controller_ids: Default::default(),
            replay_id: Default::default(),
            config_id: Some(format!("7in_4s_drone")),
            simulation_id: Default::default(),
        };
        sim_context.refresh_cache();
        sim_context
    }
}

impl SimContext {
    pub fn set_loader(&mut self, loader_type: &LoaderType) {
        self.loader = loader_type.to_loader();
        self.load_replay_ids();
        self.load_res_controllers();
    }

    pub fn set_logger_type(&mut self, logger_type: LoggerType) {
        self.logger_type = logger_type;
    }

    pub fn set_controller(&mut self, controller: ControllerType) {
        self.controller = controller;
    }

    pub fn logger_type(&self) -> &LoggerType {
        &self.logger_type
    }

    pub fn contoller(&self) -> &ControllerType {
        &self.controller
    }

    pub fn load_simulator(&mut self, simulation_id: String, config_id: &str) -> Simulator {
        let drone = self.loader.load_drone(config_id);
        let flight_controller: Arc<dyn FlightController> = match &self.controller {
            ControllerType::Betafligt => Arc::new(BFController::default()),
            ControllerType::Reservoir(res_id) => Arc::new(self.loader.load_res_controller(&res_id)),
            ControllerType::NullController => Arc::new(NullController::default()),
        };
        let logger: Arc<Mutex<dyn LoggerTrait>> = match &self.logger_type {
            LoggerType::Db => Arc::new(Mutex::new(DBLogger::new(simulation_id.to_owned()))),
            LoggerType::Rerun => Arc::new(Mutex::new(RerunLogger::new(simulation_id.to_owned()))),
            LoggerType::Empty => Arc::new(Mutex::new(EmptyLogger::default())),
            LoggerType::File => Arc::new(Mutex::new(FileLogger::new(simulation_id.to_string()))),
        };
        Simulator {
            drone: drone.clone(),
            flight_controller: flight_controller.clone(),
            time_accu: Duration::default(),
            time: Duration::new(0, 0),
            dt: Duration::from_nanos(5000), // TODO: update this maybe?
            fc_time_accu: Duration::default(),
            logger,
        }
    }

    pub fn set_simulation_id(&mut self, simulation_id: String) {
        self.simulation_id = Some(simulation_id)
    }

    pub fn try_load_simulator(&mut self) -> Option<Simulator> {
        let Some(config_id) = self.config_id.clone() else {
            return None;
        };
        let simulation_id = if let Some(simulation_id) = self.simulation_id.take() {
            simulation_id.clone()
        } else {
            Uuid::new_v4().to_string()
        };
        Some(self.load_simulator(simulation_id, &config_id))
    }

    pub fn load_replayer(&mut self, config_id: &str, replay_id: &str) -> Replayer {
        let drone = self.loader.load_drone(config_id);
        let sim_logs = self.loader.load_replay(replay_id);
        Replayer {
            drone,
            time: Duration::new(0, 0),
            time_accu: Duration::new(0, 0),
            time_steps: sim_logs,
            replay_index: 0,
            dt: Duration::from_nanos(5000), // TODO: update this
        }
    }

    pub fn try_load_replay(&mut self) -> Option<Replayer> {
        if let (Some(config_id), Some(replay_id)) = (self.config_id.clone(), self.replay_id.clone())
        {
            Some(self.load_replayer(&config_id, &replay_id))
        } else {
            None
        }
    }

    pub fn load_replay_ids(&mut self) {
        let replay_ids = match &mut self.loader {
            Loader::DBLoader(db_loader) => db_loader.get_replay_ids(),
            Loader::FileLoader(file_loader) => file_loader.get_replay_ids(),
            Loader::DefaultLoader(loader) => loader.get_replay_ids(),
        };
        self.replay_ids = replay_ids;
    }

    pub fn load_res_controllers(&mut self) {
        let reservoir_controler_ids = match &mut self.loader {
            Loader::DBLoader(db_loader) => db_loader.get_reservoir_controller_ids(),
            Loader::FileLoader(file_loader) => file_loader.get_reservoir_controller_ids(),
            Loader::DefaultLoader(loader) => loader.get_reservoir_controller_ids(),
        };
        self.reservoir_controller_ids = reservoir_controler_ids
    }

    pub fn refresh_cache(&mut self) {
        self.load_replay_ids();
        self.load_res_controllers();
    }

    pub fn set_replay_id(&mut self, replay_id: String) {
        self.replay_id = Some(replay_id);
    }

    pub fn insert_reservoir(&mut self, res: NewDBRcModel) {
        todo!()
    }

    pub fn select_reservoir(&mut self, id: &str) -> DBRcModel {
        todo!()
    }

    pub fn insert_logs(&mut self, id: &str, flight_logs: Vec<DBNewFlightLog>) {
        todo!()
    }
}
