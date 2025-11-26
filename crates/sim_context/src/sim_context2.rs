use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use flight_controller::{
    controllers::{
        bf_controller::BFController, null_controller::NullController, res_controller::ResController,
    },
    FlightController,
};
use loaders::{
    db_loader::DBLoader, default_laoder::DefaultLoader, file_loader::FileLoader, LoaderTrait,
};
use loggers::{
    db_logger::DBLogger, empty_logger::EmptyLogger, file_logger::FileLogger,
    rerun_logger::RerunLogger, Logger as LoggerTrait,
};
use loggers::{FlightLog, Logger};
use simulator::{Replayer, Simulator};
use uuid::Uuid;

use crate::{ControllerType, LoaderType, LoggerType};

pub struct SimContext {
    // Logger
    pub flight_controller: Arc<dyn FlightController>,
    // Controller
    pub logger: Arc<Mutex<dyn Logger>>,
    // Loader
    pub loader: Arc<Mutex<dyn LoaderTrait>>,

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

impl std::fmt::Debug for SimContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // TODO: add implementation
        Ok(())
    }
}

impl Default for SimContext {
    fn default() -> Self {
        let mut sim_context = SimContext {
            logger: Arc::new(Mutex::new(EmptyLogger::default())),
            flight_controller: Arc::new(NullController::default()),
            loader: Arc::new(Mutex::new(DefaultLoader::default())),
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
        match loader_type {
            LoaderType::DB => self.loader = Arc::new(Mutex::new(DBLoader::default())),
            LoaderType::File => self.loader = Arc::new(Mutex::new(FileLoader::default())),
            LoaderType::DefaultLoader => {
                self.loader = Arc::new(Mutex::new(DefaultLoader::default()))
            }
        }
    }

    pub fn get_simulation_id(&mut self) -> String {
        if let Some(simulation_id) = self.simulation_id.take() {
            simulation_id.clone()
        } else {
            Uuid::new_v4().to_string()
        }
    }

    pub fn set_simulation_id(&mut self, simulation_id: String) {
        self.simulation_id = Some(simulation_id)
    }

    pub fn set_replay_id(&mut self, replay_id: String) {
        self.replay_id = Some(replay_id)
    }

    pub fn set_logger(&mut self, logger_type: LoggerType) {
        let logger: Arc<Mutex<dyn LoggerTrait>> = match logger_type {
            LoggerType::Db => {
                let simulation_id = self.get_simulation_id();
                Arc::new(Mutex::new(DBLogger::new(simulation_id)))
            }
            LoggerType::Rerun => {
                let simulation_id = self.get_simulation_id();
                Arc::new(Mutex::new(RerunLogger::new(simulation_id)))
            }
            LoggerType::Empty => Arc::new(Mutex::new(EmptyLogger::default())),
            LoggerType::File => {
                let simulation_id = self.get_simulation_id();
                Arc::new(Mutex::new(FileLogger::new(simulation_id.to_string())))
            }
        };
        self.logger = logger;
    }

    pub fn set_controller(&mut self, controller: ControllerType) {
        let flight_controller: Arc<dyn FlightController> = match controller {
            ControllerType::Betafligt => Arc::new(BFController::default()),
            ControllerType::Reservoir(res_id) => {
                let res_controller = self.loader.lock().unwrap().load_res_controller(&res_id);
                Arc::new(res_controller)
            }
            ControllerType::NullController => Arc::new(NullController::default()),
        };
        self.flight_controller = flight_controller;
    }

    pub fn load_simulator(&self, config_id: &str) -> Simulator {
        let drone = self.loader.lock().unwrap().load_drone(&config_id);
        Simulator {
            drone: drone.clone(),
            flight_controller: self.flight_controller.clone(),
            time_accu: Duration::default(),
            time: Duration::new(0, 0),
            dt: Duration::from_nanos(5000), // TODO: update this maybe?
            fc_time_accu: Duration::default(),
            logger: self.logger.clone(),
        }
    }

    pub fn try_load_simulator(&mut self) -> Option<Simulator> {
        let Some(config_id) = self.config_id.clone() else {
            return None;
        };
        Some(self.load_simulator(&config_id))
    }

    pub fn load_replay_ids(&mut self) {
        let replay_ids = self.loader.lock().unwrap().get_replay_ids();
        self.replay_ids = replay_ids
    }

    pub fn load_res_controllers_ids(&mut self) {
        let reservoir_controler_ids = self.loader.lock().unwrap().get_reservoir_controller_ids();
        self.reservoir_controller_ids = reservoir_controler_ids
    }

    pub fn refresh_cache(&mut self) {
        self.load_replay_ids();
        self.load_res_controllers_ids();
    }

    pub fn load_replay(&mut self, replay_id: &str) -> FlightLog {
        self.loader.lock().unwrap().load_replay(replay_id)
    }

    pub fn load_replayer(&mut self, config_id: &str, replay_id: &str) -> Replayer {
        let drone = self.loader.lock().unwrap().load_drone(config_id);
        let sim_logs = self.loader.lock().unwrap().load_replay(replay_id);
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

    pub fn insert_drone_rc(&mut self, controller_id: &str, controller: ResController) {
        self.loader
            .lock()
            .unwrap()
            .insert_reservoir(controller_id, controller);
    }

    pub fn load_drone_rc(&mut self, controller_id: &str) -> ResController {
        self.loader
            .lock()
            .unwrap()
            .load_res_controller(controller_id)
    }

    pub fn insert_logs(&mut self, fl: FlightLog) {
        let mut logger = self.logger.lock().unwrap();
        logger.set_simulation_id(&fl.simulation_id);
        for s in fl.steps {
            logger.log_time_stamp(s);
        }
        logger.flush();
    }
}
