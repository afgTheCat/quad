use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use db::{simulation::DBFlightLog, AscentDb};
use db2::{db_loader::DBLoader, file_loader::FileLoader, DataAccessLayer};
use flight_controller::{
    controllers::{
        bf_controller::BFController, null_controller::NullController, res_controller::ResController,
    },
    Channels, FlightController,
};
use simulator::{
    loggers::{DBLogger, EmptyLogger, Logger as LoggerTrait, RerunLogger},
    BatteryUpdate, MotorInput, Replayer,
};
use simulator::{Drone, Simulator};
use uuid::Uuid;

#[derive(Default, Eq, PartialEq, Hash, Debug, Clone)]
pub enum LoggerType {
    #[default]
    File,
    Db,
    Rerun,
    Empty,
}

#[derive(Clone, Eq, PartialEq, Hash, Debug, Default)]
pub enum Controller {
    #[default]
    Betafligt, // no parameters
    Reservoir(String), // reservoir controller id
    NullController,    // no controller
}

#[derive(Clone, Debug)]
pub enum Loader {
    DBLoader(DBLoader),
    FileLoader(FileLoader),
}

impl Loader {
    fn load_drone(&self, config_id: i64) -> Drone {
        match self {
            Self::DBLoader(loader) => loader.load_drone(config_id),
            Self::FileLoader(loader) => loader.load_drone(config_id),
        }
    }

    fn load_res_controller(&self, controller_id: &str) -> ResController {
        match self {
            Self::DBLoader(loader) => loader.load_res_controller(controller_id),
            Self::FileLoader(loader) => loader.load_res_controller(controller_id),
        }
    }

    fn load_replay(&self, replay_id: &str) -> Vec<DBFlightLog> {
        todo!()
    }
}

impl Default for Loader {
    fn default() -> Self {
        Self::FileLoader(FileLoader::default())
    }
}

// #[derive(Default)]
pub struct SimContext {
    // what kind of logger should be loaded
    logger_type: LoggerType,
    // what kind of flight controller we want to use
    contoller: Controller,
    // loaders
    loader: Loader,
    // Replay ids
    pub replay_ids: Vec<String>,
    // Res controller ids
    pub reservoir_controller_ids: Vec<String>,
    // Selected replay id
    replay_id: Option<String>,
    // Config id
    config_id: Option<i64>,
}

impl Default for SimContext {
    fn default() -> Self {
        let mut sim_context = SimContext {
            logger_type: Default::default(),
            contoller: Default::default(),
            loader: Default::default(),
            replay_ids: Default::default(),
            reservoir_controller_ids: Default::default(),
            replay_id: Default::default(),
            config_id: Default::default(),
        };
        sim_context.refresh_cache();
        sim_context
    }
}

impl SimContext {
    pub fn set_loader(&mut self, loader: Loader) {
        self.loader = loader;
        self.load_replay_ids();
        self.load_res_controllers();
    }

    pub fn set_logger_type(&mut self, logger_type: LoggerType) {
        self.logger_type = logger_type;
    }

    pub fn set_contoller(&mut self, contoller: Controller) {
        self.contoller = contoller;
    }

    pub fn logger_type(&self) -> &LoggerType {
        &self.logger_type
    }

    pub fn contoller(&self) -> &Controller {
        &self.contoller
    }

    pub fn load_simulator(&self, config_id: i64) -> Simulator {
        let simulation_id = Uuid::new_v4().to_string();
        let drone = self.loader.load_drone(config_id);
        let flight_controller: Arc<dyn FlightController> = match &self.contoller {
            Controller::Betafligt => Arc::new(BFController::default()),
            Controller::Reservoir(res_id) => Arc::new(self.loader.load_res_controller(&res_id)),
            Controller::NullController => Arc::new(NullController::default()),
        };
        let battery_state = &drone.current_frame.battery_state;
        let logger: Arc<Mutex<dyn LoggerTrait>> = match &self.logger_type {
            LoggerType::Db => {
                let db = Arc::new(AscentDb::default());
                Arc::new(Mutex::new(DBLogger::new(
                    simulation_id.to_owned(),
                    MotorInput::default(),
                    BatteryUpdate {
                        bat_voltage_sag: battery_state.bat_voltage_sag,
                        bat_voltage: battery_state.bat_voltage,
                        amperage: battery_state.amperage,
                        m_ah_drawn: battery_state.m_ah_drawn,
                        cell_count: drone.battery_model.quad_bat_cell_count,
                    },
                    drone.current_frame.gyro_state.gyro_update(),
                    Channels::default(),
                    db.clone(),
                )))
            }
            LoggerType::Rerun => Arc::new(Mutex::new(RerunLogger::new(simulation_id.to_owned()))),
            LoggerType::Empty => Arc::new(Mutex::new(EmptyLogger::default())),
            // TODO: add file logger!
            LoggerType::File => todo!(),
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

    pub fn try_load_simulator(&self) -> Option<Simulator> {
        let Some(config_id) = self.config_id else {
            return None;
        };
        Some(self.load_simulator(config_id))
    }

    pub fn load_replayer(&self, config_id: i64, replay_id: &str) -> Replayer {
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

    pub fn try_load_replay(&self) -> Option<Replayer> {
        let (Some(config_id), Some(replay_id)) = (self.config_id, &self.replay_id) else {
            return None;
        };
        Some(self.load_replayer(config_id, replay_id))
    }

    pub fn load_replay_ids(&mut self) {
        let replay_ids = match &self.loader {
            Loader::DBLoader(db_loader) => db_loader.get_replay_ids(),
            Loader::FileLoader(file_loader) => file_loader.get_replay_ids(),
        };
        self.replay_ids = replay_ids;
    }

    pub fn load_res_controllers(&mut self) {
        let reservoir_controler_ids = match &self.loader {
            Loader::DBLoader(db_loader) => db_loader.get_reservoir_controller_ids(),
            Loader::FileLoader(file_loader) => file_loader.get_reservoir_controller_ids(),
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
}
