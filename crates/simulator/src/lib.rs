use drone::{Drone, SimulationFrame};
use flight_controller::{
    controllers::bf_controller::BFController, Channels, FlightController, FlightControllerUpdate,
};
pub use flight_controller::{BatteryUpdate, GyroUpdate, MotorInput};
use loggers::{empty_logger::EmptyLogger, FlightLog, Logger, SnapShot};
use nalgebra::{Rotation3, Vector3, Vector4};
use rand::{rngs::StdRng, Rng, SeedableRng};
use std::{
    cell::RefCell,
    ops::Range,
    sync::{Arc, Mutex},
    time::Duration,
};

pub const MAX_EFFECT_SPEED: f64 = 18.0;
pub const AIR_RHO: f64 = 1.225;
pub const GRAVITY: f64 = 9.81;

thread_local! {
    static RNG: RefCell<StdRng> = RefCell::new(StdRng::seed_from_u64(0));
}

pub fn rng_gen_range(range: Range<f64>) -> f64 {
    RNG.with(|rng| rng.borrow_mut().gen_range(range))
}

#[derive(Debug, Default)]
pub struct SimulationObservation {
    pub simulation_time: Duration,
    pub rotation: Rotation3<f64>,
    pub position: Vector3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub thrusts: Vector4<f64>,
    pub rpms: Vector4<f64>,
    pub pwms: Vector4<f64>,
    pub bat_voltage: f64,
    pub bat_voltage_sag: f64,
}

// The simulator simulates the complete drone with a flight controller and all the neccessary aux
// information.
pub struct Simulator {
    pub drone: Drone,
    pub time: Duration,
    pub time_accu: Duration, // the accumulated time between two steps + the correction from the
    pub dt: Duration,
    pub flight_controller: Arc<dyn FlightController>,
    pub fc_time_accu: Duration,
    pub logger: Arc<Mutex<dyn Logger>>, // needs to be mutable
}

impl Simulator {
    pub fn default_from_drone(drone: Drone) -> Self {
        Self {
            drone,
            flight_controller: Arc::new(BFController::default()),
            logger: Arc::new(Mutex::new(EmptyLogger::default())),
            time: Duration::default(),
            dt: Duration::from_nanos(5000),
            fc_time_accu: Duration::default(),
            time_accu: Duration::default(),
        }
    }

    // TODO: dont need this
    pub fn simulation_info(&self) -> SimulationObservation {
        let current_frame = &self.drone.current_frame;

        let rotors_state = &current_frame.rotors_state;
        let drone_state = &current_frame.drone_frame_state;
        let battery_state = &current_frame.battery_state;

        let thrusts = Vector4::from_row_slice(
            &rotors_state
                .iter()
                .map(|r| r.effective_thrust)
                .collect::<Vec<f64>>(),
        );
        let rpms =
            Vector4::from_row_slice(&rotors_state.iter().map(|r| r.rpm).collect::<Vec<f64>>());
        let pwms =
            Vector4::from_row_slice(&rotors_state.iter().map(|r| r.rpm).collect::<Vec<f64>>());

        SimulationObservation {
            simulation_time: self.time,
            rotation: drone_state.rotation,
            position: drone_state.position,
            linear_velocity: drone_state.linear_velocity,
            acceleration: drone_state.acceleration,
            angular_velocity: drone_state.angular_velocity,
            thrusts,
            rpms,
            pwms,
            bat_voltage: battery_state.bat_voltage,
            bat_voltage_sag: battery_state.bat_voltage_sag,
        }
    }

    /// Given a duration (typically 10ms between frames), runs the simulation until the time
    /// accumlator is less then the simulation's dt. It will also try to
    pub fn simulate_delta(&mut self, delta: Duration, channels: Channels) -> SimulationObservation {
        self.time_accu += delta;
        while self.time_accu > self.dt {
            self.fc_time_accu += self.dt;
            self.drone.update(self.dt.as_secs_f64());

            let call_fc = self.fc_time_accu > self.flight_controller.scheduler_delta();

            // update the flight controller
            if call_fc {
                let motor_input = self.flight_controller.update(
                    self.fc_time_accu.as_secs_f64(),
                    FlightControllerUpdate {
                        battery_update: self.drone.battery_update(),
                        gyro_update: self.drone.current_frame.gyro_state.gyro_update(),
                        channels,
                    },
                );
                self.drone.set_motor_pwms(motor_input);
                self.fc_time_accu -= self.flight_controller.scheduler_delta();

                let mut logger = self.logger.lock().unwrap();
                let snapshot = SnapShot {
                    duration: self.time,
                    motor_input: motor_input,
                    battery_update: self.drone.battery_update(),
                    gyro_update: self.drone.current_frame.gyro_state.gyro_update(),
                    channels,
                };
                logger.log_time_stamp(snapshot);
            }

            self.time_accu -= self.dt;
            self.time += self.dt;
        }

        self.simulation_info()
    }

    pub fn init(&mut self) {
        self.flight_controller.init();
    }
}

pub struct Replayer {
    pub drone: Drone,
    pub time: Duration, // TODO: remove this
    pub time_accu: Duration,
    pub dt: Duration,
    // we assume that therer are not gaps in the input and the range of the input is always larger
    // than dt, since the simulation generarally runs at a higher frequency. Maybe in the future we
    // can eliviate these issues
    pub time_steps: FlightLog,
    pub replay_index: usize,
}

impl Replayer {
    fn get_motor_input(&mut self) -> Option<MotorInput> {
        if self.replay_index < self.time_steps.steps.len() {
            self.time += self.dt;
            let SnapShot {
                duration,
                motor_input,
                ..
            } = self.time_steps.steps[self.replay_index];
            if self.time.as_secs_f64() >= duration.as_secs_f64() {
                self.replay_index += 1;
            }
            Some(MotorInput {
                input: [
                    motor_input[0],
                    motor_input[1],
                    motor_input[2],
                    motor_input[3],
                ],
            })
        } else {
            None
        }
    }

    pub fn simulation_info(&self) -> SimulationObservation {
        let current_frame = &self.drone.current_frame;

        let rotors_state = &current_frame.rotors_state;
        let drone_state = &current_frame.drone_frame_state;
        let battery_state = &current_frame.battery_state;

        let thrusts = Vector4::from_row_slice(
            &rotors_state
                .iter()
                .map(|r| r.effective_thrust)
                .collect::<Vec<f64>>(),
        );
        let rpms =
            Vector4::from_row_slice(&rotors_state.iter().map(|r| r.rpm).collect::<Vec<f64>>());
        let pwms =
            Vector4::from_row_slice(&rotors_state.iter().map(|r| r.rpm).collect::<Vec<f64>>());

        SimulationObservation {
            simulation_time: self.time,
            rotation: drone_state.rotation,
            position: drone_state.position,
            linear_velocity: drone_state.linear_velocity,
            acceleration: drone_state.acceleration,
            angular_velocity: drone_state.angular_velocity,
            thrusts,
            rpms,
            pwms,
            bat_voltage: battery_state.bat_voltage,
            bat_voltage_sag: battery_state.bat_voltage_sag,
        }
    }

    pub fn replay_delta(&mut self, delta: Duration) -> SimulationObservation {
        self.time_accu += delta;
        while self.time_accu > self.dt {
            self.drone.update(self.dt.as_secs_f64());
            let motor_input = self.get_motor_input();
            if let Some(motor_input) = motor_input {
                self.drone.set_motor_pwms(motor_input);
            } else {
                self.drone.set_motor_pwms(MotorInput::default());
            }
            self.time_accu -= self.dt;
        }

        self.simulation_info()
    }

    pub fn reset(&mut self, initial_frame: SimulationFrame) {
        self.drone.reset(initial_frame);
        self.time = Duration::new(0, 0);
        self.time_accu = Duration::new(0, 0);
        self.replay_index = 0;
    }
}
