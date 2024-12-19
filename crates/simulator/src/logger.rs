use db::FlightLog;
use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
use std::time::Duration;

pub struct SimLogger {
    current_time_step: Duration,
    current_input: MotorInput,
    current_battery_update: BatteryUpdate,
    current_gyro: GyroUpdate,
    current_channels: Channels,
    pub simulation_id: Option<String>,
    pub data: Vec<FlightLog>, // start_seconds, end_seconds, motor_input_1, motor_input_2, motor_input_3, motor_input_4
}

impl SimLogger {
    pub fn new(
        current_input: MotorInput,
        current_battery_update: BatteryUpdate,
        current_gyro: GyroUpdate,
        current_channels: Channels,
    ) -> Self {
        let current_time_step = Duration::new(0, 0);
        Self {
            current_time_step,
            current_input,
            current_battery_update,
            current_gyro,
            current_channels,
            simulation_id: None,
            data: vec![],
        }
    }

    pub fn init(&mut self, simulation_id: String) {
        self.simulation_id = Some(simulation_id)
    }

    pub fn insert_data(
        &mut self,
        next_time_step: Duration,
        next_input: MotorInput,
        next_battery_input: BatteryUpdate,
        next_gyro_udpate: GyroUpdate,
        next_channels: Channels,
    ) {
        self.data.push(FlightLog {
            range: self.current_time_step..next_time_step,
            motor_input: self.current_input.clone(),
            battery_update: self.current_battery_update.clone(),
            gyro_update: self.current_gyro.clone(),
            channels: self.current_channels.clone(),
        });

        self.current_time_step = next_time_step;
        self.current_input = next_input;
        self.current_battery_update = next_battery_input;
        self.current_gyro = next_gyro_udpate;
        self.current_channels = next_channels;
    }

    pub fn deinit(&mut self) {
        self.simulation_id = None
    }
}
