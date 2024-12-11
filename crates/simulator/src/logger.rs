use flight_controller::MotorInput;
use std::time::Duration;

pub struct SimLogger {
    current_time_step: Duration,
    current_input: MotorInput,
    pub simulation_id: Option<String>,
    pub data: Vec<[f64; 6]>, // start_seconds, end_seconds, motor_input_1, motor_input_2, motor_input_3, motor_input_4
}

impl SimLogger {
    pub fn new(current_input: MotorInput) -> Self {
        let current_time_step = Duration::new(0, 0);
        Self {
            current_time_step,
            current_input,
            simulation_id: None,
            data: vec![],
        }
    }

    pub fn init(&mut self, simulation_id: String) {
        self.simulation_id = Some(simulation_id)
    }

    pub fn insert_data(&mut self, next_time_step: Duration, next_input: MotorInput) {
        self.data.push([
            self.current_time_step.as_secs_f64(),
            next_time_step.as_secs_f64(),
            self.current_input.input[0],
            self.current_input.input[1],
            self.current_input.input[2],
            self.current_input.input[3],
        ]);

        // wrtr.write_record(csv_record).unwrap();
        self.current_time_step = next_time_step;
        self.current_input = next_input;
    }

    pub fn deinit(&mut self) {
        self.simulation_id = None
    }
}
