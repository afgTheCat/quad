use csv::Writer;
use flight_controller::MotorInput;
use std::{fs::File, time::Duration};

pub struct SimLogger {
    wrtr: Writer<File>,
    current_time_step: Duration,
    current_input: MotorInput,
}

impl SimLogger {
    pub fn new(path: &str, current_input: MotorInput) -> Self {
        let current_time_step = Duration::new(0, 0);
        let wrtr = Writer::from_path(path).unwrap();
        Self {
            wrtr,
            current_time_step,
            current_input,
        }
    }

    pub fn insert_data(&mut self, next_time_step: Duration, next_input: MotorInput) {
        let start_seconds = self.current_time_step.as_secs_f64().to_string();
        let end_seconds = next_time_step.as_secs_f64().to_string();
        let motor_inputs = self.current_input.input.map(|v| v.to_string());
        let csv_record = [&[start_seconds, end_seconds], &motor_inputs[..]].concat();

        self.wrtr.write_record(csv_record).unwrap();
        self.current_time_step = next_time_step;
        self.current_input = next_input;
    }

    pub fn write(&mut self) {
        self.wrtr.flush().unwrap();
    }

    pub fn clear(&mut self) {
        // TODO: we probably just want to clear the writer in the future
        self.wrtr.flush().unwrap();
    }
}
