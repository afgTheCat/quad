use drone::Drone;
use simulator::loggers::Logger;

pub struct FileLogger {}

impl FileLogger {
    pub fn new() -> Self {
        Self {}
    }
}

impl Logger for FileLogger {
    fn log_time_stamp(
        &mut self,
        time: std::time::Duration,
        drone: &Drone,
        channels: flight_controller::Channels,
        fc_called: bool,
    ) {
    }
}
