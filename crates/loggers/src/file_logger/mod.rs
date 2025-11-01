use crate::Logger;

pub struct FileLogger {}

impl Logger for FileLogger {
    fn log_time_stamp(&mut self, duration: std::time::Duration, snapshot: crate::SnapShot) {}
}
