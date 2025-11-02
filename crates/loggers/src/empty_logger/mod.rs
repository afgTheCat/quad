use crate::Logger;

#[derive(Default)]
pub struct EmptyLogger {}

impl Logger for EmptyLogger {
    fn log_time_stamp(&mut self, duration: std::time::Duration, snapshot: crate::SnapShot) {}
}
