use crate::{Logger, SnapShot};

#[derive(Default)]
pub struct EmptyLogger {}

impl Logger for EmptyLogger {
    fn log_time_stamp(&mut self, _snapshot: SnapShot) {}
    fn flush(&mut self) {}
}
