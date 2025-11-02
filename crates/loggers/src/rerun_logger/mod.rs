use crate::{Logger, SnapShot};
use rerun::{Points3D, RecordingStream, external::glam::Vec3};
use std::time::Duration;

pub struct RerunLogger {
    counter: usize,
    rec: RecordingStream,
}

impl Logger for RerunLogger {
    fn log_time_stamp(&mut self, duration: Duration, snapshot: SnapShot) {
        self.rec
            .set_time_nanos("stable_time", duration.as_nanos() as i64);
        // let position = snapshot.current_frame.drone_frame_state.position;
        // let point = Vec3::new(position.x as f32, position.z as f32, position.y as f32);
        // let points = Points3D::new(vec![point]);
        // self.rec
        //     .log("drone/pos", &points.with_radii([0.8]))
        //     .unwrap();
    }
}

impl Drop for RerunLogger {
    fn drop(&mut self) {
        self.rec.flush_blocking();
    }
}
