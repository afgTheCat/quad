use crate::{Logger, SnapShot};
use rerun::RecordingStream;

pub struct RerunLogger {
    counter: usize,
    rec: RecordingStream,
}

impl Logger for RerunLogger {
    fn log_time_stamp(&mut self, snapshot: SnapShot) {
        self.rec
            .set_time_nanos("stable_time", snapshot.duration.as_nanos() as i64);
        // let position = snapshot.current_frame.drone_frame_state.position;
        // let point = Vec3::new(position.x as f32, position.z as f32, position.y as f32);
        // let points = Points3D::new(vec![point]);
        // self.rec
        //     .log("drone/pos", &points.with_radii([0.8]))
        //     .unwrap();
    }

    fn flush(&mut self) {
        self.rec.flush_blocking();
    }
}

impl Drop for RerunLogger {
    fn drop(&mut self) {
        self.flush();
    }
}

impl RerunLogger {
    pub fn new(simulation_id: String) -> Self {
        let rec = rerun::RecordingStreamBuilder::new(simulation_id)
            .spawn()
            .unwrap();
        rec.set_time_secs("stable_time", 0f64);

        Self {
            counter: 1000, // just because we have a bunch of data
            rec,
        }
    }
}
