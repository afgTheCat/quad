use crate::Drone;
use db::{simulation::DBNewFlightLog, AscentDb};
use flight_controller::{BatteryUpdate, Channels, GyroUpdate, MotorInput};
use rerun::{external::glam::Vec3, Points3D, RecordingStream};
use std::{sync::Arc, time::Duration};

pub trait Logger: Sync + Send {
    // process the current state of the simulation
    fn process_state(&mut self, time: Duration, drone: &Drone, channels: Channels, fc_called: bool);
}

struct DBCurrentLog {
    start_seconds: f64,
    current_input: MotorInput,
    current_battery_update: BatteryUpdate,
    current_gyro: GyroUpdate,
    current_channels: Channels,
}

impl DBCurrentLog {
    fn to_db_log(self, simulation_id: String, end_seconds: f64) -> DBNewFlightLog {
        DBNewFlightLog {
            simulation_id,
            start_seconds: self.start_seconds,
            end_seconds,
            motor_input_1: self.current_input.input[0],
            motor_input_2: self.current_input.input[1],
            motor_input_3: self.current_input.input[2],
            motor_input_4: self.current_input.input[3],
            battery_voltage_sag: self.current_battery_update.bat_voltage_sag,
            battery_voltage: self.current_battery_update.bat_voltage,
            amperage: self.current_battery_update.amperage,
            mah_drawn: self.current_battery_update.m_ah_drawn,
            cell_count: self.current_battery_update.cell_count as i64,
            rot_quat_x: self.current_gyro.rotation[0],
            rot_quat_y: self.current_gyro.rotation[1],
            rot_quat_z: self.current_gyro.rotation[2],
            rot_quat_w: self.current_gyro.rotation[3],
            linear_acceleration_x: self.current_gyro.linear_acc[0],
            linear_acceleration_y: self.current_gyro.linear_acc[1],
            linear_acceleration_z: self.current_gyro.linear_acc[2],
            angular_velocity_x: self.current_gyro.angular_velocity[0],
            angular_velocity_y: self.current_gyro.angular_velocity[1],
            angular_velocity_z: self.current_gyro.angular_velocity[2],
            throttle: self.current_channels.throttle,
            roll: self.current_channels.roll,
            pitch: self.current_channels.pitch,
            yaw: self.current_channels.yaw,
        }
    }
}

pub struct DBLogger {
    db_current_log: DBCurrentLog,
    simulation_id: String,
    data: Vec<DBNewFlightLog>,
    db: Arc<AscentDb>,
}

impl Logger for DBLogger {
    fn process_state(
        &mut self,
        time: Duration,
        drone: &Drone,
        channels: Channels,
        fc_called: bool,
    ) {
        if !fc_called {
            return;
        }
        let current_step = std::mem::replace(
            &mut self.db_current_log,
            DBCurrentLog {
                start_seconds: time.as_secs_f64(),
                current_input: drone.motor_input(),
                current_battery_update: drone.battery_update(),
                current_gyro: drone.current_frame.gyro_state.gyro_update(),
                current_channels: channels,
            },
        );
        self.data
            .push(current_step.to_db_log(self.simulation_id.to_owned(), time.as_secs_f64()));
    }
}

impl Drop for DBLogger {
    fn drop(&mut self) {
        self.db.write_flight_logs(&self.simulation_id, &self.data);
    }
}

impl DBLogger {
    pub fn new(
        simulation_id: String,
        current_input: MotorInput,
        current_battery_update: BatteryUpdate,
        current_gyro: GyroUpdate,
        current_channels: Channels,
        db: Arc<AscentDb>,
    ) -> Self {
        let db_current_log = DBCurrentLog {
            start_seconds: 0.,
            current_input,
            current_battery_update,
            current_gyro,
            current_channels,
        };
        Self {
            simulation_id,
            db_current_log,
            data: vec![],
            db,
        }
    }
}

pub struct RerunLogger {
    counter: usize,
    rec: RecordingStream,
    current_time_step: Duration,
}

impl Logger for RerunLogger {
    fn process_state(&mut self, time: Duration, drone: &Drone, _: Channels, fc_called: bool) {
        if !fc_called {
            return;
        }
        if self.counter == 0 {
            self.rec
                .set_time_nanos("stable_time", time.as_nanos() as i64);
            let position = drone.position();

            let point = Vec3::new(position.x as f32, position.z as f32, position.y as f32);
            let points = Points3D::new(vec![point]);

            self.rec
                .log("drone/pos", &points.with_radii([0.8]))
                .unwrap();
            self.counter = 1000;
        } else {
            self.counter -= 1;
        }
    }
}

impl Drop for RerunLogger {
    fn drop(&mut self) {
        self.rec.flush_blocking();
    }
}

impl RerunLogger {
    pub fn new(simulation_id: String) -> Self {
        let current_time_step = Duration::new(0, 0);
        let rec = rerun::RecordingStreamBuilder::new(simulation_id)
            .spawn()
            .unwrap();
        rec.set_time_seconds("stable_time", 0f64);

        Self {
            counter: 1000, // just because we have a bunch of data
            rec,
            current_time_step,
        }
    }
}
