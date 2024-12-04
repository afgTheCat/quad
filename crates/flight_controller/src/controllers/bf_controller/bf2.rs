use crate::{
    bindings::sitl_generated::FCInput, FlightController, FlightControllerUpdate, MotorInput,
};
use libloading::Library;
use std::{
    ffi::{c_char, CString},
    path::Path,
    sync::Arc,
    time::Duration,
};

type AscentInit = unsafe fn(file_name: *const c_char);
type AscentUpdate = unsafe fn(dt: u64, fc_input: FCInput) -> bool;
type AscentMotorPwmSignal = unsafe fn(signals: *mut f32);
type GetArmed = unsafe fn() -> bool;
type Scheduler = unsafe fn();
// pub type UpdateSerialWs = unsafe fn();

pub struct BFController2 {
    pub libsitl: Arc<Library>,
    scheduler_delta: Duration,
}

impl BFController2 {
    pub fn new() -> Self {
        let path = Path::new("/home/gabor/ascent/quad/crates/flight_controller/sitl/libsitl.so");
        let scheduler_delta = Duration::from_micros(50);
        let libsitl = unsafe { libloading::Library::new(path).unwrap() };
        Self {
            libsitl: Arc::new(libsitl),
            scheduler_delta,
        }
    }

    pub fn scheduler(&self) {
        unsafe {
            let sched: libloading::Symbol<Scheduler> = self.libsitl.get(b"scheduler").unwrap();
            sched();
        }
    }
}

impl FlightController for BFController2 {
    fn init(&self) {
        let file_name = CString::new("eeprom.bin").expect("CString::new failed");
        unsafe {
            let init: libloading::Symbol<AscentInit> = self.libsitl.get(b"ascent_init").unwrap();
            init(file_name.as_ptr());
        }
    }

    fn update(&self, delta_time_us: u64, fc_update: FlightControllerUpdate) -> MotorInput {
        let fc_input = FCInput {
            voltage: fc_update.battery_update.bat_voltage as f32,
            sagged_voltage: fc_update.battery_update.bat_voltage_sag as f32,
            amperage: fc_update.battery_update.amperage,
            mah_drawn: fc_update.battery_update.m_ah_drawn,
            cell_count: fc_update.battery_update.cell_count,
            attitude: fc_update.gyro_update.rotation.map(|x| x as f32),
            acceleration: fc_update.gyro_update.linear_acc.map(|x| x as f32),
            angular_velocity: fc_update.gyro_update.angular_velocity.map(|x| x as f32),
            rc_data: fc_update.channels.to_bf_channels(),
        };
        unsafe {
            let update: libloading::Symbol<AscentUpdate> = self.libsitl.get(b"update").unwrap();
            // let armed_fn: libloading::Symbol<GetArmed> = self.libsitl.get(b"get_armed").unwrap();
            // let armed = armed_fn();
            // println!("armed: {armed}");
            let getmotors_signals: libloading::Symbol<AscentMotorPwmSignal> =
                self.libsitl.get(b"get_motor_pwm_signals").unwrap();
            update(delta_time_us, fc_input);
            let mut motors_signal = [0.; 4];
            getmotors_signals(motors_signal.as_mut_ptr());
            MotorInput {
                input: motors_signal.map(|x| x as f64),
            }
        }
    }

    fn scheduler_delta(&self) -> Duration {
        self.scheduler_delta
    }
}
