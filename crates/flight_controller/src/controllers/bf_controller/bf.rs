use crate::{
    bindings::sitl_generated::{FCInput, Sitl},
    FlightController, FlightControllerUpdate, MotorInput,
};
use std::{
    cell::UnsafeCell,
    ffi::CString,
    path::Path,
    sync::{Mutex, OnceLock},
    time::Duration,
};

pub struct BFController2 {
    scheduler_delta: Duration,
}

#[derive(Debug)]
struct SitlWrapper {
    inner: UnsafeCell<Sitl>,
}

impl SitlWrapper {
    fn new() -> Self {
        let path = Path::new("/home/gabor/ascent/quad/crates/flight_controller/sitl/libsitl.so");
        let sitl = unsafe { Sitl::new(path).unwrap() };
        Self {
            inner: UnsafeCell::new(sitl),
        }
    }

    // Do not leak any pointers
    pub fn access<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut Sitl) -> R,
    {
        unsafe { f(&mut *self.inner.get()) }
    }
}

// Living on the edge, but actually is good, since we don'r really expose pointers and we are super
// careful not to introduce race conditions etc.
unsafe impl Send for SitlWrapper {}
unsafe impl Sync for SitlWrapper {}

static SITL_INSTANCE: OnceLock<Mutex<SitlWrapper>> = OnceLock::new();

// should only be constructed once for now
impl BFController2 {
    pub fn new() -> Self {
        let sitl = SitlWrapper::new();
        SITL_INSTANCE.set(Mutex::new(sitl)).unwrap();
        let scheduler_delta = Duration::from_micros(50);
        Self { scheduler_delta }
    }
}

impl FlightController for BFController2 {
    fn init(&self) {
        let guard = SITL_INSTANCE.get().unwrap().lock().unwrap();
        guard.access(|inner| unsafe {
            let file_name = CString::new("eeprom.bin").expect("CString::new failed");
            inner.ascent_init(file_name.as_ptr());
        });
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
        let guard = SITL_INSTANCE.get().unwrap().lock().unwrap();
        let motor_input = guard.access(|inner| unsafe {
            inner.update(delta_time_us, fc_input);
            let mut motors_signal = [0.; 4];
            inner.get_motor_pwm_signals(motors_signal.as_mut_ptr());
            MotorInput {
                input: motors_signal.map(|x| x as f64),
            }
        });
        motor_input
    }

    fn scheduler_delta(&self) -> Duration {
        self.scheduler_delta
    }
}
