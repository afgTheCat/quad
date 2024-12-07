use crate::{
    bindings::sitl_generated::{FCInput, Sitl},
    FlightController, FlightControllerUpdate, MotorInput,
};
use std::{
    cell::{OnceCell, UnsafeCell},
    collections::{hash_map::Entry, HashMap},
    ffi::CString,
    path::Path,
    sync::{Mutex, OnceLock},
    time::Duration,
};

pub struct BFController {
    id: String,
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

#[derive(Default)]
struct SitlManager {
    instances: HashMap<String, SitlWrapper>,
}

impl SitlManager {
    pub fn new() -> Self {
        Self {
            instances: HashMap::new(),
        }
    }

    pub fn register_new(&mut self, instance_id: String) {
        let entry = self.instances.entry(instance_id);
        match entry {
            Entry::Vacant(vacant_entry) => {
                let sitl_wrapper = SitlWrapper::new();
                vacant_entry.insert(sitl_wrapper);
            }
            Entry::Occupied(_) => {
                todo!("Hanle collisions");
            }
        }
    }

    pub fn close(&mut self, instance_id: String) {
        let removed = self.instances.remove(&instance_id);
        if removed.is_none() {
            todo!("Handle closing unloaded entry")
        }
    }

    pub fn access<F, R>(&self, instance_id: &str, f: F) -> R
    where
        F: FnOnce(&mut Sitl) -> R,
    {
        match self.instances.get(instance_id) {
            Some(entry) => entry.access(f),
            None => todo!("Handle accessing unloaded entry"),
        }
    }
}

unsafe impl Send for SitlManager {}
unsafe impl Sync for SitlManager {}

static SITL_INSTANCE: OnceLock<Mutex<SitlWrapper>> = OnceLock::new();
// This is not optimal since we have to be able to access multiple instances at the same time. Will
// see what the solution should be.
static SITL_INSTANCE_MANAGER: OnceLock<Mutex<SitlManager>> = OnceLock::new();

// should only be constructed once for now
impl BFController {
    pub fn new() -> Self {
        let id = format!("default_id");
        let sitl = SitlWrapper::new();
        SITL_INSTANCE.set(Mutex::new(sitl)).unwrap();
        let scheduler_delta = Duration::from_micros(50);
        Self {
            scheduler_delta,
            id,
        }
    }
}

impl FlightController for BFController {
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
