use crate::{
    bindings::sitl_generated::{FCInput, Sitl},
    FlightController, FlightControllerUpdate, MotorInput,
};
use once_cell::sync::Lazy;
use std::fmt::Debug;
use std::{
    collections::{hash_map::Entry, HashMap},
    ffi::CString,
    path::Path,
    sync::{Arc, Mutex},
    time::Duration,
};

pub struct BFController {
    instance_id: String,
    scheduler_delta: Duration,
}

#[derive(Clone)]
struct SitlWrapper {
    sitl: Arc<Sitl>,
}

impl Debug for SitlWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // let's ignore the sitl field for now
        f.debug_struct("SitlWrapper").finish()
    }
}

impl SitlWrapper {
    fn new() -> Self {
        let path = Path::new("/home/gabor/ascent/quad/crates/flight_controller/sitl/libsitl.so");
        let sitl = unsafe { Sitl::new(path).unwrap() };
        Self {
            sitl: Arc::new(sitl),
        }
    }

    // Do not leak any pointers
    pub fn access<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&Sitl) -> R,
    {
        f(&self.sitl)
    }
}

// Living on the edge, but actually is good, since we don'r really expose pointers and we are super
// careful not to introduce race conditions etc.
unsafe impl Send for SitlWrapper {}
unsafe impl Sync for SitlWrapper {}

#[derive(Default)]
struct SitlManager {
    instances: Arc<Mutex<HashMap<String, SitlWrapper>>>,
}

impl SitlManager {
    pub fn new() -> Self {
        Self {
            instances: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn register_new(&self, instance_id: String) {
        let mut guard = self.instances.lock().unwrap();
        let entry = guard.entry(instance_id);
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

    pub fn close(&self, instance_id: &str) {
        let mut guard = self.instances.lock().unwrap();
        let removed = guard.remove(instance_id);
        if removed.is_none() {
            todo!("Handle closing unloaded entry")
        }
    }

    pub fn access<F, R>(&self, instance_id: &str, f: F) -> R
    where
        F: FnOnce(&Sitl) -> R,
    {
        let library;
        {
            // prevent locking
            let guard = self.instances.lock().unwrap();
            library = guard.get(instance_id).cloned().unwrap();
        }
        library.access(f)
    }
}

static SITL_INSTANCE_MANAGER: Lazy<SitlManager> = Lazy::new(|| SitlManager::new());

impl BFController {
    pub fn new() -> Self {
        let instance_id = format!("default_id");
        // SITL_INSTANCE_MANAGER.register_new(instance_id.clone());
        let scheduler_delta = Duration::from_micros(50);
        Self {
            scheduler_delta,
            instance_id,
        }
    }
}

impl FlightController for BFController {
    fn init(&self) {
        SITL_INSTANCE_MANAGER.register_new(self.instance_id.clone());
        SITL_INSTANCE_MANAGER.access(&self.instance_id, |sitl| unsafe {
            let file_name =
                CString::new("/home/gabor/ascent/quad/eeprom.bin").expect("CString::new failed");
            sitl.ascent_init(file_name.as_ptr());
        })
    }

    fn deinit(&self) {
        SITL_INSTANCE_MANAGER.close(&self.instance_id);
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
        SITL_INSTANCE_MANAGER.access(&self.instance_id, |sitl| unsafe {
            sitl.update(delta_time_us, fc_input);
            let mut motors_signal = [0.; 4];
            sitl.get_motor_pwm_signals(motors_signal.as_mut_ptr());
            MotorInput {
                input: motors_signal.map(|x| x as f64),
            }
        })
    }

    fn scheduler_delta(&self) -> Duration {
        self.scheduler_delta
    }
}

impl Drop for BFController {
    fn drop(&mut self) {
        // close the loaded library
        SITL_INSTANCE_MANAGER.close(&self.instance_id);
    }
}
