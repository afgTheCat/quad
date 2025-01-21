use crate::{FlightController, FlightControllerUpdate, MotorInput};
use libloading::Library;
use once_cell::sync::Lazy;
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

type VBFInit = unsafe extern "C" fn(file_name: *const std::os::raw::c_char);
type VBFUpdate = unsafe extern "C" fn(micros_passed: u64);
type VBFArm = unsafe extern "C" fn();
type VBFDisArm = unsafe extern "C" fn();
type VBFUpdateSerialWs = unsafe extern "C" fn();
type VBFGetIsArmed = unsafe extern "C" fn() -> bool;
type VBFGetIsBeeping = unsafe extern "C" fn() -> bool;
type VBFGetArmingDisbleFlags = unsafe extern "C" fn() -> std::os::raw::c_int;
type VBFGetMicrosPassed = unsafe extern "C" fn() -> u64;
type VBFGetMotorSignals = unsafe extern "C" fn(*mut f32);
type VBFSetMotorRcData = unsafe extern "C" fn(*const f32);
type VBFSetGyroData = unsafe extern "C" fn(*const f32);
type VBFSetAttitude = unsafe extern "C" fn(*const f32);
type VBFSetAccelData = unsafe extern "C" fn(*const f32);
type VBFSetBattery = unsafe extern "C" fn(u8, f32, f32, f64, f64);
type VBFSetGpsData = unsafe extern "C" fn(i32, i32, i32, u16);

struct VirtualBF {
    lib: Library,
    vbf_init: Result<VBFInit, libloading::Error>,
    vbf_update: Result<VBFUpdate, libloading::Error>,
    vbf_arm: Result<VBFArm, libloading::Error>,
    vbf_disarm: Result<VBFDisArm, libloading::Error>,
    vbf_update_serial_ws: Result<VBFUpdateSerialWs, libloading::Error>,
    vbf_get_is_armed: Result<VBFGetIsArmed, libloading::Error>,
    vbf_get_is_beeping: Result<VBFGetIsBeeping, libloading::Error>,
    vbf_get_arming_disable_flags: Result<VBFGetArmingDisbleFlags, libloading::Error>,
    vbf_get_micros_passed: Result<VBFGetMicrosPassed, libloading::Error>,
    vbf_get_motor_signals: Result<VBFGetMotorSignals, libloading::Error>,
    vbf_set_rc_data: Result<VBFSetMotorRcData, libloading::Error>,
    vbf_set_gyro_data: Result<VBFSetGyroData, libloading::Error>,
    vbf_set_accel_data: Result<VBFSetAccelData, libloading::Error>,
    vbf_set_attitude: Result<VBFSetAttitude, libloading::Error>,
    vbf_set_battery_data: Result<VBFSetBattery, libloading::Error>,
    vbf_set_gps_data: Result<VBFSetGpsData, libloading::Error>,
}

impl VirtualBF {
    unsafe fn new(path: &Path) -> Self {
        let lib = Library::new(&path).unwrap();
        macro_rules! get_vbf_method {
            ($fn:ident) => {
                let name = format!("{}\0", stringify!($fn));
                let $fn = lib.get(name.as_bytes()).map(|sym| *sym);
            };
        }

        get_vbf_method!(vbf_init);
        get_vbf_method!(vbf_update);
        get_vbf_method!(vbf_arm);
        get_vbf_method!(vbf_disarm);
        get_vbf_method!(vbf_update_serial_ws);
        get_vbf_method!(vbf_get_is_armed);
        get_vbf_method!(vbf_get_is_beeping);
        get_vbf_method!(vbf_get_arming_disable_flags);
        get_vbf_method!(vbf_get_micros_passed);
        get_vbf_method!(vbf_get_motor_signals);
        get_vbf_method!(vbf_set_rc_data);
        get_vbf_method!(vbf_set_gyro_data);
        get_vbf_method!(vbf_set_accel_data);
        get_vbf_method!(vbf_set_attitude);
        get_vbf_method!(vbf_set_battery_data);
        get_vbf_method!(vbf_set_gps_data);

        Self {
            lib,
            vbf_init,
            vbf_update,
            vbf_arm,
            vbf_disarm,
            vbf_update_serial_ws,
            vbf_get_is_armed,
            vbf_get_is_beeping,
            vbf_get_arming_disable_flags,
            vbf_get_micros_passed,
            vbf_get_motor_signals,
            vbf_set_rc_data,
            vbf_set_gyro_data,
            vbf_set_accel_data,
            vbf_set_attitude,
            vbf_set_battery_data,
            vbf_set_gps_data,
        }
    }
}

macro_rules! impl_virtual_bf_fn {
    ($fn:ident) => {
        unsafe fn $fn(&self) {
            (self.$fn.as_ref().unwrap())()
        }
    };

    ($fn:ident; $return_type:ty) => {
        unsafe fn $fn(&self) -> $return_type {
            (self.$fn.as_ref().unwrap())()
        }
    };

    ($fn:ident, $($arg_name:ident: $arg_type:ty),*) => {
        unsafe fn $fn(&self, $($arg_name: $arg_type),*) {
            (self.$fn.as_ref().unwrap())($($arg_name),*)
        }
    };
}

impl VirtualBF {
    impl_virtual_bf_fn!(vbf_init, file_name: *const std::os::raw::c_char);
    impl_virtual_bf_fn!(vbf_update, micros_passed: u64);
    impl_virtual_bf_fn!(vbf_arm);
    impl_virtual_bf_fn!(vbf_disarm);
    impl_virtual_bf_fn!(vbf_update_serial_ws);
    impl_virtual_bf_fn!(vbf_get_is_armed; bool);
    impl_virtual_bf_fn!(vbf_get_is_beeping; bool);
    impl_virtual_bf_fn!(vbf_get_arming_disable_flags; std::os::raw::c_int);
    impl_virtual_bf_fn!(vbf_get_micros_passed; u64);
    impl_virtual_bf_fn!(vbf_get_motor_signals, motor_signal: *mut f32);
    impl_virtual_bf_fn!(vbf_set_rc_data, rc_data: *const f32);
    impl_virtual_bf_fn!(vbf_set_gyro_data, gyro_data: *const f32);
    impl_virtual_bf_fn!(vbf_set_accel_data, accel_data: *const f32);
    impl_virtual_bf_fn!(vbf_set_attitude, attitude_quaternion: *const f32);
    impl_virtual_bf_fn!(vbf_set_battery_data, cell_count: u8, voltage: f32, sagged_voltage: f32, amperage: f64, mah_drawn: f64);
    impl_virtual_bf_fn!(vbf_set_gps_data, lat: i32, lon: i32, alt: i32, gound_speed: u16);
}

#[derive(Clone)]
struct VirtualBFWrapper {
    virtual_bf: Arc<VirtualBF>,
}

impl VirtualBFWrapper {
    fn new() -> Self {
        let path = Path::new("/home/gabor/ascent/quad/crates/flight_controller/virtual_bf/src/libvirtual_betaflight.so");
        let virtual_bf = unsafe { VirtualBF::new(path) };
        Self {
            virtual_bf: Arc::new(virtual_bf),
        }
    }

    // Do not leak any pointers
    pub fn access<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&VirtualBF) -> R,
    {
        f(&self.virtual_bf)
    }
}

// Living on the edge, but actually is good, since we don'r really expose pointers and we are super
// careful not to introduce race conditions etc.
unsafe impl Send for VirtualBFWrapper {}
unsafe impl Sync for VirtualBFWrapper {}

#[derive(Default)]
struct ViratulBFManager {
    instances: Arc<Mutex<HashMap<String, VirtualBFWrapper>>>,
}

impl ViratulBFManager {
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
                let wrapper = VirtualBFWrapper::new();
                vacant_entry.insert(wrapper);
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
        F: FnOnce(&VirtualBF) -> R,
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

static VIRTUAL_BF_INSTANCE_MANAGER: Lazy<ViratulBFManager> = Lazy::new(|| ViratulBFManager::new());

impl BFController {
    pub fn new() -> Self {
        let instance_id = format!("default_id");
        let scheduler_delta = Duration::from_micros(50);
        Self {
            scheduler_delta,
            instance_id,
        }
    }
}

impl FlightController for BFController {
    fn init(&self) {
        VIRTUAL_BF_INSTANCE_MANAGER.register_new(self.instance_id.clone());
        VIRTUAL_BF_INSTANCE_MANAGER.access(&self.instance_id, |virtual_bf| unsafe {
            let file_name =
                CString::new("/home/gabor/ascent/quad/eeprom.bin").expect("CString::new failed");
            virtual_bf.vbf_init(file_name.as_ptr());
            virtual_bf.vbf_arm();
        })
    }

    fn deinit(&self) {
        VIRTUAL_BF_INSTANCE_MANAGER.close(&self.instance_id);
    }

    fn update(&self, delta_time_us: u64, fc_update: FlightControllerUpdate) -> MotorInput {
        VIRTUAL_BF_INSTANCE_MANAGER.access(&self.instance_id, |virtual_bf| unsafe {
            virtual_bf.vbf_set_battery_data(
                fc_update.battery_update.cell_count,
                fc_update.battery_update.bat_voltage as f32,
                fc_update.battery_update.bat_voltage_sag as f32,
                fc_update.battery_update.amperage,
                fc_update.battery_update.m_ah_drawn,
            );
            let attitude_update = fc_update.gyro_update.rotation.map(|x| x as f32);
            virtual_bf.vbf_set_attitude(attitude_update.as_ptr());

            let accel_update = fc_update.gyro_update.linear_acc.map(|x| x as f32);
            virtual_bf.vbf_set_accel_data(accel_update.as_ptr());

            let gyro_update = fc_update.gyro_update.angular_velocity.map(|x| x as f32);
            virtual_bf.vbf_set_gyro_data(gyro_update.as_ptr());

            let rc_data = fc_update.channels.to_bf_channels();
            virtual_bf.vbf_set_rc_data(rc_data.as_ptr());

            virtual_bf.vbf_update(delta_time_us);

            let mut motors_signal = [0.; 4];
            virtual_bf.vbf_get_motor_signals(motors_signal.as_mut_ptr());
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
        VIRTUAL_BF_INSTANCE_MANAGER.close(&self.instance_id);
    }
}

#[cfg(test)]
mod test {
    use super::VIRTUAL_BF_INSTANCE_MANAGER;

    // laods and unloads bf 100.000 times
    #[test]
    fn test_mem_leak() {
        let counter = 0;
        for _ in 0..10000000 {
            let instance_id = counter.to_string();
            VIRTUAL_BF_INSTANCE_MANAGER.register_new(instance_id.clone());
            VIRTUAL_BF_INSTANCE_MANAGER.close(&instance_id);
        }
    }
}
