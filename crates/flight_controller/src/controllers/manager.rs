//! TODO: this whole thing super in progress. I am just playing around with the manager thing

use core::panic;
use once_cell::sync::Lazy;
use std::{
    collections::{hash_map::Entry, HashMap},
    ffi::{CStr, CString},
    os::raw,
    sync::{Arc, Mutex},
    time::Duration,
};
use uuid::Uuid;

use crate::{FlightController, MotorInput};

const LM_ID_NEWLM: i64 = -1; // Create a new namespace
const RTLD_FLAGS: i32 = 0x0002; // Resolves all symbols and do not use them for further resolutions

unsafe extern "C" {
    fn dlmopen(lmid: i64, filename: *const raw::c_char, flag: i32) -> *mut raw::c_void;
    fn dlsym(handle: *mut raw::c_void, symbol: *const raw::c_char) -> *mut raw::c_void;
    fn dlclose(handle: *mut raw::c_void) -> i32;
    fn dlerror() -> *mut raw::c_char;

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
type VBFGetAttitudeQuat = unsafe extern "C" fn(*mut f32);

// represents a loaded library
#[derive(Debug)]
pub struct VirtualBF2 {
    pub lib_handle: *mut raw::c_void,
    pub vbf_init: VBFInit,
    pub vbf_update: VBFUpdate,
    pub vbf_arm: VBFArm,
    pub vbf_get_motor_signals: VBFGetMotorSignals,
    pub vbf_set_rc_data: VBFSetMotorRcData,
    pub vbf_set_gyro_data: VBFSetGyroData,
    pub vbf_set_accel_data: VBFSetAccelData,
    pub vbf_set_attitude: VBFSetAttitude,
    pub vbf_set_battery_data: VBFSetBattery,
}

unsafe fn check_dl_error(dl_sym: *mut raw::c_void) -> Result<(), CString> {
    if dl_sym.is_null() {
        let dlerror_str = dlerror();
        if dlerror_str.is_null() {
            panic!("Unknown error when loading a dl");
        } else {
            let error_str = CStr::from_ptr(dlerror_str);
            return Err(error_str.into());
        }
    }
    Ok(())
}

impl VirtualBF2 {
    unsafe fn new() -> Result<Self, CString> {
        let lib_path = "/home/gabor/ascent/quad/crates/flight_controller/virtual_bf/src/libvirtual_betaflight.so\0";
        let lib_handle = dlmopen(LM_ID_NEWLM, lib_path.as_ptr().cast(), RTLD_FLAGS);
        check_dl_error(lib_handle)?;

        macro_rules! get_vb_method {
            ($fn:ident, $fn_type:ty) => {
                let function_name = format!("{}\0", stringify!($fn));
                let fn_pointer = dlsym(lib_handle, function_name.as_ptr().cast());
                check_dl_error(fn_pointer)?;
                let $fn: $fn_type = std::mem::transmute(fn_pointer);
            };
        }

        get_vb_method!(vbf_init, VBFInit);
        get_vb_method!(vbf_update, VBFUpdate);
        get_vb_method!(vbf_arm, VBFArm);
        get_vb_method!(vbf_get_motor_signals, VBFGetMotorSignals);
        get_vb_method!(vbf_set_rc_data, VBFSetMotorRcData);
        get_vb_method!(vbf_set_gyro_data, VBFSetGyroData);
        get_vb_method!(vbf_set_accel_data, VBFSetAccelData);
        get_vb_method!(vbf_set_attitude, VBFSetAttitude);
        get_vb_method!(vbf_set_battery_data, VBFSetBattery);
        Ok(Self {
            lib_handle,
            vbf_init,
            vbf_arm,
            vbf_get_motor_signals,
            vbf_set_accel_data,
            vbf_set_attitude,
            vbf_update,
            vbf_set_rc_data,
            vbf_set_gyro_data,
            vbf_set_battery_data,
        })
    }
}

impl Drop for VirtualBF2 {
    fn drop(&mut self) {
        unsafe { dlclose(self.lib_handle) };
    }
}

unsafe impl Send for VirtualBF2 {}
unsafe impl Sync for VirtualBF2 {}

pub struct BFManager2 {
    instances: Mutex<HashMap<String, Arc<VirtualBF2>>>,
}

impl BFManager2 {
    fn new() -> Self {
        Self {
            instances: Mutex::new(HashMap::new()),
        }
    }

    fn register_new(&self) -> String {
        let new_id = Uuid::new_v4().to_string();
        let mut guard = self.instances.lock().unwrap();
        let entry = guard.entry(new_id.clone());
        match entry {
            Entry::Vacant(vacant) => {
                let controller = unsafe { VirtualBF2::new().unwrap() };
                vacant.insert(Arc::new(controller));
            }
            Entry::Occupied(_) => {
                todo!("Hanle collisions");
            }
        }
        new_id
    }

    pub fn access<F, R>(&self, instance_id: &str, f: F) -> R
    where
        F: FnOnce(&VirtualBF2) -> R,
    {
        // What we actually want is to prevent access only to the locked instance
        let instance;
        {
            let guard = self.instances.lock().unwrap();
            instance = guard.get(instance_id).unwrap().clone();
        }
        if Arc::strong_count(&instance) > 2 {
            panic!("Tried to access the same library multiple times");
        }
        f(&instance)
    }

    fn close(&self, instance_id: &str) {
        let mut guard = self.instances.lock().unwrap();
        guard.remove(instance_id);
    }
}

pub static VIRTUAL_BF_MANAGER_2: Lazy<BFManager2> = Lazy::new(|| BFManager2::new());

#[derive(Debug)]
pub struct BFController2 {
    pub instance_id: String,
    pub scheduler_delta: Duration,
}

impl BFController2 {
    pub fn new() -> Self {
        let instance_id = VIRTUAL_BF_MANAGER_2.register_new();
        let scheduler_delta = Duration::from_micros(50);
        Self {
            instance_id,
            scheduler_delta,
        }
    }
}

impl FlightController for BFController2 {
    fn init(&self) {
        VIRTUAL_BF_MANAGER_2.access(&self.instance_id, |virtual_bf| unsafe {
            let file_name =
                CString::new("/home/gabor/ascent/quad/eeprom.bin").expect("CString::new failed");
            (virtual_bf.vbf_init)(file_name.as_ptr());
            (virtual_bf.vbf_arm)();
        });
    }

    fn deinit(&self) {
        VIRTUAL_BF_MANAGER_2.close(&self.instance_id);
    }

    fn update(
        &self,
        delta_time_us: u64,
        update: crate::FlightControllerUpdate,
    ) -> crate::MotorInput {
        VIRTUAL_BF_MANAGER_2.access(&self.instance_id, |virtual_bf| unsafe {
            (virtual_bf.vbf_set_battery_data)(
                update.battery_update.cell_count,
                update.battery_update.bat_voltage as f32,
                update.battery_update.bat_voltage_sag as f32,
                update.battery_update.amperage,
                update.battery_update.m_ah_drawn,
            );
            let attitude_update = update.gyro_update.rotation.map(|x| x as f32);
            (virtual_bf.vbf_set_attitude)(attitude_update.as_ptr());

            let accel_update = update.gyro_update.linear_acc.map(|x| x as f32);
            (virtual_bf.vbf_set_accel_data)(accel_update.as_ptr());

            let gyro_update = update.gyro_update.angular_velocity.map(|x| x as f32);
            (virtual_bf.vbf_set_gyro_data)(gyro_update.as_ptr());

            let rc_data = update.channels.to_bf_channels();
            (virtual_bf.vbf_set_rc_data)(rc_data.as_ptr());

            (virtual_bf.vbf_update)(delta_time_us);

            let mut motors_signal = [0.; 4];
            (virtual_bf.vbf_get_motor_signals)(motors_signal.as_mut_ptr());
            MotorInput {
                input: motors_signal.map(|x| x as f64),
            }
        })
    }

    fn scheduler_delta(&self) -> Duration {
        self.scheduler_delta
    }
}

#[cfg(test)]
mod test_vb {
    use crate::controllers::manager::{BFController2, VIRTUAL_BF_MANAGER_2};

    #[test]
    fn test_unique_loading() {
        let controller1 = BFController2::new();
        let controller2 = BFController2::new();

        let lib_handle1 = VIRTUAL_BF_MANAGER_2
            .access(&controller1.instance_id, |virtual_bf| virtual_bf.lib_handle);
        let lib_handle2 = VIRTUAL_BF_MANAGER_2
            .access(&controller2.instance_id, |virtual_bf| virtual_bf.lib_handle);

        assert_ne!(lib_handle1, lib_handle2);
    }
}
