//! TODO: this whole thing super in progress. I am just playing around with the manager thing

use crate::{FlightController, MotorInput};
use core::panic;
use libc::{dlclose, dlerror, dlinfo, dlmopen, dlsym, Lmid_t, LM_ID_NEWLM, RTLD_DI_LMID};
use once_cell::sync::Lazy;
use std::{
    collections::{hash_map::Entry, HashMap},
    ffi::{CStr, CString},
    os::raw::{self, c_void},
    sync::{Arc, Mutex},
    time::Duration,
};
use uuid::Uuid;

const RTLD_FLAGS: i32 = 0x0002; // Resolves all symbols and do not use them for further resolutions
const LIB_PATH: &str =
    "/home/gabor/ascent/quad/crates/flight_controller/virtual_bf/src/libvirtual_betaflight.so\0";

type VBFInit = unsafe extern "C" fn(file_name: *const std::os::raw::c_char);
type VBFUpdate = unsafe extern "C" fn(micros_passed: u64);
type VBFArm = unsafe extern "C" fn();
// type VBFDisArm = unsafe extern "C" fn();
// type VBFUpdateSerialWs = unsafe extern "C" fn();
// type VBFGetIsArmed = unsafe extern "C" fn() -> bool;
// type VBFGetIsBeeping = unsafe extern "C" fn() -> bool;
// type VBFGetArmingDisbleFlags = unsafe extern "C" fn() -> std::os::raw::c_int;
// type VBFGetMicrosPassed = unsafe extern "C" fn() -> u64;
type VBFGetMotorSignals = unsafe extern "C" fn(*mut f32);
type VBFSetMotorRcData = unsafe extern "C" fn(*const f32);
type VBFSetGyroData = unsafe extern "C" fn(*const f32);
type VBFSetAttitude = unsafe extern "C" fn(*const f32);
type VBFSetAccelData = unsafe extern "C" fn(*const f32);
type VBFSetBattery = unsafe extern "C" fn(u8, f32, f32, f64, f64);
// type VBFSetGpsData = unsafe extern "C" fn(i32, i32, i32, u16);
// type VBFGetAttitudeQuat = unsafe extern "C" fn(*mut f32);

// represents a loaded library
#[derive(Debug)]
pub struct VirtualBF2 {
    pub lib_handle: *mut raw::c_void,
    pub lmid: i64,
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
unsafe fn get_lm_id(dl_handle: *mut raw::c_void) -> i64 {
    let mut lmid: Lmid_t = 0;
    let result = dlinfo(dl_handle, RTLD_DI_LMID, &mut lmid as *mut _ as *mut c_void);
    if result != 0 {
        panic!("Could not get dl info");
    }
    lmid as Lmid_t
}

impl VirtualBF2 {
    unsafe fn new(lib_handle: *mut c_void, lmid: i64) -> Result<Self, CString> {
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
            lmid,
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
        let code = unsafe { dlclose(self.lib_handle) };
        if code == 0 {
            log::info!("Vbf instance with handle {:?} dropped", self.lib_handle);
        }
    }
}

unsafe impl Send for VirtualBF2 {}
unsafe impl Sync for VirtualBF2 {}

#[derive(Debug)]
pub struct BFManager2 {
    instances: Mutex<HashMap<String, Arc<VirtualBF2>>>,
    available_workspace_ids: Mutex<Vec<i64>>,
}

impl BFManager2 {
    fn new() -> Self {
        Self {
            instances: Mutex::new(HashMap::new()),
            available_workspace_ids: Mutex::new(vec![]),
        }
    }

    unsafe fn load_library(&self) -> (*mut c_void, i64) {
        let mut available_workspace_ids = self.available_workspace_ids.lock().unwrap();
        let lib_handle = if let Some(workspace_id) = available_workspace_ids.pop() {
            dlmopen(workspace_id, LIB_PATH.as_ptr().cast(), RTLD_FLAGS)
        } else {
            dlmopen(LM_ID_NEWLM, LIB_PATH.as_ptr().cast(), RTLD_FLAGS)
        };
        check_dl_error(lib_handle).unwrap();

        let lmid = get_lm_id(lib_handle);
        (lib_handle, lmid)
    }

    unsafe fn register_new2(&self) -> String {
        let (lib_handle, lmid) = self.load_library();

        let new_id = Uuid::new_v4().to_string();
        let mut guard = self.instances.lock().unwrap();
        let entry = guard.entry(new_id.clone());

        match entry {
            Entry::Vacant(vacant) => {
                let controller = unsafe { VirtualBF2::new(lib_handle, lmid).unwrap() };
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
        let mut instances_guard = self.instances.lock().unwrap();
        let mut workspace_guard = self.available_workspace_ids.lock().unwrap();
        let vbf = instances_guard.remove(instance_id).unwrap();
        workspace_guard.push(vbf.lmid);
    }

    // only static managers can register new controllers
    pub fn request_new_controller(&'static self) -> BFController2 {
        let instance_id = unsafe { self.register_new2() };
        let scheduler_delta = Duration::from_micros(50);
        BFController2 {
            manager: self,
            instance_id,
            scheduler_delta,
        }
    }
}

pub static VIRTUAL_BF_MANAGER_2: Lazy<BFManager2> = Lazy::new(|| BFManager2::new());

#[derive(Debug)]
pub struct BFController2 {
    pub instance_id: String,
    pub scheduler_delta: Duration,
    manager: &'static BFManager2,
}

impl Default for BFController2 {
    fn default() -> Self {
        // default manager
        VIRTUAL_BF_MANAGER_2.request_new_controller()
    }
}

impl Drop for BFController2 {
    fn drop(&mut self) {
        self.manager.close(&self.instance_id);
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
