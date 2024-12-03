use crate::{
    bindings::sitl_generated::{motorsPwm, FCInput},
    MotorInput,
};
use libloading::Library;
use std::{
    ffi::{c_char, CString},
    path::Path,
};

type AscentInit = unsafe fn(file_name: *const c_char);
type AscentUpdate = unsafe fn(dt: u64, fc_input: FCInput) -> bool;

struct BFWorker2 {
    libsitl: Library,
}

impl BFWorker2 {
    fn new() -> Self {
        let path = Path::new("/home/gabor/ascent/quad/crates/flight_controller/sitl/libsitl.so");
        let libsitl = unsafe { libloading::Library::new(path).unwrap() };
        Self { libsitl }
    }

    fn init(&self) {
        let file_name = CString::new("eeprom.bin").expect("CString::new failed");
        let init: libloading::Symbol<AscentInit> =
            unsafe { self.libsitl.get(b"ascent_init").unwrap() };
        unsafe { init(file_name.as_ptr()) }
    }

    fn update(&self, dt: u64) {
        unsafe {
            let update: libloading::Symbol<AscentUpdate> = self.libsitl.get(b"update").unwrap();
            let motor_input = MotorInput {
                input: [
                    motorsPwm[0] as f64 / 1000.,
                    motorsPwm[1] as f64 / 1000.,
                    motorsPwm[2] as f64 / 1000.,
                    motorsPwm[3] as f64 / 1000.,
                ],
            };
            // update(dt);
        }
    }
}
