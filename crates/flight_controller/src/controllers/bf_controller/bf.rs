use std::{
    os::raw::c_ulonglong,
    path::Path,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

use libloading::Library;

use crate::{
    bindings::sitl_generated::{
        armingFlags, getCurrentMeter, getVoltageMeter, imuSetAttitudeQuat, init, motorsPwm,
        rxFrameState_e_RX_FRAME_COMPLETE, rxProvider_t_RX_PROVIDER_UDP, rxRuntimeState,
        rxRuntimeState_s, scheduler, setCellCount, virtualAccDev, virtualAccSet, virtualGyroDev,
        virtualGyroSet,
    },
    BatteryUpdate, GyroUpdate,
};

use super::FCMutex;

fn constarain_i16(val: f64, min: f64, max: f64) -> i16 {
    if val < min {
        min as i16
    } else if val > max {
        max as i16
    } else {
        val as i16
    }
}

static mut RC_DATA_CACHE: [u16; 16] = [1500; 16];
const ACC_SCALE: f64 = 256. / 9.80665;
const GYRO_SCALE: f64 = 16.4;
const RAD2DEG: f64 = 180.0 / 3.14159265358979323846264338327950288;

unsafe extern "C" fn rx_rc_read_data(_: *const rxRuntimeState_s, channel: u8) -> f32 {
    RC_DATA_CACHE[channel as usize] as f32
}

unsafe extern "C" fn rx_rc_frame_status(_: *mut rxRuntimeState_s) -> u8 {
    rxFrameState_e_RX_FRAME_COMPLETE as u8
}

pub struct BFWorker {
    pub fc_mutex: Arc<Mutex<FCMutex>>,
    pub scheduler_delta: Duration,
    libsitl: Library,
}

type AscentInit = unsafe fn(file_name: *const i8);
type AscentUpdate = unsafe fn(delta_time_us: c_ulonglong) -> bool;

impl BFWorker {
    pub fn new(fc_mutex: Arc<Mutex<FCMutex>>, scheduler_delta: Duration) -> Self {
        // TODO: do not hardcode things
        let path = Path::new("/home/gabor/ascent/quad/crates/flight_controller/sitl/libsitl.so");
        let libsitl = unsafe { libloading::Library::new(path).unwrap() };

        Self {
            fc_mutex,
            scheduler_delta,
            libsitl,
        }
    }

    unsafe fn set_rc_data(&self, data: [f32; 8]) {
        for i in 0..8 {
            RC_DATA_CACHE[i] = (1500. + data[i] * 500.) as u16;
        }
    }

    unsafe fn update_battery(&self, update: BatteryUpdate) {
        setCellCount(update.cell_count);
        let voltage_meter = getVoltageMeter();
        if !voltage_meter.is_null() {
            (*voltage_meter).unfiltered = (update.bat_voltage_sag * 1e2) as u16;
            (*voltage_meter).displayFiltered = (update.bat_voltage_sag * 1e2) as u16;
            (*voltage_meter).sagFiltered = (update.bat_voltage * 1e2) as u16;
        } else {
            println!("Voltage meter not found");
        }
        let current_meter = getCurrentMeter();
        if !current_meter.is_null() {
            (*current_meter).amperage = (update.amperage * 1e2) as i32;
            (*current_meter).amperageLatest = (update.amperage * 1e2) as i32;
            (*current_meter).mAhDrawn = (update.m_ah_drawn) as i32;
        } else {
            println!("Current meter not found");
        }
    }

    pub unsafe fn init_arm(&self) {
        init();
        armingFlags |= 1;
        rxRuntimeState.channelCount = 8; // seems redundant
        rxRuntimeState.rcReadRawFn = Some(rx_rc_read_data);
        rxRuntimeState.rcFrameStatusFn = Some(rx_rc_frame_status);
        rxRuntimeState.rxProvider = rxProvider_t_RX_PROVIDER_UDP;
    }

    unsafe fn update_gyro_acc(&self, update: GyroUpdate) {
        // rotation is w, i, j, k
        imuSetAttitudeQuat(
            update.rotation[0] as f32,
            -update.rotation[3] as f32,
            update.rotation[1] as f32,
            -update.rotation[2] as f32,
        );

        let x = constarain_i16(update.linear_acc[2] * ACC_SCALE, -32767., 32767.); // TODO: sign
        let y = constarain_i16(update.linear_acc[0] * ACC_SCALE, -32767., 32767.); // TODO: sign
        let z = constarain_i16(update.linear_acc[1] * ACC_SCALE, -32767., 32767.);
        virtualAccSet(virtualAccDev, x, y, z);

        let x = constarain_i16(
            -update.angular_velocity[2] * GYRO_SCALE * RAD2DEG,
            -32767.,
            32767.,
        );
        let y = constarain_i16(
            -update.angular_velocity[0] * GYRO_SCALE * RAD2DEG,
            -32767.,
            32767.,
        );
        let z = constarain_i16(
            -update.angular_velocity[1] * GYRO_SCALE * RAD2DEG,
            -32767.,
            32767.,
        );

        virtualGyroSet(virtualGyroDev, x, y, z);
    }

    /// It locks the input mutex. If a new input can be read, it gets processed and the scheduler
    /// runs. If no new input can be read, only the scheduler runs.
    pub fn update(&self) {
        let mut fc_mutex = self.fc_mutex.lock().unwrap();
        if let Some(update) = fc_mutex.update {
            let motor_input = unsafe {
                self.set_rc_data(update.channels.to_bf_channels());
                self.update_battery(update.battery_update);
                self.update_gyro_acc(update.gyro_update);
                scheduler();
                [
                    motorsPwm[0] as f64 / 1000.,
                    motorsPwm[1] as f64 / 1000.,
                    motorsPwm[2] as f64 / 1000.,
                    motorsPwm[3] as f64 / 1000.,
                ]
            };
            fc_mutex.update = None;
            fc_mutex.motor_input.set_input(motor_input);
        } else {
            unsafe { scheduler() }
        }
    }

    /// Ensures that the input is only read at a certain frequency.
    pub fn work(&self) {
        loop {
            let start = Instant::now();
            self.update();
            let elapsed = start.elapsed();
            let sleep_time = self.scheduler_delta.saturating_sub(elapsed);
            thread::sleep(sleep_time);
        }
    }

    pub fn init(&self) {
        unsafe { init() }
    }
}
