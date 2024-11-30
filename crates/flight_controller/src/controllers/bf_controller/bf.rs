use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use crate::{
    bindings::{
        bf_bindings_generated::{
            armingFlags, getCurrentMeter, getVoltageMeter, imuSetAttitudeQuat, init,
            rxFrameState_e_RX_FRAME_COMPLETE, rxProvider_t_RX_PROVIDER_UDP, rxRuntimeState,
            rxRuntimeState_s, scheduler, setCellCount, timeUs_t, virtualAccDev, virtualAccSet,
            virtualGyroDev, virtualGyroSet,
        },
        motorsPwm, SIMULATOR_MAX_RC_CHANNELS_U8,
    },
    BatteryUpdate, GyroUpdate, MotorInput,
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

unsafe extern "C" fn rx_rc_frame_time_us() -> timeUs_t {
    todo!()
}

pub struct BFWorker {
    pub fc_mutex: Arc<Mutex<FCMutex>>,
    // pub counter: RefCell<u64>,
}

impl BFWorker {
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
        rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS_U8; // seems redundant
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

    pub fn update(&self) {
        let mut fc_mutex = self.fc_mutex.lock().unwrap();
        if let Some(update) = fc_mutex.update {
            let motor_input = unsafe {
                self.set_rc_data(update.channels.to_bf_channels());
                self.update_battery(update.battery_update);
                self.update_gyro_acc(update.gyro_update);
                scheduler();
                MotorInput([
                    motorsPwm[0] as f64 / 1000.,
                    motorsPwm[1] as f64 / 1000.,
                    motorsPwm[2] as f64 / 1000.,
                    motorsPwm[3] as f64 / 1000.,
                ])
            };
            fc_mutex.update = None;
            fc_mutex.motor_input = Some(motor_input);
        } else {
            unsafe { scheduler() }
        }
    }

    pub fn work(&self) {
        loop {
            self.update();
            thread::sleep(Duration::from_micros(50));
        }
    }

    pub fn init(&self) {
        unsafe { init() }
    }
}
