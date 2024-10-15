use crate::{
    bindings::{
        bf_bindings_generated::{
            getCurrentMeter, getVoltageMeter, init, rxFrameState_e_RX_FRAME_COMPLETE,
            rxProvider_t_RX_PROVIDER_UDP, rxRuntimeState, rxRuntimeState_s, rxRuntimeState_t,
            sensors, sensors_e_SENSOR_ACC, virtualGyroDev, virtualGyroSet,
        },
        SIMULATOR_MAX_RC_CHANNELS_U8, SIMULATOR_MAX_RC_CHANNELS_USIZE,
    },
    BatteryUpdate,
};

// My eyes are hurting
static mut RC_DATA_CACHE: [u16; 16] = [0; 16];
const GYRO_SCALE: f64 = 16.4;
const M_PI: f64 = 3.14159265358979323846264338327950288;
const RAD2DEG: f64 = 180.0 / M_PI;

fn constarain_i16(val: f64, min: f64, max: f64) -> i16 {
    if val < min {
        min as i16
    } else if val > max {
        max as i16
    } else {
        val as i16
    }
}

pub struct BFController;

unsafe extern "C" fn rx_rc_read_data(_: *const rxRuntimeState_s, channel: u8) -> f32 {
    RC_DATA_CACHE[channel as usize] as f32
}

unsafe extern "C" fn rx_rc_frame_status(_: *mut rxRuntimeState_s) -> u8 {
    rxFrameState_e_RX_FRAME_COMPLETE as u8
}

unsafe extern "C" fn rx_rc_frame_time_us(_: *mut rxRuntimeState_s) -> u8 {
    todo!()
}

impl BFController {
    fn init() {
        unsafe {
            init();
        }
    }

    fn reset_rc_data(&mut self) {
        for i in 0..SIMULATOR_MAX_RC_CHANNELS_USIZE {
            unsafe {
                RC_DATA_CACHE[i] = 1000;
            }
        }
    }

    fn rx_rc_read_data(&self, _rx_runtime_state: &rxRuntimeState_t, channel: u8) -> u16 {
        unsafe { RC_DATA_CACHE[channel as usize] }
    }

    fn set_rc_data(&mut self, data: [f32; 8]) {
        for i in 0..8 {
            unsafe {
                RC_DATA_CACHE[i] = (1500. + data[i] * 500.) as u16;
            }
        }
        unsafe {
            rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS_U8; // seems redundant
            rxRuntimeState.rcReadRawFn = Some(rx_rc_read_data);
            rxRuntimeState.rcFrameStatusFn = Some(rx_rc_frame_status);
            rxRuntimeState.rxProvider = rxProvider_t_RX_PROVIDER_UDP;
            rxRuntimeState.lastRcFrameTimeUs = 0; // TODO: this needs to be added
        }
    }

    fn udpate_battery(update: BatteryUpdate) {
        unsafe {
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
    }

    fn update_gyro_acc(gyro: [f64; 3]) {
        unsafe {
            if sensors(sensors_e_SENSOR_ACC) {
                println!("snesors acc");
            } else {
                println!("no sensors acc");
            }
            let x = constarain_i16(-gyro[2] * GYRO_SCALE * RAD2DEG, -32767., 32767.);
            let y = constarain_i16(-gyro[0] * GYRO_SCALE * RAD2DEG, -32767., 32767.);
            let z = constarain_i16(-gyro[1] * GYRO_SCALE * RAD2DEG, -32767., 32767.);
            virtualGyroSet(virtualGyroDev, x, y, z);
        }
    }
}

#[cfg(test)]
mod test {
    use super::BFController;
    use crate::BatteryUpdate;

    #[test]
    fn bf_controller() {
        BFController::init();
        let battery_update = BatteryUpdate {
            bat_voltage_sag: 1.,
            bat_voltage: 1.,
            amperage: 1.,
            m_ah_drawn: 1.,
        };
        BFController::udpate_battery(battery_update);
        BFController::update_gyro_acc([0., 0., 0.]);
    }
}
