use crate::{
    bindings::{
        bf_bindings_generated::{
            armingFlag_e_ARMED, armingFlags, enableFlightMode, flightModeFlags_e_ANGLE_MODE,
            getArmingDisableFlags, getCurrentMeter, getVoltageMeter, imuSetAttitudeQuat, init,
            rxFrameState_e_RX_FRAME_COMPLETE, rxProvider_t_RX_PROVIDER_UDP, rxRuntimeState,
            rxRuntimeState_s, rxRuntimeState_t, scheduler, sensors, sensors_e_SENSOR_ACC,
            virtualAccDev, virtualAccSet, virtualGyroDev, virtualGyroSet,
        },
        motorsPwm, SIMULATOR_MAX_RC_CHANNELS_U8, SIMULATOR_MAX_RC_CHANNELS_USIZE,
    },
    BatteryUpdate, FlightControllerUpdate, GyroUpdate, MotorInput,
};

// My eyes are hurting
static mut RC_DATA_CACHE: [u16; 16] = [0; 16];
const GYRO_SCALE: f64 = 16.4;
const M_PI: f64 = 3.14159265358979323846264338327950288;
const ACC_SCALE: f64 = 256. / 9.80665;
const RAD2DEG: f64 = 180.0 / M_PI;
const VIDEO_LINES: usize = 16;

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
    unsafe fn init() {
        init();
    }

    unsafe fn reset_rc_data(&mut self) {
        for i in 0..SIMULATOR_MAX_RC_CHANNELS_USIZE {
            RC_DATA_CACHE[i] = 1000;
        }
    }

    unsafe fn set_rc_data(data: [f32; 8]) {
        for i in 0..8 {
            RC_DATA_CACHE[i] = (1500. + data[i] * 500.) as u16;
        }
        rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS_U8; // seems redundant
        rxRuntimeState.rcReadRawFn = Some(rx_rc_read_data);
        rxRuntimeState.rcFrameStatusFn = Some(rx_rc_frame_status);
        rxRuntimeState.rxProvider = rxProvider_t_RX_PROVIDER_UDP;
        rxRuntimeState.lastRcFrameTimeUs = 0; // TODO: this needs to be added
    }

    unsafe fn update_battery(update: BatteryUpdate) {
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

    unsafe fn update_gyro_acc(update: GyroUpdate) {
        if sensors(sensors_e_SENSOR_ACC) {
            imuSetAttitudeQuat(
                update.rotation[3] as f32,
                -update.rotation[2] as f32,
                -update.rotation[0] as f32,
                update.rotation[1] as f32,
            );
            let x = constarain_i16(-update.acc[2] * ACC_SCALE, -32767., 32767.);
            let y = constarain_i16(update.acc[0] * ACC_SCALE, -32767., 32767.);
            let z = constarain_i16(update.acc[1] * ACC_SCALE, -32767., 32767.);
            virtualAccSet(virtualAccDev, x, y, z);
        }
        let x = constarain_i16(-update.gyro[2] * GYRO_SCALE * RAD2DEG, -32767., 32767.);
        let y = constarain_i16(update.gyro[0] * GYRO_SCALE * RAD2DEG, -32767., 32767.);
        let z = constarain_i16(-update.gyro[1] * GYRO_SCALE * RAD2DEG, -32767., 32767.);
        virtualGyroSet(virtualGyroDev, x, y, z);
    }

    fn update_gps() {
        todo!()
    }

    fn update_osd() {
        todo!()
    }

    // cuz fuck it, why not
    unsafe fn set_armed() {
        armingFlags |= 1 << 0
    }

    unsafe fn enable_angle_mode() {
        enableFlightMode(flightModeFlags_e_ANGLE_MODE);
    }

    unsafe fn update(update: FlightControllerUpdate) -> MotorInput {
        let mut scheduler_executed = false;
        Self::update_battery(update.battery_update);
        Self::update_gyro_acc(update.gyro_update);

        // TODO: this should be incorporated somehow
        //  if (BF::sleep_timer > 0) {
        //   BF::sleep_timer -= dt;
        //   BF::sleep_timer = std::max(int64_t(0), BF::sleep_timer);
        // } else {
        //   BF::scheduler();
        //   schedulerExecuted = true;
        // }
        // TODO: what should we do with this?

        scheduler();
        let armed = armingFlags & armingFlag_e_ARMED as u8 == armingFlag_e_ARMED as u8;
        let motor_input_1 = motorsPwm[0] as f64 / 1000.;
        let motor_input_2 = motorsPwm[1] as f64 / 1000.;
        let motor_input_3 = motorsPwm[2] as f64 / 1000.;
        let motor_input_4 = motorsPwm[3] as f64 / 1000.;
        MotorInput([motor_input_1, motor_input_2, motor_input_3, motor_input_4])
    }
}

#[cfg(test)]
mod test {
    use super::BFController;
    use crate::{BatteryUpdate, FlightControllerUpdate, GyroUpdate};

    #[test]
    fn bf_controller() {
        unsafe { BFController::init() };
        unsafe { BFController::set_armed() };
        unsafe { BFController::enable_angle_mode() };
        unsafe { BFController::set_rc_data([0.5; 8]) };
        let battery_update = BatteryUpdate {
            bat_voltage_sag: 1.,
            bat_voltage: 1.,
            amperage: 1.,
            m_ah_drawn: 1.,
        };
        let gyro_update = GyroUpdate {
            rotation: [0., 0., 0., 0.],
            acc: [0., 0., 0.],
            gyro: [0., 0., 0.],
        };
        let flight_controller_update = FlightControllerUpdate {
            battery_update,
            gyro_update,
        };
        unsafe {
            loop {
                BFController::update(flight_controller_update);
            }
        }
    }
}
