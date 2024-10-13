// Done in the style so that BF can be added later on

use nalgebra::{Quaternion, Vector3, Vector4};

use crate::Drone;

#[derive(Debug, Clone, Copy)]
struct Channels {
    throttle: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorInput(Vector4<f64>);

impl Default for MotorInput {
    fn default() -> Self {
        Self(Vector4::new(0.5, 0.5, 0.5, 0.5))
    }
}

impl MotorInput {
    pub fn pwms(&self) -> Vector4<f64> {
        self.0
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct SetPoints {
    roll: f64,
    pitch: f64,
    yaw: f64,
}

// according to BF
#[derive(Debug, Clone, Copy, Default)]
struct VoltageMeter {
    unfiltered: u16,
    display_filtered: u16,
    sag_filtered: u16,
}

#[derive(Debug, Clone, Copy, Default)]
struct CurrentMeter {
    amperage: i32,
    amperage_latest: i32,
    m_ah_drawn: i32,
}

// TODO: all these should be i16 I guess
#[derive(Debug, Clone, Copy, Default)]
struct Imu {
    attitude: Quaternion<f64>,
    virtual_acceleration: Vector3<f64>,
    virtual_gyro: Vector3<f64>,
}

// basic PID controller
#[derive(Default, Clone)]
pub struct FlightController {
    throttle: f64,
    set_points: SetPoints,
    voltage_meter: VoltageMeter,
    current_meter: CurrentMeter,
    imu: Imu,
}

impl FlightController {
    fn set_channels(&mut self, channels: Channels) {
        let Channels {
            throttle,
            yaw,
            roll,
            pitch,
        } = channels;
        self.throttle = throttle;
        self.set_points.roll = roll;
        self.set_points.yaw = yaw;
        self.set_points.pitch = pitch;
    }

    fn update_battery(&mut self, drone: &Drone) {
        // set voltage meter
        self.voltage_meter.unfiltered = (drone.battery.get_bat_voltage_sag() * 1e2) as u16;
        self.voltage_meter.display_filtered = (drone.battery.get_bat_voltage_sag() * 1e2) as u16;
        self.voltage_meter.sag_filtered = (drone.battery.get_bat_voltage() * 1e2) as u16;

        // set amperage meter
        self.current_meter.amperage = (drone.battery.amperage() * 1e2) as i32;
        self.current_meter.amperage_latest = (drone.battery.amperage() * 1e2) as i32;
        self.current_meter.m_ah_drawn = (drone.battery.m_ah_drawn() * 1e2) as i32;
    }

    fn update_gyro_acc(&mut self, drone: &Drone) {
        // original code is:
        //              simState.rotation[3],
        //              -simState.rotation[2],
        //              -simState.rotation[0],
        //              simState.rotation[1]

        // TODO: readd this
        // self.imu.attitude = DQuat::from_vec4(drone.gyro.rotation());
        self.imu.virtual_acceleration = drone.gyro.acceleration();
        self.imu.virtual_gyro = drone.gyro.angular_vel();
    }

    fn update_gps(&mut self, drone: &Drone) {
        todo!("probably not that important")
    }

    fn update(&mut self, drone: &Drone) -> MotorInput {
        self.update_battery(drone);
        self.update_gyro_acc(drone);
        // self.update_gps(drone);
        MotorInput::default()
    }
}

#[derive(Default, Clone)]
pub struct Model {
    flight_controller: FlightController,
}

impl Model {
    pub fn update(&mut self, drone: &Drone) -> MotorInput {
        self.flight_controller.update(drone)
    }
}
