mod bf;
use bf::BFWorker;
use std::{
    sync::{Arc, Mutex},
    thread,
};

use crate::{FlightController, FlightControllerUpdate, MotorInput};

#[derive(Default)]
pub struct BFController {
    fc_mutex: Arc<Mutex<FCMutex>>,
}

#[derive(Default)]
struct FCMutex {
    update: Option<FlightControllerUpdate>,
    motor_input: Option<MotorInput>,
}

impl BFController {
    pub fn new() -> Self {
        let fc_mutex = Arc::new(Mutex::new(FCMutex::default()));
        Self { fc_mutex }
    }

    pub fn only_scheduler(&self) {
        let mutex_clone = self.fc_mutex.clone();

        let thread = move || {
            let worker = BFWorker {
                fc_mutex: mutex_clone,
            };
            unsafe { worker.init() };
            worker.work();
        };

        let handler = thread::spawn(thread);
        handler.join().unwrap();
    }
}

impl FlightController for BFController {
    fn init(&self) {
        let mutex_clone = self.fc_mutex.clone();

        let thread = move || {
            let worker = BFWorker {
                fc_mutex: mutex_clone,
            };
            unsafe { worker.init_arm() };
            worker.work();
        };
        thread::spawn(thread);
    }

    fn update(&self, update: FlightControllerUpdate) -> Option<MotorInput> {
        let mut mutex = self.fc_mutex.lock().unwrap();
        mutex.update = Some(update);
        mutex.motor_input
    }
}

#[cfg(test)]
mod test {
    use super::BFController;
    use crate::{BatteryUpdate, Channels, FlightController, FlightControllerUpdate, GyroUpdate};
    use std::{thread, time::Duration};

    #[test]
    fn controller_bf_controller() {
        let battery_update = BatteryUpdate {
            bat_voltage_sag: 16.,
            bat_voltage: 16.,
            amperage: 1.,
            m_ah_drawn: 1.,
            cell_count: 4,
        };

        let gyro_update = GyroUpdate {
            rotation: [1., 0., 0., 0.],
            acc: [0., 0., 0.],
            gyro: [0., 0., 0.],
        };

        let channels = Channels {
            throttle: -1.,
            yaw: 0.,
            pitch: 0.,
            roll: 0.,
        };

        let flight_controller_update = FlightControllerUpdate {
            battery_update,
            gyro_update,
            channels,
        };

        let controller = BFController::default();
        controller.init();

        loop {
            controller.update(flight_controller_update);
            thread::sleep(Duration::from_micros(10));
        }
    }

    #[test]
    fn only_scheduler() {
        let controller = BFController::default();
        controller.only_scheduler();
    }
}
