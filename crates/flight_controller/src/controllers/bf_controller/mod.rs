mod bf;
use bf::BFWorker;
use std::{
    sync::{
        mpsc::{self, Receiver, SyncSender},
        Arc, Mutex,
    },
    thread,
};

use crate::{FlightController, FlightControllerUpdate, MotorInput};

#[derive(Default)]
pub struct BFController {
    tx: Option<SyncSender<FlightControllerUpdate>>,
    rx: Option<Receiver<MotorInput>>,
}

#[derive(Default)]
struct FCMutex {
    update: Option<FlightControllerUpdate>,
    motor_input: Option<MotorInput>,
}

impl BFController {
    fn new() -> Self {
        Self { tx: None, rx: None }
    }
}

impl FlightController for BFController {
    fn init(&mut self) {
        let (controller_tx, worker_rx) = mpsc::sync_channel(0); // no buffering
        let (worker_tx, controller_rx) = mpsc::channel();
        let fc_mutex = Arc::new(Mutex::new(FCMutex::default()));
        let worker = BFWorker {
            rx: worker_rx,
            tx: worker_tx,
            fc_mutex: fc_mutex.clone(),
        };
        self.tx = Some(controller_tx);
        self.rx = Some(controller_rx);
        thread::spawn(move || {
            unsafe { worker.init() };
            worker.work();
        });
    }

    fn update(&self, update: FlightControllerUpdate) -> Option<MotorInput> {
        let (Some(tx), Some(rx)) = (&self.tx, &self.rx) else {
            panic!("No channels initialized")
        };
        if tx.try_send(update).is_ok() {
            // We wait for the BF update
            Some(rx.recv().unwrap())
        } else {
            // If we are unable to input the flight controller, do not return the input
            None
        }
    }

    fn set_armed(&self) {}
}

#[cfg(test)]
mod test {
    use super::{bf::BFWorker, BFController, FCMutex};
    use crate::{BatteryUpdate, Channels, FlightController, FlightControllerUpdate, GyroUpdate};
    use std::{
        sync::{mpsc, Arc, Mutex},
        thread,
        time::Duration,
    };

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
            throttle: 1.,
            yaw: 0.,
            pitch: 0.,
            roll: 0.,
        };

        let flight_controller_update = FlightControllerUpdate {
            battery_update,
            gyro_update,
            channels,
        };

        let mut controller = BFController::default();
        controller.init();

        loop {
            controller.update(flight_controller_update);
            thread::sleep(Duration::from_micros(10));
        }
    }

    #[test]
    fn brrr_worker_bf_controller() {
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
            throttle: 1.,
            yaw: 0.,
            pitch: 0.,
            roll: 0.,
        };

        let (tx, _) = mpsc::channel();
        let (_, rx) = mpsc::channel();

        let fc_mutex = Arc::new(Mutex::new(FCMutex::default()));
        let worker = BFWorker { rx, tx, fc_mutex };
        worker.brrr(FlightControllerUpdate {
            battery_update,
            gyro_update,
            channels,
        });
    }

    #[test]
    fn normal_controller() {
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
            throttle: 1.,
            yaw: 0.,
            pitch: 0.,
            roll: 0.,
        };

        let (tx, _) = mpsc::channel();
        let (_, rx) = mpsc::channel();

        let fc_mutex = Arc::new(Mutex::new(FCMutex::default()));
        let mutex_clone = fc_mutex.clone();

        let thread = move || {
            let worker = BFWorker {
                rx,
                tx,
                fc_mutex: mutex_clone,
            };
            unsafe { worker.init() };
            worker.work2();
        };

        thread::spawn(thread);

        loop {
            {
                let mut mutex = fc_mutex.lock().unwrap();
                mutex.update = Some(FlightControllerUpdate {
                    battery_update,
                    gyro_update,
                    channels,
                });
            }
            thread::sleep(Duration::from_millis(10));
        }
    }
}
