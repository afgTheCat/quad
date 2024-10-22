mod bf;
use bf::BFWorker;
use std::{
    sync::mpsc::{self, Receiver, SyncSender},
    thread,
};

use crate::{FlightController, FlightControllerUpdate, MotorInput};

#[derive(Default)]
pub struct BFController {
    tx: Option<SyncSender<FlightControllerUpdate>>,
    rx: Option<Receiver<MotorInput>>,
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
        let worker = BFWorker {
            rx: worker_rx,
            tx: worker_tx,
        };
        self.tx = Some(controller_tx);
        self.rx = Some(controller_rx);
        unsafe {
            worker.init();
        }
        thread::spawn(move || worker.work());
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
    use std::{sync::mpsc, thread, time::Duration};

    use super::{bf::BFWorker, BFController};
    use crate::{BatteryUpdate, Channels, FlightController, FlightControllerUpdate, GyroUpdate};

    #[test]
    fn controller_bf_controller() {
        let mut controller = BFController::default();
        controller.init();
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

        loop {
            let flight_controller_update = FlightControllerUpdate {
                battery_update,
                gyro_update,
                channels,
            };
            controller.update(flight_controller_update);
            thread::sleep(Duration::from_micros(10));
        }
    }

    #[test]
    fn worker_bf_controller() {
        // let mut controller = BFController::default();
        // controller.init();
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

        let worker = BFWorker { rx, tx };
        worker.update2(FlightControllerUpdate {
            battery_update,
            gyro_update,
            channels,
        });

        // loop {
        //     let flight_controller_update = FlightControllerUpdate {
        //         battery_update,
        //         gyro_update,
        //         channels,
        //     };
        //     controller.update(flight_controller_update);
        //     thread::sleep(Duration::from_micros(10));
        // }
    }
}
