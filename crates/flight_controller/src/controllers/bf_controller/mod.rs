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

    pub fn only_telemetry(&self) {
        let mutex_clone = self.fc_mutex.clone();

        let thread = move || {
            let worker = BFWorker {
                fc_mutex: mutex_clone,
            };
            worker.init();
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
