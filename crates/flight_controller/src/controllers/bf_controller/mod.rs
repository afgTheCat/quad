mod bf;
use bf::BFWorker;
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use crate::{FlightController, FlightControllerUpdate, MotorInput};

#[derive(Default)]
pub struct BFController {
    fc_mutex: Arc<Mutex<FCMutex>>,
    scheduler_delta: Duration,
}

#[derive(Default)]
struct FCMutex {
    update: Option<FlightControllerUpdate>,
    motor_input: Option<MotorInput>,
}

impl BFController {
    pub fn new() -> Self {
        let fc_mutex = Arc::new(Mutex::new(FCMutex::default()));
        let scheduler_delta = Duration::from_micros(50);
        Self {
            fc_mutex,
            scheduler_delta,
        }
    }
}

impl FlightController for BFController {
    fn init(&self) {
        let mutex_clone = self.fc_mutex.clone();
        let scheduler_delta = self.scheduler_delta.clone();

        let thread = move || {
            let worker = BFWorker {
                fc_mutex: mutex_clone,
                scheduler_delta,
            };
            unsafe { worker.init_arm() };
            worker.work();
        };
        thread::spawn(thread);
    }

    /// Reads the current motor input if it can be found.
    fn update(&self, update: FlightControllerUpdate) -> Option<MotorInput> {
        // The mutex is only locked when the scheduler is running, so this cannot really cause any
        // hang-ups
        let mut mutex = self.fc_mutex.lock().unwrap();
        mutex.update = Some(update);
        mutex.motor_input
    }

    fn scheduler_delta(&self) -> Duration {
        self.scheduler_delta
    }
}
