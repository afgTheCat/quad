use bevy::{
    prelude::{Res, Resource},
    time::Time,
};
use core::panic;
use flight_controller::MotorInput;
use simulator::{Drone, SimulationDebugInfo};
use std::{ops::Range, time::Duration};

#[derive(Clone)]
struct ReplayTimeStep {
    motor_input: MotorInput,
    replay_range: Range<Duration>,
}

struct ReplayTimeSteps {
    time_steps: Vec<ReplayTimeStep>,
    indx: usize,
}

impl ReplayTimeSteps {
    /// checks if the current corresponds
    fn dt_is_current_timestep(&self, dt: Duration) -> bool {
        let replay_range = &self.time_steps[self.indx].replay_range;
        replay_range.start < dt && replay_range.end >= dt
    }

    fn last_time_step_range_end(&self) -> Duration {
        self.time_steps.last().unwrap().replay_range.end
    }

    fn replay_state(&self, dt: Duration) -> ReplayState {
        if self.last_time_step_range_end() > dt {
            ReplayState::Running
        } else {
            ReplayState::Finished
        }
    }

    fn replay_time_step(&mut self, dt: Duration) -> Option<ReplayTimeStep> {
        if let ReplayState::Finished = self.replay_state(dt) {
            return None;
        }

        while !self.dt_is_current_timestep(dt) {
            self.indx += 1;
        }

        let current_reply_time_step = self.time_steps[self.indx].clone();
        Some(current_reply_time_step)
    }
}

#[derive(Resource)]
struct Replay {
    drone: Drone,
    time_steps: Option<ReplayTimeSteps>,
    time_accu: Duration,
    dt: Duration,
}

enum ReplayState {
    Running,
    Finished,
}

struct ReplayInfo {
    replay_state: ReplayState,
    sim_info: SimulationDebugInfo,
}

impl Replay {
    fn replay_delta(&mut self, dt: Duration) -> ReplayInfo {
        let Some(replay_steps) = &mut self.time_steps else {
            panic!()
        };

        while self.time_accu < dt {
            self.time_accu -= self.dt;
            match replay_steps.replay_time_step(dt) {
                Some(replay_state) => {
                    self.drone.set_motor_pwms(replay_state.motor_input);
                    self.drone.update(dt.as_secs_f64());
                }
                None => break,
            }
        }

        let sim_info = self.drone.debug_info();
        let replay_state = replay_steps.replay_state(dt);
        ReplayInfo {
            replay_state,
            sim_info,
        }
    }
}

pub fn replay_loop(mut simulation_replay: Res<Replay>, timer: Res<Time>) {}
