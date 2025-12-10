use drone::Drone;
use flight_controller::{Channels, FlightController, FlightControllerUpdate, MotorInput};
use loggers::SnapShot;
use nalgebra::{DMatrix, DVector};
use res::{
    esn::Esn,
    input::{FlightInput, RcInput},
    representation::{
        AllStatesForSingleEp, BufferedStatesForSingleEp, LastStateRepr, OutputRepr, Representation,
        RepresentationType,
    },
};
use ridge::RidgeRegression;
use serde::{Deserialize, Serialize};
use std::time::Duration;

#[derive(Serialize, Deserialize, Clone)]
pub struct DroneRc {
    pub esn: Esn,
    pub representation: Representation,
    // Could be changed to ElasticNetWrapper
    pub readout: RidgeRegression,
}

impl DroneRc {
    pub fn new(
        n_internal_units: usize,
        connectivity: f64,
        spectral_radius: f64,
        input_scaling: f64,
        representation: RepresentationType,
        readout: RidgeRegression,
    ) -> Self {
        let esn = Esn::new(
            n_internal_units,
            connectivity,
            spectral_radius,
            input_scaling,
        );
        let representation = match representation {
            RepresentationType::LastState => Representation::LastState(LastStateRepr::default()),
            RepresentationType::Output(alpha) => Representation::Output(OutputRepr::new(alpha)),
            RepresentationType::AllStates => {
                Representation::AllStateForSingle(AllStatesForSingleEp)
            }
            RepresentationType::BufferedStates(states) => {
                Representation::BufferedStates(BufferedStatesForSingleEp(states))
            }
        };
        Self {
            esn,
            representation,
            readout,
        }
    }

    // this is the same, maybe it works maybe it does not
    pub fn fit(&mut self, input: Box<dyn RcInput>, motor_inputs: DMatrix<f64>) {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.fit_multiple_svd(input_repr, &motor_inputs);
    }

    pub fn predict(&self, input: Box<dyn RcInput>) -> DMatrix<f64> {
        let res_states = self.esn.compute_state_matricies(&input);
        let input_repr = self.representation.repr(input, res_states);
        self.readout.predict(input_repr)
    }
}

// describes how to convert a flight controller update to a reservoir input
pub fn flight_controller_update_to_reservoir_input(update: FlightControllerUpdate) -> DVector<f64> {
    let Channels {
        throttle,
        roll,
        pitch,
        yaw,
    } = update.channels;
    let [rot_w, rot_x, rot_y, rot_z] = update.gyro_update.rotation;
    DVector::from_row_slice(&[throttle, roll, yaw, pitch, rot_w, rot_x, rot_y, rot_z])
}

pub fn snapshot_to_reservoir_input(snapshot: &SnapShot, reference_drone: &Drone) -> DVector<f64> {
    DVector::from_row_slice(&[
        // snapshot.battery_update.bat_voltage_sag
        //     / (drone.current_frame.battery_state.bat_voltage_sag
        //         * drone.battery_model.quad_bat_cell_count as f64),
        // snapshot.battery_update.bat_voltage
        //     / (drone.current_frame.battery_state.bat_voltage
        //         * drone.battery_model.quad_bat_cell_count as f64),
        // snapshot.battery_update.amperage / 60., // TODO: should be calculated
        // snapshot.battery_update.m_ah_drawn / drone.battery_model.quad_bat_capacity,
        // snapshot.gyro_update.linear_acc[0] / 40.,
        // snapshot.gyro_update.linear_acc[1] / 40.,
        // snapshot.gyro_update.linear_acc[2] / 40.,
        // snapshot.gyro_update.angular_velocity[0] / 20.,
        // snapshot.gyro_update.angular_velocity[1] / 20.,
        // snapshot.gyro_update.angular_velocity[2] / 20.,
        snapshot.channels.throttle,
        snapshot.channels.roll,
        snapshot.channels.yaw,
        snapshot.channels.pitch,
        snapshot.gyro_update.rotation[0],
        snapshot.gyro_update.rotation[1],
        snapshot.gyro_update.rotation[2],
        snapshot.gyro_update.rotation[3],
    ])
}

impl FlightController for DroneRc {
    fn init(&self) {}

    fn update(&self, _delta_time: f64, update: FlightControllerUpdate) -> MotorInput {
        let rc_input = flight_controller_update_to_reservoir_input(update);
        let input = FlightInput::new_from_rc_input(vec![vec![rc_input]]);
        let pr = self.predict(Box::new(input));
        let motor_input_1 = f64::clamp(*pr.row(0).get(0).unwrap(), 0., 1.);
        let motor_input_2 = f64::clamp(*pr.row(0).get(1).unwrap(), 0., 1.);
        let motor_input_3 = f64::clamp(*pr.row(0).get(2).unwrap(), 0., 1.);
        let motor_input_4 = f64::clamp(*pr.row(0).get(3).unwrap(), 0., 1.);
        MotorInput {
            input: [motor_input_1, motor_input_2, motor_input_3, motor_input_4],
        }
    }

    fn scheduler_delta(&self) -> Duration {
        Duration::from_millis(5)
    }
}
