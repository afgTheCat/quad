use db::AscentDb2;
use std::{sync::Mutex, time::Duration};
// This is going to be the rc based flight controller after training
use res::{drone::DroneRc, input::FlightInput};

use crate::{FlightController, MotorInput};

pub struct ResController {
    model: Mutex<DroneRc>,
}

impl FlightController for ResController {
    // TODO: set the reservoir states to zeros
    fn init(&self) {}

    // TODO: set the reservoir states to zeros
    fn deinit(&self) {}

    // TODO: consider the delta time us
    fn update(
        &self,
        delta_time_us: u64,
        update: crate::FlightControllerUpdate,
    ) -> crate::MotorInput {
        let rc_input = update.to_rc_input();
        let input = FlightInput::new_from_rc_input(vec![vec![rc_input]]);
        let mut model = self.model.lock().unwrap();

        let pr = model.predict(Box::new(input));
        let motor_input_1 = f64::clamp(*pr.row(0).get(0).unwrap(), 0., 1.);
        let motor_input_2 = f64::clamp(*pr.row(0).get(1).unwrap(), 0., 1.);
        let motor_input_3 = f64::clamp(*pr.row(0).get(2).unwrap(), 0., 1.);
        let motor_input_4 = f64::clamp(*pr.row(0).get(3).unwrap(), 0., 1.);
        MotorInput {
            input: [motor_input_1, motor_input_2, motor_input_3, motor_input_4],
        }
    }

    fn scheduler_delta(&self) -> std::time::Duration {
        Duration::from_millis(5)
    }
}

impl ResController {
    fn from_db(db: AscentDb2, id: &str) -> Self {
        let drone_rc = DroneRc::read_from_db(id, &db).unwrap();
        Self {
            model: Mutex::new(drone_rc),
        }
    }
}

#[cfg(test)]
mod test {
    use db::{simulation::DBNewFlightLog, AscentDb2};
    use nalgebra::DMatrix;
    use res::{
        drone::DroneRc,
        input::{db_fl_to_rc_input, FlightInput},
        representation::RepresentationType,
        ridge::RidgeRegression,
    };

    #[test]
    fn train_thing() {
        let db = AscentDb2::new("/home/gabor/ascent/quad/data.sqlite");
        let flight_log = db.get_simulation_data(&"86a9dd7f-f730-40cb-8fe8-e5a076867545");
        let mut drone_rc = DroneRc::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );
        drone_rc.esn.set_input_weights(18);
        let input = FlightInput::new_from_db_fl_log(vec![flight_log.clone()]);
        let data_points =
            DMatrix::from_columns(&flight_log.iter().map(db_fl_to_rc_input).collect::<Vec<_>>())
                .transpose();

        drone_rc.fit(Box::new(input.clone()), data_points);
        drone_rc.save_model_to_db("only_up".into(), &db);

        let mut new_rc_mode = DroneRc::read_from_db("only_up", &db).unwrap();
        let predicted_points = new_rc_mode.predict(Box::new(input));
        println!("{:#?}", predicted_points);

        let mut rec_flight_logs = vec![];
        for (i, out) in predicted_points.row_iter().enumerate() {
            let fl = DBNewFlightLog {
                simulation_id: "rec".into(),
                start_seconds: flight_log[i].start_seconds,
                end_seconds: flight_log[i].end_seconds,
                motor_input_1: *out.get(0).unwrap(),
                motor_input_2: *out.get(1).unwrap(),
                motor_input_3: *out.get(2).unwrap(),
                motor_input_4: *out.get(3).unwrap(),
                battery_voltage_sag: flight_log[i].battery_voltage_sag,
                battery_voltage: flight_log[i].battery_voltage,
                amperage: flight_log[i].amperage,
                mah_drawn: flight_log[i].mah_drawn,
                cell_count: flight_log[i].cell_count,
                rot_quat_x: flight_log[i].rot_quat_x,
                rot_quat_y: flight_log[i].rot_quat_y,
                rot_quat_z: flight_log[i].rot_quat_z,
                rot_quat_w: flight_log[i].rot_quat_w,
                linear_acceleration_x: flight_log[i].linear_acceleration_x,
                linear_acceleration_y: flight_log[i].linear_acceleration_y,
                linear_acceleration_z: flight_log[i].linear_acceleration_z,
                angular_velocity_x: flight_log[i].angular_velocity_x,
                angular_velocity_y: flight_log[i].angular_velocity_y,
                angular_velocity_z: flight_log[i].angular_velocity_z,
                throttle: flight_log[i].throttle,
                roll: flight_log[i].roll,
                pitch: flight_log[i].pitch,
                yaw: flight_log[i].yaw,
            };
            rec_flight_logs.push(fl);
        }

        db.write_flight_logs("rec", &rec_flight_logs);
    }

    #[test]
    fn train_on_many() {
        let db = AscentDb2::new("/home/gabor/ascent/quad/data.sqlite");
        let simulation_id = db.get_all_simulation_ids();
        let tr_ids = simulation_id
            .iter()
            .filter(|id| id.starts_with("ds_id_1_tr"))
            .cloned()
            .collect::<Vec<_>>();
        let te_ids = simulation_id
            .iter()
            .filter(|id| id.starts_with("ds_id_1_te"))
            .cloned()
            .collect::<Vec<_>>();

        let mut drone_rc = DroneRc::new(
            500,
            0.3,
            0.99,
            0.2,
            RepresentationType::Output(1.),
            RidgeRegression::new(1.),
        );
        drone_rc.esn.set_input_weights(18);
        for tr_id in tr_ids {
            let flight_log = db.get_simulation_data(&tr_id);
            let input = FlightInput::new_from_db_fl_log(vec![flight_log.clone()]);
            let data_points = DMatrix::from_columns(
                &flight_log.iter().map(db_fl_to_rc_input).collect::<Vec<_>>(),
            )
            .transpose();
            drone_rc.fit(Box::new(input.clone()), data_points);
        }
    }
}
