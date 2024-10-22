use bevy::{
    math::{DMat3, DVec3, DVec4, Mat3, Quat, Vec3},
    prelude::{Query, Res, Transform},
    time::Time,
};
use bevy_panorbit_camera::PanOrbitCamera;
use flight_controller::{Channels, FlightControllerUpdate};
use nalgebra::{Matrix3, Rotation3, Vector3, Vector4};

use crate::{
    ui::UiSimulationInfo, Controller, DroneComponent, FlightControllerComponent, SimContext,
};

fn ntb_vec3(vec: Vector3<f64>) -> Vec3 {
    Vec3::new(vec[0] as f32, vec[1] as f32, vec[2] as f32)
}

fn ntb_dvec3(vec: Vector3<f64>) -> DVec3 {
    DVec3::new(vec[0], vec[1], vec[2])
}

fn ntb_dvec4(vec: Vector4<f64>) -> DVec4 {
    DVec4::new(vec[0], vec[1], vec[2], vec[3])
}

fn ntb_mat3(matrix: Rotation3<f64>) -> Mat3 {
    Mat3::from_cols(
        Vec3::from_slice(
            &matrix
                .matrix()
                .column(0)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
        Vec3::from_slice(
            &matrix
                .matrix()
                .column(1)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
        Vec3::from_slice(
            &matrix
                .matrix()
                .column(2)
                .iter()
                .map(|x| *x as f32)
                .collect::<Vec<_>>(),
        ),
    )
}

fn ntb_dmat3(matrix: Rotation3<f64>) -> DMat3 {
    DMat3::from_cols(
        DVec3::from_slice(
            &matrix
                .matrix()
                .column(0)
                .iter()
                .map(|x| *x)
                .collect::<Vec<_>>(),
        ),
        DVec3::from_slice(
            &matrix
                .matrix()
                .column(1)
                .iter()
                .map(|x| *x)
                .collect::<Vec<_>>(),
        ),
        DVec3::from_slice(
            &matrix
                .matrix()
                .column(2)
                .iter()
                .map(|x| *x)
                .collect::<Vec<_>>(),
        ),
    )
}

pub fn debug_drone(
    mut drone_query: Query<(
        &mut Transform,
        &mut DroneComponent,
        &mut FlightControllerComponent,
        &mut UiSimulationInfo,
        &mut SimContext,
        &mut Controller,
    )>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    timer: Res<Time>,
) {
    let (
        mut drone_transform,
        mut drone,
        flight_controller,
        mut ui_simulation_info,
        mut sim_context,
        controller,
    ) = drone_query.single_mut();
    let mut camera = camera_query.single_mut();

    sim_context.time_accu += timer.delta();
    while sim_context.step_context() {
        drone.update_gyro(sim_context.dt.as_secs_f64());
        drone.update_physics(sim_context.dt.as_secs_f64(), sim_context.ambient_temp);
        let battery_update = drone.battery_update();
        let gyro_update = drone.gyro_update();
        let channels = controller.channels();
        let fc_input = FlightControllerUpdate {
            gyro_update,
            battery_update,
            channels,
        };
        let pwms = flight_controller.update(fc_input);
        if let Some(pwms) = pwms {
            drone.set_motor_pwms(pwms);
        }
    }

    let drone_translation = ntb_vec3(drone.0.rigid_body.position);
    drone_transform.translation = drone_translation;
    drone_transform.rotation = Quat::from_mat3(&ntb_mat3(drone.0.rigid_body.rotation));
    ui_simulation_info.update_state(
        ntb_dmat3(drone.0.rigid_body.rotation),
        ntb_dvec3(drone.0.rigid_body.position),
        ntb_dvec3(drone.0.rigid_body.linear_velocity),
        ntb_dvec3(drone.0.rigid_body.acceleration),
        ntb_dvec3(drone.0.rigid_body.angular_velocity),
        ntb_dvec4(drone.0.thrusts()),
        ntb_dvec4(drone.0.rpms()),
        ntb_dvec4(drone.0.motor_pwms()),
        drone.0.battery.state.bat_voltage,
        drone.0.battery.state.bat_voltage_sag,
    );
    camera.target_focus = drone_translation;
}
