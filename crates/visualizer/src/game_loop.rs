use crate::{ui::UiInfo, Controller, DroneComponent, FlightControllerComponent, SimContext};
use bevy::{
    color::palettes::css::RED,
    math::{Mat3, Quat, Vec3},
    prelude::{Gizmos, Query, Res, Transform},
    time::Time,
};
use bevy_panorbit_camera::PanOrbitCamera;
use flight_controller::FlightControllerUpdate;
use nalgebra::{Rotation3, Vector3};

pub fn ntb_vec3(vec: Vector3<f64>) -> Vec3 {
    Vec3::new(vec[0] as f32, vec[1] as f32, vec[2] as f32)
}

pub fn ntb_mat3(matrix: Rotation3<f64>) -> Mat3 {
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

pub fn debug_drone(
    mut gizmos: Gizmos,
    mut drone_query: Query<(
        &mut Transform,
        &mut DroneComponent,
        &mut FlightControllerComponent,
        &mut UiInfo,
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
        mut ui_info,
        mut sim_context,
        controller,
    ) = drone_query.single_mut();
    let mut camera = camera_query.single_mut();

    sim_context.time_accu += timer.delta();
    while sim_context.step_context() {
        let drone_state = drone.update(sim_context.dt.as_secs_f64(), sim_context.ambient_temp);
        let motor_input = flight_controller.update(FlightControllerUpdate {
            battery_update: drone_state.battery_update,
            gyro_update: drone_state.gyro_update,
            channels: controller.to_channels(),
        });
        if let Some(pwms) = motor_input {
            drone.set_motor_pwms(pwms);
        }
    }
    let sim_debug_info = drone.debug_info();

    // update drone translation and rotation
    let drone_translation = ntb_vec3(sim_debug_info.position);
    drone_transform.translation = drone_translation;
    drone_transform.rotation = Quat::from_mat3(&ntb_mat3(sim_debug_info.rotation));

    // Show debug thrusts
    let down_dir = sim_debug_info.rotation * Vector3::new(0., -1., -0.);
    for motor_index in 0..4 {
        let motor_pos = drone_translation
            + ntb_vec3(sim_debug_info.rotation * drone.0.arms[motor_index].motor_pos());
        gizmos.arrow(
            motor_pos,
            motor_pos + ntb_vec3(down_dir * drone.0.arms[motor_index].thrust()),
            RED,
        );
    }

    // Update debug ui info
    *ui_info = UiInfo(sim_debug_info);

    // Update camera translation
    // TODO: we should also have an option to lock the orientation to be always behind the drone
    camera.target_focus = drone_translation;
}
