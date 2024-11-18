use crate::{ui::UiInfo, Controller, DroneComponent, FlightControllerComponent, SimContext};
use bevy::{
    color::palettes::css::{ORANGE, PINK, RED},
    math::{DMat3, DVec3, DVec4, Mat3, Quat, Vec3},
    prelude::{Gizmos, Query, Res, Transform},
    time::Time,
};
use bevy_panorbit_camera::PanOrbitCamera;
use flight_controller::FlightControllerUpdate;
use nalgebra::{Rotation3, Vector3, Vector4};

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
        let pwms = flight_controller.update(FlightControllerUpdate {
            battery_update: drone_state.battery_update,
            gyro_update: drone_state.gyro_update,
            channels: controller.to_channels(),
        });
        if let Some(pwms) = pwms {
            drone.set_motor_pwms(pwms);
        }
    }
    let sim_debug_info = drone.debug_info();

    let drone_translation = ntb_vec3(sim_debug_info.position);
    drone_transform.translation = drone_translation;
    drone_transform.rotation = Quat::from_mat3(&ntb_mat3(sim_debug_info.rotation));

    let down_dir = sim_debug_info.rotation * Vector3::new(0., -1., -0.);

    let motor0pos =
        drone_translation + ntb_vec3(sim_debug_info.rotation * drone.0.arms[0].motor_pos());
    let motor1pos =
        drone_translation + ntb_vec3(sim_debug_info.rotation * drone.0.arms[1].motor_pos());
    let motor2pos =
        drone_translation + ntb_vec3(sim_debug_info.rotation * drone.0.arms[2].motor_pos());
    let motor3pos =
        drone_translation + ntb_vec3(sim_debug_info.rotation * drone.0.arms[3].motor_pos());

    *ui_info = UiInfo(sim_debug_info);

    gizmos.arrow(
        motor0pos,
        motor0pos + ntb_vec3(down_dir * drone.0.arms[0].thrust()),
        RED,
    );
    gizmos.arrow(
        motor1pos,
        motor1pos + ntb_vec3(down_dir * drone.0.arms[1].thrust()),
        RED,
    );
    gizmos.arrow(
        motor2pos,
        motor2pos + ntb_vec3(down_dir * drone.0.arms[2].thrust()),
        RED,
    );
    gizmos.arrow(
        motor3pos,
        motor3pos + ntb_vec3(down_dir * drone.0.arms[3].thrust()),
        RED,
    );

    gizmos.arrow(Vec3::ZERO, Vec3::new(1., 0., 0.), ORANGE);
    gizmos.arrow(Vec3::ZERO, Vec3::new(0., 0., 1.), PINK);

    camera.target_focus = drone_translation;
}
