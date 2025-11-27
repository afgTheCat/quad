use crate::{ntb_mat3, ntb_vec3, Context};
use bevy::{
    color::palettes::css::RED,
    math::{Quat, Vec3},
    prelude::{Commands, Deref, DerefMut, Gizmos, Query, Res, ResMut, Resource, Transform},
    scene::SceneRoot,
    time::Time,
};
use bevy_panorbit_camera::PanOrbitCamera;
use nalgebra::Vector3;
use simulator::Replayer;

#[derive(Resource, Deref, DerefMut)]
pub struct Replay(pub Replayer);

pub fn replay_loop(
    mut gizmos: Gizmos,
    timer: Res<Time>,
    mut replay: ResMut<Replay>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut scene_query: Query<(&mut Transform, &SceneRoot)>,
) {
    let (mut tranform, _) = scene_query.single_mut().unwrap();
    let mut camera = camera_query.single_mut().unwrap();

    let debug_info = replay.replay_delta(timer.delta());
    let drone_translation = ntb_vec3(debug_info.position);
    let drone_rotation = Quat::from_mat3(&ntb_mat3(debug_info.rotation));
    tranform.translation = drone_translation;
    tranform.rotation = drone_rotation;

    let down_dir = debug_info.rotation * Vector3::new(0., -1., -0.);
    let current_frame = &replay.drone.current_frame;
    for motor_index in 0..4 {
        let motor_pos = drone_translation
            + ntb_vec3(debug_info.rotation * current_frame.rotors_state.0[motor_index].motor_pos);
        gizmos.arrow(
            motor_pos,
            motor_pos
                + ntb_vec3(down_dir * current_frame.rotors_state.0[motor_index].effective_thrust),
            RED,
        );
    }
    camera.target_focus = drone_translation;
}

pub fn enter_replay(mut context: ResMut<Context>, mut commands: Commands) {
    let replay = context.try_load_replay().unwrap();
    commands.insert_resource(Replay(replay));
}

pub fn exit_replay(
    mut scene_query: Query<(&mut Transform, &SceneRoot)>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut commands: Commands,
) {
    let (mut transform, _) = scene_query.single_mut().unwrap();
    let mut camera = camera_query.single_mut().unwrap();
    transform.rotation = Quat::IDENTITY;
    transform.translation = Vec3::ZERO;
    camera.target_focus = transform.translation;

    commands.remove_resource::<Replay>();
}
