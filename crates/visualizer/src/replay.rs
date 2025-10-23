use std::time::Duration;

use crate::{ntb_mat3, ntb_vec3, ui::menu::SelectionConfig, Loader, VisualizerData};
use bevy::{
    asset::Handle,
    color::palettes::css::RED,
    math::{Quat, Vec3},
    prelude::{Commands, Deref, DerefMut, Gizmos, Query, Res, ResMut, Resource, Transform},
    scene::Scene,
    time::Time,
};
use bevy_panorbit_camera::PanOrbitCamera;
use db2::DataAccessLayer;
use nalgebra::Vector3;
use simulator::Replayer;

#[derive(Resource, Deref, DerefMut)]
pub struct Replay(pub Replayer);

pub fn replay_loop(
    mut gizmos: Gizmos,
    timer: Res<Time>,
    mut replay: ResMut<Replay>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
) {
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();

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

pub fn enter_replay(
    sim_data: ResMut<VisualizerData>,
    db: Res<Loader>,
    mut commands: Commands,
    loader: Res<Loader>,
) {
    let SelectionConfig::Replay {
        replay_id: Some(simulation_id),
        ..
    } = &sim_data.selection_config
    else {
        unreachable!()
    };

    let sim_logs = db.load_replay(simulation_id);
    let drone = loader.load_drone(1);
    let replay = Replay(Replayer {
        drone,
        time: Duration::new(0, 0),
        time_accu: Duration::new(0, 0),
        time_steps: sim_logs,
        replay_index: 0,
        dt: Duration::from_nanos(5000), // TODO: update this
    });
    commands.insert_resource(replay);
}

pub fn exit_replay(
    mut scene_query: Query<(&mut Transform, &Handle<Scene>)>,
    mut camera_query: Query<&mut PanOrbitCamera>,
    mut commands: Commands,
) {
    let (mut tranform, _) = scene_query.single_mut();
    let mut camera = camera_query.single_mut();
    tranform.rotation = Quat::IDENTITY;
    tranform.translation = Vec3::ZERO;
    camera.target_focus = tranform.translation;

    commands.remove_resource::<Replay>();
}
