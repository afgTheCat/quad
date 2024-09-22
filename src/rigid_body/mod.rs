use std::time::Duration;

use bevy::{
    asset::Assets,
    color::Color,
    math::Vec3,
    pbr::{PbrBundle, StandardMaterial},
    prelude::{default, Commands, Component, Cuboid, Mesh, Query, Res, ResMut, Transform},
    time::Time,
};

#[derive(Component)]
pub struct RigidBody {
    mass: f32,
    linear_veolcity: Vec3,
}

// Euler for now
impl RigidBody {
    fn euler_integrate(&mut self, transform: &mut Transform, dt: &Duration) {
        transform.translation += self.linear_veolcity * dt.as_secs_f32();
    }
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            mass: 1.,
            linear_veolcity: Vec3::new(1.0, 0.0, 0.),
        }
    }
}

#[derive(Component)]
pub struct SimulationContext {
    pub dt: Duration,
    pub time_accu: Duration, // the accumulated time between two steps + the correction from the
                             // previus step
}

impl SimulationContext {
    fn step_context(&mut self) -> bool {
        if self.time_accu > self.dt {
            self.time_accu -= self.dt;
            true
        } else {
            false
        }
    }
}

impl Default for SimulationContext {
    fn default() -> Self {
        Self {
            dt: Duration::from_millis(1),
            time_accu: Duration::default(),
        }
    }
}

pub fn setup_rigid_body_context(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let simulation_context = SimulationContext::default();
    let cube_mesh = PbrBundle {
        mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
        material: materials.add(Color::srgb_u8(124, 144, 255)),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        ..default()
    };
    commands.spawn((RigidBody::default(), cube_mesh, simulation_context));
}

pub fn debug_move_cube(
    mut query: Query<(&mut Transform, &mut RigidBody, &mut SimulationContext)>,
    timer: Res<Time>,
) {
    let (mut transform, mut rigid_body, mut simulation_context) = query.single_mut();
    simulation_context.time_accu += timer.delta();
    while simulation_context.step_context() {
        rigid_body.euler_integrate(&mut transform, &simulation_context.dt);
    }
}
