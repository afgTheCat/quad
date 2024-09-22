use bevy::{
    asset::Assets,
    color::Color,
    math::Vec3,
    pbr::{PbrBundle, StandardMaterial},
    prelude::{default, Commands, Component, Cuboid, Mesh, Query, Res, ResMut, Transform},
    time::Time,
};

#[derive(Clone, Component)]
pub struct RigidBody {
    mass: f32,
    linear_veolcity: Vec3,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            mass: 1.,
            linear_veolcity: Vec3::default(),
        }
    }
}

#[derive(Component)]
pub struct SimulationContext {
    dt: f32,
}

pub fn setup_rigid_body_context(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_mesh = PbrBundle {
        mesh: meshes.add(Cuboid::new(1.0, 1.0, 1.0)),
        material: materials.add(Color::srgb_u8(124, 144, 255)),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        ..default()
    };
    commands.spawn((RigidBody::default(), cube_mesh));
}

pub fn debug_move_cube(mut query: Query<(&mut Transform, &mut RigidBody)>, timer: Res<Time>) {
    let (mut transform, mut rigid_body) = query.single_mut();
    let forward = transform.forward();
    transform.translation += forward * timer.delta_seconds();
    println!("ok")
}
