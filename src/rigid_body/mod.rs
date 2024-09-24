use std::time::Duration;

use bevy::{
    asset::Assets,
    color::Color,
    math::{Mat3, Quat, Vec3, VectorSpace},
    pbr::{PbrBundle, StandardMaterial},
    prelude::{default, Commands, Component, Cuboid, Mesh, Query, Res, ResMut, Transform},
    time::Time,
};

#[derive(Component)]
pub struct RigidBody {
    mass: f32,
    linear_velocity: Vec3,
    inertia_tensor: Mat3,
    angular_momentum: Vec3,
}

fn rectangular_cuboid_inertia_matrix(a: f32, b: f32, c: f32) -> Mat3 {
    let v = a * b * c;
    v * Mat3::from_diagonal(Vec3::new(
        b.powi(2) + c.powi(2) / 12.,
        b.powi(2) + c.powi(2) / 12.,
        b.powi(2) + c.powi(2) / 12.,
    ))
}

// Euler for now
impl RigidBody {
    fn euler_integrate(&mut self, transform: &mut Transform, dt: &Duration) {
        // linear part
        transform.translation += self.linear_velocity * dt.as_secs_f32();

        let r = Mat3::from_quat(transform.rotation.inverse());
        let omega = r * self.inertia_tensor.inverse() * r.transpose() * self.angular_momentum;

        // Quaternion representation of angular velocity
        let q_omega = Quat::from_xyzw(omega[0], omega[1], omega[2], 0.);

        // Update rotation
        let dq_omega = q_omega * dt.as_secs_f32() * 0.5;
        let d_rotation = transform.rotation * dq_omega;
        transform.rotation = (transform.rotation + d_rotation).normalize();
    }
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            mass: 1.,
            linear_velocity: Vec3::new(0.0, 0.0, 0.),
            inertia_tensor: Mat3::from_cols(
                Vec3::new(1., 0., 0.),
                Vec3::new(0., 1., 0.),
                Vec3::new(0., 0., 1.),
            ),
            angular_momentum: Vec3::new(40., 4., 1.),
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
    let (a, b, c) = (3.0, 0.3, 6.);
    let simulation_context = SimulationContext::default();
    let cube_mesh = PbrBundle {
        mesh: meshes.add(Cuboid::new(a, b, c)),
        material: materials.add(Color::srgb_u8(124, 144, 255)),
        // transform: Transform::from_xyz(0.0, 0.5, 0.0),
        transform: Transform {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        ..default()
    };
    commands.spawn((
        RigidBody {
            inertia_tensor: rectangular_cuboid_inertia_matrix(a, b, c),
            ..RigidBody::default()
        },
        cube_mesh,
        simulation_context,
    ));
}

pub fn debug_move_cube(
    mut query: Query<(&mut Transform, &mut RigidBody, &mut SimulationContext)>,
    timer: Res<Time>,
) {
    if timer.elapsed_seconds() < 2. {
        return;
    }
    let (mut transform, mut rigid_body, mut simulation_context) = query.single_mut();
    simulation_context.time_accu += timer.delta();
    while simulation_context.step_context() {
        rigid_body.euler_integrate(&mut transform, &simulation_context.dt);
    }
}
