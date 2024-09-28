use std::time::Duration;

use bevy::{
    asset::Assets,
    color::Color,
    math::{Mat3, Quat, Vec3},
    pbr::{PbrBundle, StandardMaterial},
    prelude::{default, Commands, Component, Cuboid, Mesh, Query, Res, ResMut, Transform},
    time::Time,
};

#[derive(Component)]
pub struct RigidBody {
    mass: f32,
    pub sides: Vec3,
    pub linear_velocity: Vec3,
    pub inv_inertia_tensor: Mat3,
    pub angular_momentum: Vec3,
}

fn inv_rectangular_cuboid_inertia_matrix(sides: Vec3) -> Mat3 {
    let Vec3 { x, y, z } = sides;
    let diag = Vec3::new(
        y.powi(2) + z.powi(2),
        x.powi(2) + z.powi(2),
        x.powi(2) + y.powi(2),
    );
    let v = x * y * z;
    1. / (v * Mat3::from_diagonal(diag) / 12.)
}

// Euler for now
impl RigidBody {
    fn euler_integrate(&mut self, transform: &mut Transform, dt: &Duration) {
        // println!("{:?}", Mat3::from_quat(transform.rotation));
        // Linear part (position update)
        transform.translation += self.linear_velocity * dt.as_secs_f32();

        let r = Mat3::from_quat(transform.rotation);
        let angular_velocity_world =
            r * self.inv_inertia_tensor * r.inverse() * self.angular_momentum;

        let q_omega = Quat::from_xyzw(
            angular_velocity_world[0],
            angular_velocity_world[1],
            angular_velocity_world[2],
            0.,
        );
        let dq_omega = q_omega * dt.as_secs_f32() * 0.5;
        let d_rotation = transform.rotation * dq_omega;
        transform.rotation = (transform.rotation + d_rotation).normalize();
        // println!("{:?}", Mat3::from_quat(transform.rotation));
    }
}

#[derive(Component)]
pub struct SimulationContext {
    pub dt: Duration,
    pub simulation_running: bool,
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
            simulation_running: true,
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
    let sides = Vec3::new(1.4, 0.2, 2.1);
    let simulation_context = SimulationContext::default();
    let cube_mesh = PbrBundle {
        mesh: meshes.add(Cuboid::from_size(sides)),
        material: materials.add(Color::srgb_u8(124, 144, 255)),
        transform: Transform {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        ..default()
    };
    commands.spawn((
        RigidBody {
            inv_inertia_tensor: inv_rectangular_cuboid_inertia_matrix(sides),
            mass: 1.,
            sides,
            angular_momentum: Vec3::new(1., 0.8, 0.9),
            linear_velocity: Vec3::ZERO,
        },
        cube_mesh,
        simulation_context,
    ));
}

pub fn main_qube(
    mut query: Query<(&mut Transform, &mut RigidBody, &mut SimulationContext)>,
    timer: Res<Time>,
) {
    let (mut transform, mut rigid_body, mut simulation_context) = query.single_mut();
    if simulation_context.simulation_running {
        simulation_context.time_accu += timer.delta();
        while simulation_context.step_context() {
            rigid_body.euler_integrate(&mut transform, &simulation_context.dt);
        }
    }
}
