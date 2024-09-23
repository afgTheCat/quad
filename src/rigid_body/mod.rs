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
    // angular_velocity: Vec3,
    inertia_tensor: Mat3,
    angular_momentum: Vec3,
}

// Euler for now
impl RigidBody {
    fn euler_integrate(&mut self, transform: &mut Transform, dt: &Duration) {
        // linear part
        transform.translation += self.linear_velocity * dt.as_secs_f32();

        // Step 1: Transform angular momentum to body space
        let angular_momentum_body = transform.rotation.inverse() * self.angular_momentum;

        // Step 2: Apply inertia tensor in body space to get angular velocity in body space
        let angular_velocity_body = self.inertia_tensor.inverse() * angular_momentum_body;

        // println!("{:?}", angular_velocity_body);

        // Step 3: Transform angular velocity back to world space
        let angular_velocity_world = transform.rotation * angular_velocity_body;

        // Step 4: Quaternion representation of angular velocity in world space
        let q_omega = Quat::from_xyzw(
            angular_velocity_world[0],
            angular_velocity_world[1],
            angular_velocity_world[2],
            0.,
        );

        // Step 5: Update rotation using quaternion integration
        let dq_omega = q_omega * dt.as_secs_f32() * 0.5;
        let d_rotation = transform.rotation * dq_omega;
        transform.rotation = (transform.rotation + d_rotation).normalize();
    }
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            mass: 1.,
            linear_velocity: Vec3::new(1.0, 0.0, 0.),
            inertia_tensor: Mat3::from_cols(
                Vec3::new(1., 0., 0.),
                Vec3::new(0., 1., 0.),
                Vec3::new(0., 0., 1.),
            ),
            // inertia_tensor: Mat3::from_cols(
            //     Vec3::new(2. / 3., -0.25, -0.25),
            //     Vec3::new(-0.25, 2. / 3., -0.25),
            //     Vec3::new(-0.25, -0.25, 2. / 3.),
            // ),
            angular_momentum: Vec3::new(0., 1., 0.),
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
        // transform: Transform::from_xyz(0.0, 0.5, 0.0),
        transform: Transform {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        ..default()
    };
    commands.spawn((RigidBody::default(), cube_mesh, simulation_context));
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
