use std::time::Duration;

use bevy::{
    asset::{Assets, Handle},
    color::palettes::css::{BLUE, RED, YELLOW},
    math::{DMat3, DVec3, Mat3, Quat, Vec3},
    prelude::{Component, Cuboid, Gizmos, Mesh, Query, Res, ResMut, Transform},
    time::Time,
};

// This is the
#[derive(Component)]
pub struct CubeRigidBody {
    pub density: f32,
    pub sides: Vec3,
    pub linear_velocity: Vec3,
    pub inv_inertia_tensor: DMat3,
    pub angular_momentum: DVec3,
    pub mesh_handle: Handle<Mesh>,
    pub rotation: DMat3,
}

pub fn inertia_cuboid_diag(sides: Vec3) -> DVec3 {
    let DVec3 { x, y, z } = sides.into();
    let v = x * y * z;
    (1. / 12.)
        * v
        * DVec3::new(
            y.powi(2) + z.powi(2),
            x.powi(2) + z.powi(2),
            x.powi(2) + y.powi(2),
        )
}

pub fn cube_from_inertia(inertia: DVec3) -> DVec3 {
    let w1 = (inertia[0] + inertia[1] - inertia[2]) * 6.;
    let w2 = (inertia[0] + inertia[2] - inertia[1]) * 6.;
    let w3 = (inertia[1] + inertia[2] - inertia[0]) * 6.;
    let a = w1.powf(2. / 5.) / (w2 * w3).powf(1. / 10.);
    let b = w2.powf(2. / 5.) / (w1 * w3).powf(1. / 10.);
    let c = w3.powf(2. / 5.) / (w1 * w2).powf(1. / 10.);
    DVec3::new(a, b, c)
}

pub fn inv_rectangular_cuboid_inertia_matrix(sides: Vec3) -> DMat3 {
    let inertia = inertia_cuboid_diag(sides);
    let diag = 1. / inertia;
    DMat3::from_diagonal(diag)
}

fn cross_product_matrix(v: DVec3) -> DMat3 {
    DMat3 {
        x_axis: DVec3::new(0., -v[2], v[1]),
        y_axis: DVec3::new(v[2], 0., -v[0]),
        z_axis: DVec3::new(-v[1], v[0], 0.),
    }
}

pub fn angular_velocity(
    transform: &Transform,
    inv_inertia_tensor: DMat3,
    angular_momentum: DVec3,
) -> DVec3 {
    let r = DMat3::from_quat(transform.rotation.as_dquat());
    r * inv_inertia_tensor * r.inverse() * angular_momentum
}

// Euler for now
impl CubeRigidBody {
    pub fn mass(&self) -> f32 {
        self.density * self.sides[0] * self.sides[1] * self.sides[2]
    }

    pub fn set_rigid_body_props(
        &mut self,
        sides: Vec3,
        angular_momentum: DVec3,
        mut meshes: ResMut<Assets<Mesh>>,
    ) {
        self.sides = sides;
        self.angular_momentum = angular_momentum;
        self.inv_inertia_tensor = inv_rectangular_cuboid_inertia_matrix(sides);
        if let Some(mesh) = meshes.get_mut(&self.mesh_handle) {
            *mesh = Mesh::from(Cuboid::from_size(sides))
        }
    }

    pub fn inertia_diag(&self) -> DVec3 {
        DVec3::new(
            1. / self.inv_inertia_tensor.row(0)[0],
            1. / self.inv_inertia_tensor.row(1)[1],
            1. / self.inv_inertia_tensor.row(2)[2],
        )
    }

    fn euler_integrate(&mut self, dt: &Duration, dialation: f64) {
        let dt_f64 = dt.as_secs_f64() * dialation;
        // transform.translation += self.linear_velocity * dt_f32;

        let angular_velocity_world = self.rotation
            * self.inv_inertia_tensor
            * self.rotation.transpose()
            * self.angular_momentum;
        self.rotation += dt_f64 * cross_product_matrix(angular_velocity_world) * self.rotation;
    }
}

#[derive(Component)]
pub struct SimulationContext {
    pub dt: Duration,
    pub simulation_running: bool,
    pub time_accu: Duration, // the accumulated time between two steps + the correction from the
    // previus step
    pub dialation: f64,
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
            simulation_running: false,
            dt: Duration::from_nanos(100),
            time_accu: Duration::default(),
            dialation: 1.,
        }
    }
}

pub fn rigid_body(
    mut gizmos: Gizmos,
    mut query: Query<(&mut Transform, &mut CubeRigidBody, &mut SimulationContext)>,
    timer: Res<Time>,
) {
    let (mut transform, mut rigid_body, mut simulation_context) = query.single_mut();
    if simulation_context.simulation_running {
        simulation_context.time_accu += timer.delta();
        while simulation_context.step_context() {
            rigid_body.euler_integrate(&simulation_context.dt, simulation_context.dialation);
        }
    }
    transform.rotation = Quat::from_mat3(&rigid_body.rotation.as_mat3());
    let rotation = Mat3::from_quat(transform.rotation);
    gizmos.arrow(Vec3::ZERO, rotation.x_axis * 1.5, BLUE);
    gizmos.arrow(Vec3::ZERO, rotation.y_axis * 1.5, RED);
    gizmos.arrow(Vec3::ZERO, rotation.z_axis * 1.5, YELLOW);
}

#[cfg(test)]
mod test {
    use crate::rigid_body::{cube_from_inertia, inertia_cuboid_diag};
    use bevy::math::{DMat3, DVec3, Vec3};

    use super::inv_rectangular_cuboid_inertia_matrix;

    #[test]
    fn inv_rect_inverat_matrix() {
        let inertia = inertia_cuboid_diag(Vec3::new(3., 4., 5.));
        let diag = DVec3::new(205., 170., 125.);
        assert_eq!(inertia, diag);
    }

    #[test]
    fn cube_from_inertia_test() {
        let sides = Vec3::new(3., 4., 10.);
        let ineratia = inertia_cuboid_diag(sides);
        let calculated_sides = cube_from_inertia(ineratia);
        println!("sides: {:?}", calculated_sides);
    }

    #[test]
    fn calculate_angular_velocity() {
        let r = DMat3::IDENTITY;
        println!("{}", r);
        println!("{}", r.inverse());
        let angular_momentum = DVec3::new(0., 0.002, 0.2);
        let sides = Vec3::new(2.0, 0.1, 1.);
        let inv_inertia_tensor = inv_rectangular_cuboid_inertia_matrix(sides);
        let angular_vel = r * inv_inertia_tensor * r.inverse() * angular_momentum;
        let a2 = inv_inertia_tensor * angular_momentum;
        println!("{}", angular_vel);
        println!("{}", inv_inertia_tensor);
        println!("{}", a2);
    }
}
