use nalgebra::{Matrix3, Vector3};

use crate::constants::AIR_RHO;

fn inertia_cuboid_diag(sides: Vector3<f64>) -> Vector3<f64> {
    let x = sides[0];
    let y = sides[1];
    let z = sides[2];
    let v = x * y * z;
    (1. / 12.)
        * v
        * Vector3::new(
            y.powi(2) + z.powi(2),
            x.powi(2) + z.powi(2),
            x.powi(2) + y.powi(2),
        )
}

pub fn inv_cuboid_inertia_tensor(sides: Vector3<f64>) -> Matrix3<f64> {
    let x = sides[0];
    let y = sides[1];
    let z = sides[2];
    let v = x * y * z;
    let diag: Vector3<f64> = (1. / 12.)
        * v
        * Vector3::new(
            1. / y.powi(2) + z.powi(2),
            1. / x.powi(2) + z.powi(2),
            1. / x.powi(2) + y.powi(2),
        );
    Matrix3::from_diagonal(&diag)
}

fn cross_product_matrix(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0., -v[2], v[1], v[2], 0., -v[0], -v[1], v[0], 0.)
}

// TODO: this is kinda stupid as we are not planning to store the location
#[derive(Debug, Clone, Default)]
pub struct RigidBody {
    pub linear_velocity: Vector3<f64>,
    pub linear_velocity_dir: Vector3<f64>,
    pub position: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub mass: f64,
    pub frame_drag_area: Vector3<f64>,
    pub frame_drag_constant: f64,
    pub inv_tensor: Matrix3<f64>,
    pub rotation: Matrix3<f64>,
    pub acceleration: Vector3<f64>,
}

// we are concerned with 3 things: linear velocity,
impl RigidBody {
    fn drag_linear(&self, drag_dir: &Vector3<f64>) -> Vector3<f64> {
        let local_dir = self.rotation.transpose() * self.linear_velocity_dir;
        let area_linear = Vector3::dot(&self.frame_drag_area, &local_dir.abs());
        drag_dir * area_linear
    }

    // TODO: fix this
    fn drag_angular(&self, drag_dir: &Vector3<f64>) -> Vector3<f64> {
        let local_dir = self.rotation.transpose() * self.linear_velocity_dir;
        let area_angular = Vector3::dot(&self.frame_drag_area, &local_dir);
        let drag_angular: Vector3<f64> =
            self.rotation.transpose() * (drag_dir * area_angular) * 0.001;
        drag_angular
        // TODO: readd this
        // Vector3::clamp(
        //     drag_angular,
        //     Vector3::new(-0.9, -0.9, -0.9),
        //     Vector3::new(0.9, 0.9, 0.9),
        // )
    }

    pub fn integrate(
        &mut self,
        motor_torque: f64,
        sum_arm_forces: &Vector3<f64>,
        sum_prop_torques: &Vector3<f64>,
        dt: f64,
    ) {
        let drag_dir = self.linear_velocity.norm_squared()
            * self.linear_velocity_dir
            * 0.5
            * AIR_RHO
            * self.frame_drag_constant;
        let drag_linear = self.drag_linear(&drag_dir);
        let drag_angular = self.drag_angular(&drag_dir);
        let total_force = Vector3::new(0., -9.81 * self.mass, 0.) - drag_linear + sum_arm_forces;
        let acceleration = total_force / self.mass;

        // TODO: readd this
        let total_moment = self.rotation.column(1) * motor_torque
            + self.rotation.column(0) * drag_angular[1]
            + self.rotation.column(1) * drag_angular[0]
            + self.rotation.column(2) * drag_angular[2]
            + sum_prop_torques;
        // let total_moment: Vector3<f64> = Vector3::zeros();
        let angular_acc: Vector3<f64> =
            self.rotation * self.inv_tensor * self.rotation.transpose() * total_moment;

        let angular_velocity = self.angular_velocity + angular_acc * dt;
        // TODO: We probably don't need to clamp here
        //     .clamp(
        //     Vector3::new(-100., -100., -100.),
        //     Vector3::new(100., 100., 100.),
        // );

        // update things
        self.acceleration = acceleration;
        self.position += dt * self.linear_velocity + (acceleration * dt.powi(2)) / 2.;
        self.angular_velocity = angular_velocity;
        self.linear_velocity += acceleration * dt;
        self.rotation += dt * cross_product_matrix(angular_acc) * self.rotation;
        self.linear_velocity_dir = self.linear_velocity.normalize()
    }
}
