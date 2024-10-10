use crate::constants::AIR_RHO;
use bevy::math::{DMat3, DVec3, Vec3};

fn inertia_cuboid_diag(sides: Vec3) -> DVec3 {
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

pub fn inv_cuboid_inertia_tensor(sides: Vec3) -> DMat3 {
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

// TODO: this is kinda stupid as we are not planning to store the location
#[derive(Debug, Clone, Default)]
pub struct RigidBody {
    pub linear_velocity: DVec3,
    pub linear_velocity_dir: DVec3,
    pub position: DVec3,
    pub angular_velocity: DVec3,
    pub mass: f64,
    pub frame_drag_area: DVec3,
    pub frame_drag_constant: f64,
    pub inv_tensor: DMat3,
    pub rotation: DMat3,
    pub acceleration: DVec3,
}

// we are concerned with 3 things: linear velocity,
impl RigidBody {
    fn drag_linear(&self, drag_dir: DVec3) -> DVec3 {
        let local_dir = self.rotation.transpose() * self.linear_velocity_dir;
        let area_linear = DVec3::dot(self.frame_drag_area, local_dir.abs());
        drag_dir * area_linear
    }

    // TODO: fix this
    fn drag_angular(&self, drag_dir: DVec3) -> DVec3 {
        let local_dir = self.rotation.transpose() * self.linear_velocity_dir;
        let area_angular = DVec3::dot(self.frame_drag_area, local_dir);
        DVec3::clamp(
            self.rotation.transpose() * (drag_dir * area_angular) * 0.001,
            DVec3::new(-0.9, -0.9, -0.9),
            DVec3::new(0.9, 0.9, 0.9),
        )
    }

    pub fn integrate(
        &mut self,
        motor_torque: f64,
        sum_arm_forces: DVec3,
        sum_prop_torques: DVec3,
        dt: f64,
    ) {
        let drag_dir = self.linear_velocity.length().powi(2)
            * self.linear_velocity_dir
            * 0.5
            * AIR_RHO
            * self.frame_drag_constant;
        let drag_linear = self.drag_linear(drag_dir);
        let drag_angular = self.drag_angular(drag_dir);
        let total_force = DVec3::new(0., -9.81 * self.mass, 0.) - drag_linear + sum_arm_forces;
        let acceleration = total_force / self.mass;

        let total_moment = self.rotation.y_axis * motor_torque
            + self.rotation.x_axis * drag_angular[1]
            + self.rotation.y_axis * drag_angular[0]
            + self.rotation.z_axis * drag_angular[2]
            + sum_prop_torques;
        let angular_acc =
            self.rotation * self.inv_tensor * self.rotation.transpose() * total_moment;

        // We probably don't need to clamp here
        let angular_velocity = (self.angular_velocity + angular_acc * dt).clamp(
            DVec3::new(-100., -100., -100.),
            DVec3::new(100., 100., 100.),
        );

        // update things
        self.acceleration = acceleration;
        self.position += dt * self.linear_velocity + (acceleration * dt.powi(2)) / 2.;
        self.angular_velocity = angular_velocity;
        self.linear_velocity += acceleration * dt;
        self.rotation += dt * cross_product_matrix(angular_acc) * self.rotation;
        self.linear_velocity_dir = self.linear_velocity.normalize()
    }

    pub fn integrate_two(&mut self, dt: f64) {
        let drag_dir = self.linear_velocity.length().powi(2)
            * self.linear_velocity_dir
            * 0.5
            * AIR_RHO
            * self.frame_drag_constant;

        let drag_linear = self.drag_linear(drag_dir);
        let drag_angular = self.drag_angular(drag_dir);

        let total_force = DVec3::new(0., -9.81 * self.mass, 0.) - drag_linear;
        let acceleration = total_force / self.mass;
        let total_moment = self.rotation.x_axis * drag_angular[1]
            + self.rotation.y_axis * drag_angular[0]
            + self.rotation.z_axis * drag_angular[2];

        let angular_acc =
            self.rotation * self.inv_tensor * self.rotation.transpose() * total_moment;

        // We probably don't need to clamp here
        let angular_velocity = (self.angular_velocity + angular_acc * dt).clamp(
            DVec3::new(-100., -100., -100.),
            DVec3::new(100., 100., 100.),
        );

        // update things
        self.acceleration = acceleration;
        self.position += dt * self.linear_velocity + (acceleration * dt.powi(2)) / 2.;
        self.angular_velocity = angular_velocity;
        self.linear_velocity += acceleration * dt;
        self.rotation += dt * cross_product_matrix(angular_acc) * self.rotation;
        self.linear_velocity_dir = self.linear_velocity.normalize()
    }
}
