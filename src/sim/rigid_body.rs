use bevy::math::{DMat3, DVec3, Vec3};

use crate::constants::AIR_RHO;

use super::drone::{xform, xform_inv};

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

// TODO: this is kinda stupid as we are not planning to store the location
#[derive(Debug, Clone, Default)]
pub struct RigidBody {
    pub linear_velocity: DVec3,
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
    fn gravity_force(&self) -> DVec3 {
        DVec3::new(0., -9.81 * self.mass, 0.)
    }

    fn drag_dir(&self) -> DVec3 {
        let vel2 = self.linear_velocity.length().powi(2);
        let dir = self.linear_velocity.normalize();
        dir * 0.5 * AIR_RHO * vel2 * self.frame_drag_constant
    }

    fn drag_linear(&self) -> DVec3 {
        if self.linear_velocity == DVec3::ZERO {
            return DVec3::ZERO;
        }
        let dir = self.linear_velocity.normalize();
        let local_dir = xform_inv(self.rotation, dir);
        let area_linear = DVec3::dot(self.frame_drag_area, local_dir.abs());
        let drag_dir = self.drag_dir();
        drag_dir * area_linear
    }

    // TODO: fix this
    fn drag_angular(&self) -> DVec3 {
        if self.linear_velocity == DVec3::ZERO {
            return DVec3::ZERO;
        }
        let dir = self.linear_velocity.normalize();
        let drag_dir = self.drag_dir();
        let local_dir = xform_inv(self.rotation, dir);
        let area_angular = DVec3::dot(self.frame_drag_area, local_dir);
        DVec3::clamp(
            xform_inv(self.rotation, drag_dir * area_angular) * 0.001,
            DVec3::new(-0.9, -0.9, -0.9),
            DVec3::new(0.9, 0.9, 0.9),
        )
    }

    fn angular_acc(&self, moment: DVec3) -> DVec3 {
        xform(self.rotation * self.inv_tensor * self.rotation, moment)
    }

    fn rotation(&self, angular_velocity: DVec3, dt: f64) -> DMat3 {
        let w = angular_velocity * dt;
        let W = DMat3 {
            x_axis: DVec3::new(1., -w[2], w[1]),
            y_axis: DVec3::new(w[2], 1., -w[0]),
            z_axis: DVec3::new(-w[1], w[0], 1.),
        };
        W * self.rotation
    }

    pub fn integrate(
        &mut self,
        motor_torque: f64,
        sum_arm_forces: DVec3,
        sum_prop_torques: DVec3,
        dt: f64,
    ) {
        let gravity_force = self.gravity_force();
        let drag_linear = self.drag_linear();
        let drag_angular = self.drag_angular();
        let total_force = gravity_force - drag_linear + sum_arm_forces;
        let acceleration = total_force / self.mass;

        let total_moment = self.rotation.y_axis * motor_torque
            + self.rotation.x_axis * drag_angular[1]
            + self.rotation.y_axis * drag_angular[0]
            + self.rotation.z_axis * drag_angular[2]
            + sum_prop_torques;
        let angular_acc = self.angular_acc(total_moment);
        let angular_velocity = (self.angular_velocity + angular_acc * dt).clamp(
            DVec3::new(-100., -100., -100.),
            DVec3::new(100., 100., 100.),
        );
        let rotation = self.rotation(angular_velocity, dt);

        // update things
        self.acceleration = acceleration;
        self.position += dt * self.linear_velocity + (acceleration * dt.powi(2)) / 2.;
        self.rotation = rotation;
        self.angular_velocity = angular_velocity;
    }
}
