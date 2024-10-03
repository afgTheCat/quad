use bevy::math::{DMat3, DVec3};

use crate::constants::AIR_RHO;

use super::{xform, xform_inv};

// TODO: this is kinda stupid as we are not planning to store the location
#[derive(Debug, Clone)]
pub struct RigidBody {
    linear_veolcity: DVec3,
    angular_velocity: DVec3,
    mass: f64,
    frame_drag_area: DVec3,
    frame_drag_constant: f64,
    inv_tensor: DMat3,
}

impl RigidBody {
    fn new(
        linear_veolcity: DVec3,
        angular_velocity: DVec3,
        mass: f64,
        frame_drag_area: DVec3,
        frame_drag_constant: f64,
        inv_tensor: DMat3,
    ) -> Self {
        Self {
            linear_veolcity,
            angular_velocity,
            mass,
            frame_drag_area,
            frame_drag_constant,
            inv_tensor,
        }
    }
}

// we are concerned with 3 things: linear velocity,
impl RigidBody {
    pub fn gravity_force(&self) -> DVec3 {
        DVec3::new(0., -9.81 * self.mass, 0.)
    }

    pub fn acceleration(&self, force: DVec3) -> DVec3 {
        force / f64::max(self.mass, 0.0001)
    }

    fn drag_dir(&self, linear_velocity: DVec3) -> DVec3 {
        let vel2 = linear_velocity.length().powi(2);
        let dir = linear_velocity.normalize();
        dir * 0.5 * AIR_RHO * vel2 * self.frame_drag_constant
    }

    pub fn drag_linear(&self, linear_velocity: DVec3, rotation: DMat3) -> DVec3 {
        let dir = linear_velocity.normalize();
        let local_dir = xform_inv(rotation, dir);
        let area_linear = DVec3::dot(self.frame_drag_area, local_dir.abs());
        let drag_dir = self.drag_dir(linear_velocity);
        drag_dir * area_linear
    }

    pub fn drag_angular(&self, linear_velocity: DVec3, rotation: DMat3) -> DVec3 {
        let dir = linear_velocity.normalize();
        let drag_dir = self.drag_dir(linear_velocity);
        let local_dir = xform_inv(rotation, dir);
        let area_angular = DVec3::dot(self.frame_drag_area, local_dir);
        DVec3::clamp(
            xform_inv(rotation, (drag_dir * area_angular)) * 0.001,
            DVec3::new(-0.9, -0.9, -0.9),
            DVec3::new(0.9, 0.9, 0.9),
        )
    }

    pub fn angular_acc(&self, moment: DVec3, rotation: &DMat3) -> DVec3 {
        xform(
            rotation.clone() * self.inv_tensor.clone() * rotation.transpose(),
            moment,
        )
    }

    pub fn rotation(&self, angular_velocity: DVec3, dt: f64, rotation: &DMat3) -> DMat3 {
        let w = angular_velocity * dt;
        let W = DMat3 {
            x_axis: DVec3::new(1., -w[2], w[1]),
            y_axis: DVec3::new(w[2], 1., -w[0]),
            z_axis: DVec3::new(-w[1], w[0], 1.),
        };
        W * rotation.clone()
    }
}
