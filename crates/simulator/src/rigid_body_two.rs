use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::{
    components::{ComponentModel, DroneComponent},
    constants::{AIR_RHO, GRAVITY},
};

/// We often rely on the linear velocity having a direction
pub struct LinearVelocity {
    velocity: Vector3<f64>,
    direction: Option<Vector3<f64>>,
}

pub struct RigidBodyTwo {
    pub position: Vector3<f64>,
    pub rotation: Rotation3<f64>,
    pub linear_velocity: LinearVelocity,
    pub angular_velocity: Vector3<f64>,
    pub mass: f64,                // does nor change
    pub inv_tensor: Matrix3<f64>, // does not change
}

fn cross_product_matrix(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0., -v[2], v[1], v[2], 0., -v[0], -v[1], v[0], 0.)
}

impl RigidBodyTwo {
    fn integrate(&mut self, applied_torque: Vector3<f64>, applied_force: Vector3<f64>, dt: f64) {
        let acceleration = applied_force / self.mass;
        self.position += dt * self.linear_velocity.velocity + (acceleration * dt.powi(2)) / 2.;
        let angular_acc =
            self.rotation * self.inv_tensor * self.rotation.transpose() * applied_torque;
        self.angular_velocity += angular_acc * dt;
        self.rotation = Rotation3::from_matrix_eps(
            &((Matrix3::identity() + cross_product_matrix(self.angular_velocity * dt))
                * self.rotation.matrix()),
            0.0000000001,
            100,
            self.rotation,
        );
    }
}

struct RigidBodyModel {
    mass: f64,
    inv_tensor: Matrix3<f64>,
}

struct RigidBodyState {
    position: Vector3<f64>,
    rotation: Rotation3<f64>,
    linear_velocity: Vector3<f64>,
    velocity_direction: Option<Vector3<f64>>,
    angular_velocity: Vector3<f64>,
}

struct RigidBodyUpdate {
    dt: f64,
    applied_torque: Vector3<f64>,
    applied_force: Vector3<f64>,
}

impl ComponentModel for RigidBodyModel {
    type ComponentState = RigidBodyState;
    type ComponentUpdate = RigidBodyUpdate;

    fn get_new_state(
        &mut self,
        state: &Self::ComponentState,
        update: Self::ComponentUpdate,
    ) -> Self::ComponentState {
        let acceleration = update.applied_force / self.mass;
        let linear_velocity = state.linear_velocity + acceleration * update.dt;
        let velocity_direction = if linear_velocity.lp_norm(1) > 0. {
            Some(linear_velocity.normalize())
        } else {
            None
        };
        let position = state.position
            + update.dt * state.linear_velocity
            + (acceleration * update.dt.powi(2)) / 2.;
        let angular_acc =
            state.rotation * self.inv_tensor * state.rotation.transpose() * update.applied_torque;
        let angular_velocity = state.angular_velocity + angular_acc * update.dt;
        let rotation = Rotation3::from_matrix_eps(
            &((Matrix3::identity() + cross_product_matrix(state.angular_velocity * update.dt))
                * state.rotation.matrix()),
            0.0000000001,
            100,
            state.rotation,
        );
        RigidBodyState {
            position,
            rotation,
            linear_velocity,
            velocity_direction,
            angular_velocity,
        }
    }
}

type RigidBody = DroneComponent<RigidBodyModel>;

// TODO: how do I do this? frame is one component? should it have a position?
struct FrameComponentModel {
    frame_drag_area: Vector3<f64>,
    frame_drag_constant: f64,
    motor_positions: [Vector3<f64>; 4],
    motor_dir: [f64; 4],
    mass: f64,
}

impl FrameComponentModel {
    fn drag_linear(
        &self,
        drag_dir: &Vector3<f64>,
        linear_velocity_dir: &Vector3<f64>,
        rotation: Matrix3<f64>,
    ) -> Vector3<f64> {
        let local_dir = rotation.transpose() * linear_velocity_dir;
        let area_linear = Vector3::dot(&self.frame_drag_area, &local_dir.abs());
        drag_dir * area_linear
    }

    fn drag_angular(
        &self,
        drag_dir: &Vector3<f64>,
        linear_velocity_dir: &Vector3<f64>,
        rotation: Matrix3<f64>,
    ) -> Vector3<f64> {
        let local_dir = rotation.transpose() * linear_velocity_dir;
        let area_angular = Vector3::dot(&self.frame_drag_area, &local_dir);
        let drag_angular: Vector3<f64> = rotation.transpose() * (drag_dir * area_angular) * 0.001;
        drag_angular
    }
}

// nothing really
struct FrameComponentState {
    linear_drag: Vector3<f64>,
    angular_drag: Vector3<f64>,
    motor_positions: [Vector3<f64>; 4],
}

struct FrameComponentUpdate {
    thrusts: [f64; 4], // The thrust that is created by the quad
    torques: [f64; 4], // The motor torques
    linear_velocity_dir: Option<Vector3<f64>>,
    linear_velocity: Vector3<f64>,
    rotation: Matrix3<f64>,
}

impl ComponentModel for FrameComponentModel {
    type ComponentState = FrameComponentState;
    type ComponentUpdate = FrameComponentUpdate;

    fn get_new_state(
        &mut self,
        state: &Self::ComponentState,
        update: Self::ComponentUpdate,
    ) -> Self::ComponentState {
        let (drag_linear, drag_angular) = if let Some(linear_velocity_dir) =
            update.linear_velocity_dir
        {
            let drag_dir = update.linear_velocity.norm_squared()
                * linear_velocity_dir
                * 0.5
                * AIR_RHO
                * self.frame_drag_constant;
            let drag_linear = self.drag_linear(&drag_dir, &linear_velocity_dir, update.rotation);
            let drag_angular = self.drag_angular(&drag_dir, &linear_velocity_dir, update.rotation);
            (drag_linear, drag_angular)
        } else {
            (Vector3::zeros(), Vector3::zeros())
        };
        let motor_thrusts = update
            .thrusts
            .map(|thrust| update.rotation * Vector3::new(0., thrust, 0.));

        let prop_torque: Vector3<f64> = motor_thrusts
            .iter()
            .enumerate()
            .map(|(i, thrust)| {
                let rad = update.rotation * self.motor_positions[i];
                Vector3::cross(&rad, &thrust)
            })
            .sum();

        let sum_arm_forces: Vector3<f64> = motor_thrusts.iter().sum();

        let motor_torque: f64 = update
            .torques
            .iter()
            .enumerate()
            .map(|(i, torque)| torque * self.motor_dir[i])
            .sum();
        let total_moment = update.rotation.column(1) * motor_torque + prop_torque;
        let total_force = Vector3::new(0., -GRAVITY * self.mass, 0.) - drag_linear + sum_arm_forces;

        // let sum_arm_forces = Vector3::new(0., -GRAVITY * self.mass, 0.) - drag_linear +

        todo!()
    }
}

// things we need for the physics update:
// motor torques
// motor forces (thrust)
