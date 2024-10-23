use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::constants::{AIR_RHO, GRAVITY};

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
    pub linear_velocity_dir: Option<Vector3<f64>>,
    pub position: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub mass: f64,
    pub frame_drag_area: Vector3<f64>,
    pub frame_drag_constant: f64,
    pub inv_tensor: Matrix3<f64>,
    pub rotation: Rotation3<f64>,
    pub acceleration: Vector3<f64>,
}

// we are concerned with 3 things: linear velocity,
impl RigidBody {
    fn drag_linear(
        &self,
        drag_dir: &Vector3<f64>,
        linear_velocity_dir: &Vector3<f64>,
    ) -> Vector3<f64> {
        let local_dir = self.rotation.transpose() * linear_velocity_dir;
        let area_linear = Vector3::dot(&self.frame_drag_area, &local_dir.abs());
        drag_dir * area_linear
    }

    // TODO: fix this
    fn drag_angular(
        &self,
        drag_dir: &Vector3<f64>,
        linear_velocity_dir: &Vector3<f64>,
    ) -> Vector3<f64> {
        let local_dir = self.rotation.transpose() * linear_velocity_dir;
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
        sum_arm_forces: Vector3<f64>,
        sum_prop_torques: Vector3<f64>,
        dt: f64,
    ) {
        let (drag_linear, drag_angular) =
            if let Some(linear_velocity_dir) = self.linear_velocity_dir {
                let drag_dir = self.linear_velocity.norm_squared()
                    * linear_velocity_dir
                    * 0.5
                    * AIR_RHO
                    * self.frame_drag_constant;
                let drag_linear = self.drag_linear(&drag_dir, &linear_velocity_dir);
                let drag_angular = self.drag_angular(&drag_dir, &linear_velocity_dir);
                (drag_linear, drag_angular)
            } else {
                (Vector3::zeros(), Vector3::zeros())
            };
        let total_force = Vector3::new(0., -GRAVITY * self.mass, 0.) - drag_linear + sum_arm_forces;
        let acceleration = total_force / self.mass;

        // TODO: readd this
        // let total_applied_moment = Vector3::zeros();
        // println!("sum prop torques: {:?}", sum_prop_torques);
        let total_applied_moment =
            self.rotation.matrix().column(1) * motor_torque + sum_prop_torques;

        // let total_applied_moment = self.rotation.column(1) * motor_torque
        //     + self.rotation.column(0) * drag_angular[1]
        //     + self.rotation.column(1) * drag_angular[0]
        //     + self.rotation.column(2) * drag_angular[2]
        //     + sum_prop_torques;

        let angular_acc: Vector3<f64> =
            self.rotation * self.inv_tensor * self.rotation.transpose() * total_applied_moment;

        // println!("angular acc: {:?}", angular_acc);
        //
        // println!(
        //     "angular velocity: {:?}",
        //     self.angular_velocity + angular_acc * dt
        // );

        // update things
        self.angular_velocity += angular_acc * dt;
        self.acceleration = acceleration;
        self.position += dt * self.linear_velocity + (acceleration * dt.powi(2)) / 2.;
        self.linear_velocity += acceleration * dt;

        // TODO:: There might be a better way to do this
        self.rotation = Rotation3::from_matrix_eps(
            &((Matrix3::identity() + cross_product_matrix(self.angular_velocity * dt))
                * self.rotation.matrix()),
            0.0000000001,
            100,
            self.rotation,
        );
        self.linear_velocity_dir = if self.linear_velocity.lp_norm(1) > 0. {
            Some(self.linear_velocity.normalize())
        } else {
            None
        };
    }
}

#[cfg(test)]
mod test {
    use nalgebra::{Rotation3, Vector3};

    #[test]
    fn rigid_body_test() {
        let sum_prop_torques = Vector3::new(-1.312636677497592e-13, 0.0, 0.0);
        let rotation = Rotation3::<f64>::identity();

        // let angular_acc = rotation *
    }
}
