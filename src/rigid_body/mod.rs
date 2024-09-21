use bevy::math::{Mat3, Vec3};

#[derive(Debug, Clone)]
pub struct RigidBody {
    position: Vec3,
    linear_veolcity: Vec3,

    angular_velocity: Vec3,
    mass: f64,
    // frame_drag_area: Vec3,
    // frame_drag_constant: f64,
    inv_tensor: Mat3,
}
