use bevy::math::{DMat3, DVec3};

pub fn xform(m: DMat3, v: DVec3) -> DVec3 {
    DVec3::new(
        DVec3::dot(m.x_axis, v),
        DVec3::dot(m.y_axis, v),
        DVec3::dot(m.z_axis, v),
    )
}

pub fn xform_inv(m: DMat3, v: DVec3) -> DVec3 {
    DVec3::new(
        (m.x_axis[0] * v[0]) + (m.y_axis[0] * v[1]) + (m.z_axis[0] * v[2]),
        (m.x_axis[1] * v[0]) + (m.y_axis[1] * v[1]) + (m.z_axis[1] * v[2]),
        (m.x_axis[2] * v[0]) + (m.y_axis[2] * v[1]) + (m.z_axis[2] * v[2]),
    )
}
