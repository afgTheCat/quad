// maybe we can add some other things here in the future
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
// pub mod bf_bindings_generated;
pub mod sitl_generated;
// use bf_bindings_generated::{MAX_SUPPORTED_MOTORS, SIMULATOR_MAX_RC_CHANNELS};

const MAX_SUPPORTED_MOTORS_USIZE: usize = 8;
pub const SIMULATOR_MAX_RC_CHANNELS_USIZE: usize = 8;
// pub const SIMULATOR_MAX_RC_CHANNELS_U8: u8 = SIMULATOR_MAX_RC_CHANNELS as u8;
// extern "C" {
//     pub static motorsPwm: [i16; MAX_SUPPORTED_MOTORS_USIZE];
// }
