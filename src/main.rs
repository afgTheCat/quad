mod constants;
mod low_pass_filter;
mod sample_curve;
mod simplex1d;

use rand::{Rng, SeedableRng};
use rand_xoshiro::Xoshiro256PlusPlus;
use std::{cell::RefCell, ops::Range};

#[cfg(feature = "legacy_sim")]
use quboid::build_app;

#[cfg(feature = "legacy_sim")]
mod quboid;

#[cfg(not(feature = "legacy_sim"))]
mod sim;

#[cfg(not(feature = "legacy_sim"))]
use sim::build_app;

thread_local! {
    static RNG: RefCell<Xoshiro256PlusPlus> = RefCell::new(Xoshiro256PlusPlus::from_entropy());
}

pub fn rng_gen_range(range: Range<f64>) -> f64 {
    RNG.with(|rng| rng.borrow_mut().gen_range(range))
}

// We can create our own gizmo config group!

fn main() {
    let mut app = build_app();
    app.run();
}

#[cfg(test)]
mod test {
    #[test]
    fn integrate_thing() {}
}
