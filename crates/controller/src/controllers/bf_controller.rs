use crate::bindings::bf_bindings_generated::{init, scheduler};

pub struct BFController;

impl BFController {
    fn reset_rc_data() {}

    fn init() {
        unsafe {
            init();
        }
    }

    fn scheduler() {
        unsafe {
            scheduler();
        }
    }
}

#[cfg(test)]
mod test {
    use super::BFController;

    #[test]
    fn bf_init_test() {
        BFController::init();
    }
}
