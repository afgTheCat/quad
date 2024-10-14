use crate::generated::bf_bindings::init;

pub struct BFController;

impl BFController {
    fn init() {
        unsafe {
            init();
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
