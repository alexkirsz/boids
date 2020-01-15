use kiss3d::light::Light;
use kiss3d::window::Window;
use std::cell::RefCell;
use std::env;

mod app;
mod controls;
mod sim;

thread_local! {
    static SHOULD_STOP: RefCell<bool> = RefCell::new(false);
}

fn main() {
    let args: Vec<_> = env::args().collect();

    if args.len() != 2 {
        return;
    }

    controls::start_simulation(&args[1]);
}
