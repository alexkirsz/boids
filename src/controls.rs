use crate::{app, sim};
use std::cell::RefCell;

thread_local! {
    static SHOULD_STOP: RefCell<bool> = RefCell::new(false);
}

macro_rules! match_sim {
    ( $val:expr, $win:expr, $( $s:expr => $sim:path,)* ) => {
        match $val {
            $(
                $s => {
                    let state = app::AppState::<$sim>::new(
                        &mut $win,
                        &SHOULD_STOP,
                    );
                    $win.render_loop(state);
                },
            )*
            _ => panic!("Unknown type: {}", $val),
        }
    };

    ( $val:expr, $win:expr, $( $s:expr => $sim:path),* ) => {
        match_sim!($val, $win, $($x: $path,)*)
    };
}

pub fn start_simulation(typ: &str) {
    use kiss3d::light::Light;
    use kiss3d::window::Window;

    let mut window = Window::new(typ);
    window.set_light(Light::StickToCamera);

    match_sim!(&typ[..], window,
        "boid" => sim::sims::BoidSim,
        "cube" => sim::sims::CubeSim,
        "sphere_biased1" => sim::sims::SphereBiased1Sim,
        "sphere_biased2" => sim::sims::SphereBiased2Sim,
        "sphere" => sim::sims::SphereSim,
        "distribution" => sim::sims::DistributionSim,
        "no_constraints" => sim::sims::NoConstraintsSim,
        "cohesion" => sim::sims::CohesionSim,
        "separation" => sim::sims::SeparationSim,
        "alignment" => sim::sims::AlignmentSim,
        "attraction" => sim::sims::AttractionSim,
        "coherence" => sim::sims::CoherenceSim,
        "neighbors5_small" => sim::sims::Neighbors5SmallSim,
        "neighbors5_big" => sim::sims::Neighbors5BigSim,
        "leaders" => sim::sims::LeadersSim,
    );
}

pub fn stop_simulation() {
    SHOULD_STOP.with(|s| *s.borrow_mut() = true);
}
