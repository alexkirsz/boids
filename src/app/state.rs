use crate::sim::Simulation;
use kiss3d::conrod::widget_ids;
use kiss3d::window::{State, Window};
use std::cell::RefCell;
use std::thread::LocalKey;

widget_ids! {
    struct Ids {
        canvas,
        play_pause_button,
        restart_button
    }
}

macro_rules! image_ids {
    ( $( $x:ident: $path:expr,)* ) => {
        use kiss3d;

        struct ImageIds {
            $(
                $x: kiss3d::conrod::image::Id,
            )*
        }

        impl ImageIds {
            pub fn new(window: &mut kiss3d::window::Window) -> ImageIds {
                kiss3d::resource::TextureManager::get_global_manager(|tm| {
                    $(
                        tm.add_image_from_memory(include_bytes!($path), stringify!($x));
                    )*
                });

                ImageIds {
                    $(
                        $x: window.conrod_texture_id(stringify!($x)).unwrap(),
                    )*
                }
            }
        }
    };


    ( $( $x:ident: $path:expr),* ) => {
        image_ids!($($x: $path,)*)
    };
}

image_ids!(
    play_64: "images/play_64.png",
    play_hover_64: "images/play_hover_64.png",
    play_press_64: "images/play_press_64.png",
    pause_64: "images/pause_64.png",
    pause_hover_64: "images/pause_hover_64.png",
    pause_press_64: "images/pause_press_64.png",
    restart_64: "images/restart_64.png",
    restart_hover_64: "images/restart_hover_64.png",
    restart_press_64: "images/restart_press_64.png",
);

pub struct AppState<Sim> {
    ids: Ids,
    image_ids: ImageIds,
    running: bool,
    group: kiss3d::scene::SceneNode,
    sim: Sim,
    should_stop: &'static LocalKey<RefCell<bool>>,
}

impl<Sim: Simulation> AppState<Sim> {
    pub fn new(
        mut window: &mut kiss3d::window::Window,
        should_stop: &'static LocalKey<RefCell<bool>>,
    ) -> AppState<Sim> {
        let mut group = window.add_group();
        let sim = Sim::init(&mut group);
        let ids = Ids::new(window.conrod_ui_mut().widget_id_generator());
        let image_ids = ImageIds::new(&mut window);

        AppState {
            ids,
            image_ids,
            sim,
            group,
            running: true,
            should_stop,
        }
    }

    pub fn gui(&mut self, window: &mut Window) {
        use kiss3d::conrod::{position::Direction, widget, Positionable, Sizeable, Widget};

        let reset = {
            let ui = &mut window.conrod_ui_mut().set_widgets();

            let (ppb, ppb_hover, ppb_press) = if self.running {
                (
                    self.image_ids.pause_64,
                    self.image_ids.pause_hover_64,
                    self.image_ids.pause_press_64,
                )
            } else {
                (
                    self.image_ids.play_64,
                    self.image_ids.play_hover_64,
                    self.image_ids.play_press_64,
                )
            };
            let play_pause_button = widget::Button::image(ppb)
                .hover_image(ppb_hover)
                .press_image(ppb_press)
                .w(32.0)
                .h(32.0)
                .bottom_right_with_margin(8.0)
                .set(self.ids.play_pause_button, ui);

            if play_pause_button.was_clicked() {
                self.running = !self.running;
            }

            let restart_btn = widget::Button::image(self.image_ids.restart_64)
                .hover_image(self.image_ids.restart_hover_64)
                .press_image(self.image_ids.restart_press_64)
                .w(32.0)
                .h(32.0)
                .bottom_right_with_margin(8.0)
                .x_direction_from(self.ids.play_pause_button, Direction::Backwards, 8.0)
                .set(self.ids.restart_button, ui);

            restart_btn.was_clicked()
        };

        if reset {
            self.group.unlink();
            self.group = window.add_group();
            self.sim = Sim::init(&mut self.group);
        }
    }
}

impl<Sim: Simulation + 'static> State for AppState<Sim> {
    fn step(&mut self, window: &mut Window) {
        if self.running {
            self.sim.update();
        }

        self.gui(window);

        self.should_stop.with(|s| {
            if *s.borrow() {
                window.close();
            }
        });
    }
}
