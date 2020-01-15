mod app;
mod controls;
mod sim;

#[cfg(target_arch = "wasm32")]
#[stdweb::js_export]
pub fn start(typ: String) {
    controls::start_simulation(&typ);
}

#[cfg(target_arch = "wasm32")]
#[stdweb::js_export]
pub fn stop() {
    controls::stop_simulation();
}

// #[cfg(target_arch = "wasm32")]
// #[stdweb::js_export]
// pub fn stop(id: u32) -> bool {
//     use std::collections::hash_map::Entry;

//     map.with(|m| {
//         let entry = m.entry(id);
//         match entry {
//             Entry::Occupied(e) => {
//                 e.get_mut().close();
//                 e.remove_entry();
//                 true
//             }
//             _ => false,
//         }
//     })
// }
