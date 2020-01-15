pub trait Simulation {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self;
    fn update(&mut self) {}
}
