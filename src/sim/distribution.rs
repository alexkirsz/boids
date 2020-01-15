use super::Simulation;
use nalgebra::Vector3;

pub struct DistributionSim();

impl Simulation for DistributionSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let space_radius = 0.5f32;

        for _ in 0..1000 {
            let r = space_radius * rand::random::<f32>().powf(1.0 / 2.0);
            let theta = std::f32::consts::PI * rand::random::<f32>();
            let translation = Vector3::<f32>::new(r * theta.cos(), r * theta.sin(), 0.0);
            let mut node = scene.add_sphere(0.005);
            node.set_color(1.0, 0.0, 0.0);
            node.set_local_translation(translation.into());
        }

        for _ in 0..1000 {
            let r = space_radius * rand::random::<f32>();
            let theta = std::f32::consts::PI + std::f32::consts::PI * rand::random::<f32>();
            let translation = Vector3::<f32>::new(r * theta.cos(), r * theta.sin(), 0.0);
            let mut node = scene.add_sphere(0.005);
            node.set_color(0.0, 1.0, 0.0);
            node.set_local_translation(translation.into());
        }

        Self()
    }
}
