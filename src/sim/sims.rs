use super::boid_sim::BOID_MESH;
use super::Simulation;
use nalgebra::Vector3;
use std::rc::Rc;

pub struct BoidSim();

impl Simulation for BoidSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        BOID_MESH.with(|m| {
            let mut node = scene.add_mesh(Rc::clone(m), Vector3::<f32>::new(1e-1, 1e-1, 1e-1));
            node.set_color(1.0, 0.0, 0.0);
        });
        Self()
    }
}

pub struct CubeSim();

impl Simulation for CubeSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let space = Vector3::<f32>::new(1.0, 1.0, 1.0) * 5e-1;
        let offset = -space / 2.0;

        for _ in 0..1000 {
            let translation = Vector3::<f32>::new(
                rand::random::<f32>(),
                rand::random::<f32>(),
                rand::random::<f32>(),
            )
            .component_mul(&space)
                + offset;
            let mut node = scene.add_sphere(0.005);
            node.set_color(1.0, 0.0, 0.0);
            node.set_local_translation(translation.into());
        }

        Self()
    }
}

pub struct SphereBiased1Sim();

impl Simulation for SphereBiased1Sim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let sphere_radius = 5e-1;

        for _ in 0..1000 {
            let r = sphere_radius * rand::random::<f32>();
            let theta = std::f32::consts::PI * rand::random::<f32>();
            let phi = std::f32::consts::PI * 2.0 * rand::random::<f32>();
            let translation = Vector3::<f32>::new(
                r * theta.sin() * phi.cos(),
                r * theta.sin() * phi.sin(),
                r * theta.cos(),
            );
            let mut node = scene.add_sphere(0.005);
            node.set_color(1.0, 0.0, 0.0);
            node.set_local_translation(translation.into());
        }

        Self()
    }
}

pub struct SphereBiased2Sim();

impl Simulation for SphereBiased2Sim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let sphere_radius = 5e-1;

        for _ in 0..1000 {
            let r = sphere_radius * rand::random::<f32>();
            let theta = (2.0 * rand::random::<f32>() - 1.0).acos();
            let phi = std::f32::consts::PI * 2.0 * rand::random::<f32>();
            let translation = Vector3::<f32>::new(
                r * theta.sin() * phi.cos(),
                r * theta.sin() * phi.sin(),
                r * theta.cos(),
            );
            let mut node = scene.add_sphere(0.005);
            node.set_color(1.0, 0.0, 0.0);
            node.set_local_translation(translation.into());
        }

        Self()
    }
}

pub struct SphereSim();

impl Simulation for SphereSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let sphere_radius = 5e-1;

        for _ in 0..1000 {
            let r = sphere_radius * rand::random::<f32>().powf(1.0 / 3.0);
            let theta = (2.0 * rand::random::<f32>() - 1.0).acos();
            let phi = std::f32::consts::PI * 2.0 * rand::random::<f32>();
            let translation = Vector3::<f32>::new(
                r * theta.sin() * phi.cos(),
                r * theta.sin() * phi.sin(),
                r * theta.cos(),
            );
            let mut node = scene.add_sphere(0.005);
            node.set_color(1.0, 0.0, 0.0);
            node.set_local_translation(translation.into());
        }

        Self()
    }
}

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

use super::boid_sim::{Boid, BoidsSimulation};

pub struct NoConstraintsSim {
    sim: BoidsSimulation,
}

impl Simulation for NoConstraintsSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.03f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(1000, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: std::f32::INFINITY,
                separation_range: 0.0,
                cohesion_range: 0.0,
                alignment_strength: 0.0,
                coherence_strength: 0.0,
                max_speed,
                min_speed,
                max_neighbors: std::usize::MAX,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}

pub struct CohesionSim {
    sim: BoidsSimulation,
}

impl Simulation for CohesionSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.03f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(100, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: std::f32::INFINITY,
                separation_range: 0.0,
                cohesion_range: 5e0 * scale,
                alignment_strength: 0.0,
                coherence_strength: 0.0,
                max_speed,
                min_speed,
                max_neighbors: std::usize::MAX,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}

pub struct SeparationSim {
    sim: BoidsSimulation,
}

impl Simulation for SeparationSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.03f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(100, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: std::f32::INFINITY,
                separation_range: 1e0 * scale,
                cohesion_range: 5e0 * scale,
                alignment_strength: 0.0,
                coherence_strength: 0.0,
                max_speed,
                min_speed,
                max_neighbors: std::usize::MAX,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}

pub struct AlignmentSim {
    sim: BoidsSimulation,
}

impl Simulation for AlignmentSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.03f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(100, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: std::f32::INFINITY,
                separation_range: 1e0 * scale,
                cohesion_range: 5e0 * scale,
                alignment_strength: 1e-1,
                coherence_strength: 0.0,
                max_speed,
                min_speed,
                max_neighbors: std::usize::MAX,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}

pub struct AttractionSim {
    sim: BoidsSimulation,
}

impl Simulation for AttractionSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.03f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(100, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: 1e-1,
                separation_range: 1e0 * scale,
                cohesion_range: 5e0 * scale,
                alignment_strength: 1e-1,
                coherence_strength: 0.0,
                max_speed,
                min_speed,
                max_neighbors: std::usize::MAX,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}
pub struct CoherenceSim {
    sim: BoidsSimulation,
}

impl Simulation for CoherenceSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.03f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(100, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: 1e-1,
                separation_range: 1e0 * scale,
                cohesion_range: 5e0 * scale,
                alignment_strength: 1e-1,
                coherence_strength: 5e-1,
                max_speed,
                min_speed,
                max_neighbors: std::usize::MAX,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}

pub struct Neighbors5SmallSim {
    sim: BoidsSimulation,
}

impl Simulation for Neighbors5SmallSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.03f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(100, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: 1e-1,
                separation_range: 1e0 * scale,
                cohesion_range: 5e0 * scale,
                alignment_strength: 1e-1,
                coherence_strength: 5e-1,
                max_speed,
                min_speed,
                max_neighbors: 5,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}

pub struct Neighbors5BigSim {
    sim: BoidsSimulation,
}

impl Simulation for Neighbors5BigSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.01f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(2000, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: 1e-1,
                separation_range: 1e0 * scale,
                cohesion_range: 5e0 * scale,
                alignment_strength: 1e-1,
                coherence_strength: 5e-1,
                max_speed,
                min_speed,
                max_neighbors: 5,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}

pub struct LeadersSim {
    sim: BoidsSimulation,
}

impl Simulation for LeadersSim {
    fn init(scene: &mut kiss3d::scene::SceneNode) -> Self {
        let scale = 0.01f32;
        let max_speed = 1e-1 * scale;
        let min_speed = 1e-2 * scale;

        Self {
            sim: BoidsSimulation {
                boids: Boid::generate_sphere(500, 1e-1, min_speed, max_speed, scale, scene),
                attraction_center: Vector3::<f32>::new(0.0, 0.0, 0.0),
                attraction_min_range: 1e-1,
                separation_range: 1e0 * scale,
                cohesion_range: 5e0 * scale,
                alignment_strength: 1e-1,
                coherence_strength: 5e-1,
                max_speed,
                min_speed,
                max_neighbors: 5,
            },
        }
    }

    fn update(&mut self) {
        self.sim.update();
    }
}
