use kiss3d::{resource::Mesh, scene::SceneNode};
use nalgebra::{Point3, Rotation3, Vector3};
use rstar::{PointDistance, RTreeObject, AABB};
use std::cell::RefCell;
use std::rc::Rc;

pub struct BoidDesc {
    pub id: usize,
    pub position: Point3<f32>,
}

impl BoidDesc {
    pub fn new(id: usize, position: Point3<f32>) -> BoidDesc {
        BoidDesc { id, position }
    }
}

impl RTreeObject for BoidDesc {
    type Envelope = AABB<[f32; 3]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.position.x, self.position.y, self.position.z])
    }
}

impl PointDistance for BoidDesc {
    fn distance_2(&self, point: &[f32; 3]) -> f32 {
        ((self.position.x - point[0]).powi(2)
            + (self.position.y - point[1]).powi(2)
            + (self.position.z - point[2]).powi(2))
        .sqrt()
    }

    fn contains_point(&self, _point: &[f32; 3]) -> bool {
        false
    }
}

pub struct Boid {
    pub id: usize,
    pub translation: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub neighbor_velocity: Vector3<f32>,
    pub acceleration: Vector3<f32>,
    pub node: SceneNode,
}

thread_local! {
    pub static BOID_MESH: Rc<RefCell<Mesh>> = Rc::new(RefCell::new(Mesh::new(
        vec![
            Point3::<f32>::new(
                (std::f32::consts::PI * 1.0 / 2.0).cos(),
                0.0,
                (std::f32::consts::PI * 1.0 / 2.0).sin(),
            ),
            Point3::<f32>::new(
                (std::f32::consts::PI * 7.0 / 6.0).cos(),
                0.0,
                (std::f32::consts::PI * 7.0 / 6.0).sin(),
            ),
            Point3::<f32>::new(
                (std::f32::consts::PI * 11.0 / 6.0).cos(),
                0.0,
                (std::f32::consts::PI * 11.0 / 6.0).sin(),
            ),
            Point3::<f32>::new(0.0, 3.0, 0.0),
        ],
        vec![
            Point3::<u16>::new(2, 1, 0),
            Point3::<u16>::new(0, 1, 3),
            Point3::<u16>::new(1, 2, 3),
            Point3::<u16>::new(3, 2, 0),
        ],
        None,
        None,
        false,
    )));
}

impl Boid {
    pub fn generate_sphere(
        n: usize,
        sphere_radius: f32,
        min_speed: f32,
        max_speed: f32,
        scale: f32,
        scene: &mut SceneNode,
    ) -> Vec<Boid> {
        assert!(min_speed <= max_speed);

        (0..n)
            .map(|id| {
                let r = sphere_radius * rand::random::<f32>().powf(1.0 / 3.0);
                let theta = (2.0 * rand::random::<f32>() - 1.0).acos();
                let phi = std::f32::consts::PI * 2.0 * rand::random::<f32>();
                let translation = Vector3::<f32>::new(
                    r * theta.sin() * phi.cos(),
                    r * theta.sin() * phi.sin(),
                    r * theta.cos(),
                );
                let velocity = Vector3::<f32>::new(
                    rand::random::<f32>() - 5e-1,
                    rand::random::<f32>() - 5e-1,
                    rand::random::<f32>() - 5e-1,
                )
                .normalize()
                    * (min_speed + rand::random::<f32>() * (max_speed - min_speed));
                Boid::new(
                    id,
                    translation,
                    velocity,
                    Vector3::<f32>::new(0.0, 0.0, 0.0),
                    0.1 * scale,
                    scene,
                )
            })
            .collect()
    }

    pub fn new(
        id: usize,
        translation: Vector3<f32>,
        velocity: Vector3<f32>,
        acceleration: Vector3<f32>,
        scale: f32,
        scene: &mut SceneNode,
    ) -> Boid {
        let mut node = BOID_MESH
            .with(|m| scene.add_mesh(Rc::clone(m), Vector3::<f32>::new(1.0, 1.0, 1.0) * scale));

        node.set_color(1.0, 0.0, 0.0);

        let mut boid = Boid {
            id,
            translation,
            velocity,
            acceleration,
            node,
            neighbor_velocity: Vector3::<f32>::new(0.0, 0.0, 0.0),
        };
        boid.update_node();
        boid
    }

    pub fn reset(&mut self) {
        self.acceleration = Vector3::<f32>::new(0.0, 0.0, 0.0);
        self.neighbor_velocity = Vector3::<f32>::new(0.0, 0.0, 0.0);
    }

    pub fn update_node(&mut self) {
        self.node.set_local_rotation(
            Rotation3::<f32>::rotation_between(&Vector3::<f32>::y(), &self.velocity)
                .unwrap()
                .into(),
        );
        self.node.set_local_translation(self.translation.into());
    }

    pub fn desc(&self) -> BoidDesc {
        BoidDesc::new(self.id, self.translation.into())
    }
}
