use super::Boid;
use nalgebra::{Unit, Vector3};
use rstar::{RTree, RTreeObject};
use std::collections::HashMap;

const SEPARATION_FN: fn(f32) -> f32 = |t| t.powi(2) * 1e-2;

const COHESION_FN: fn(f32) -> f32 = |t| t.powi(2) * 1e-4;

const ATTRACTION_FN: fn(f32) -> f32 = |d| d.powi(2) * 1e-3;

const ALIGNMENT_FN: fn(f32) -> f32 = |t| t.powi(2) * 1e-2;

pub struct BoidsSimulation {
    pub boids: Vec<Boid>,
    pub attraction_center: Vector3<f32>,
    pub attraction_min_range: f32,
    pub separation_range: f32,
    pub cohesion_range: f32,
    pub alignment_strength: f32,
    pub coherence_strength: f32,
    pub max_speed: f32,
    pub min_speed: f32,
    pub max_neighbors: usize,
}

impl BoidsSimulation {
    pub fn update(&mut self) {
        // Build a tree for fast nearest neighbor search.
        let mut tree = RTree::new();
        for boid in &mut self.boids {
            // Reset all boids at the start of the simulation step.
            boid.reset();
            tree.insert(boid.desc());
        }

        let mut visited = HashMap::<(usize, usize), ()>::new();
        for bd1 in tree.iter() {
            for (i, bd2) in tree
                .nearest_neighbor_iter(&bd1.envelope().lower())
                .enumerate()
            {
                if i > self.max_neighbors {
                    break;
                }

                if bd1.id == bd2.id {
                    continue;
                }

                let pair = if bd1.id > bd2.id {
                    (bd2.id, bd1.id)
                } else {
                    (bd1.id, bd2.id)
                };
                // Avoid re-visiting a pair of boids: everytime we visit a pair, we already update both boids together.
                if visited.contains_key(&pair) {
                    continue;
                }

                visited.insert(pair, ());

                let b1 = &self.boids[bd1.id];
                let b2 = &self.boids[bd2.id];

                let travel = b2.translation - b1.translation;
                let dist = travel.norm();

                if dist > self.cohesion_range {
                    break;
                }

                if dist <= self.separation_range {
                    // Separation
                    let t = (self.separation_range - dist) / self.separation_range;
                    let force = -travel * SEPARATION_FN(t);
                    self.boids[bd1.id].acceleration += force;
                    self.boids[bd2.id].acceleration += -force;
                } else {
                    // Cohesion
                    let t = (self.cohesion_range - dist)
                        / (self.cohesion_range - self.separation_range);
                    let force = travel * COHESION_FN(t);

                    self.boids[bd1.id].acceleration += force;
                    self.boids[bd2.id].acceleration += -force;

                    // Alignment
                    let alignment_factor = ALIGNMENT_FN(t);
                    let b1_vel = self.boids[bd1.id].velocity;
                    let b2_vel = self.boids[bd2.id].velocity;
                    self.boids[bd1.id].neighbor_velocity += alignment_factor * b2_vel;
                    self.boids[bd2.id].neighbor_velocity += alignment_factor * b1_vel;
                }
            }
        }

        for boid in &mut self.boids {
            // Attraction
            let delta = self.attraction_center - boid.translation;
            let dist = delta.norm();
            if dist >= self.attraction_min_range {
                boid.acceleration +=
                    delta.normalize() * ATTRACTION_FN(dist - self.attraction_min_range);
            }

            // Alignment
            if boid.neighbor_velocity.norm() > 0.0 && boid.velocity.norm() > 0.0 {
                let velocity_dir = Unit::new_normalize(boid.velocity);
                let avg_neighbor_velocity_dir = Unit::new_normalize(boid.neighbor_velocity);
                let wanted_velocity = velocity_dir
                    .slerp(&avg_neighbor_velocity_dir, self.alignment_strength)
                    .into_inner()
                    * boid.velocity.norm();
                boid.acceleration += wanted_velocity - boid.velocity;
            }

            if boid.acceleration.norm() > 0.0 {
                // Coherence: ensure direction of acceleration is not too far from
                // the direction of the current velocity. Increases boids' turn radius.
                let acceleration_dir = Unit::new_normalize(boid.acceleration);
                let velocity_dir = Unit::new_normalize(boid.velocity);
                let new_acceleration_dir = acceleration_dir
                    .slerp(&velocity_dir, self.coherence_strength)
                    .into_inner();
                boid.acceleration = new_acceleration_dir * boid.acceleration.norm();
            }

            boid.velocity += boid.acceleration;

            // Speed control: ensure we don't accelerate past the max speed, or decelerate past the min speed.
            // This ensures that boids have a minimum turn radius.
            let speed = boid.velocity.norm().max(self.min_speed).min(self.max_speed);
            boid.velocity = boid.velocity.normalize() * speed;

            // Apply velocity.
            boid.translation += boid.velocity;

            boid.update_node();
        }
    }
}
