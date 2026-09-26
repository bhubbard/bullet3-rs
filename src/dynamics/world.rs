use crate::collision::broadphase::DbvtBroadphase;
use crate::collision::dispatch::RayTestResult;
use crate::collision::narrowphase::{detect_collision, ContactManifold};
use crate::collision::shapes::Shape;
use crate::dynamics::constraint::Constraint;
use crate::dynamics::rigid_body::RigidBody;
use crate::dynamics::solver::SequentialImpulseConstraintSolver;
use crate::linear_math::{Transform, Vector3};

/// Discrete dynamics world managing bodies, joints, collisions, and integration.
/// Direct equivalent to Bullet's `btDiscreteDynamicsWorld`.
pub struct DiscreteDynamicsWorld {
    pub gravity: Vector3,
    pub bodies: Vec<Option<RigidBody>>,
    pub constraints: Vec<Box<dyn Constraint>>,
    pub broadphase: DbvtBroadphase,
    pub solver: SequentialImpulseConstraintSolver,
    pub manifolds: Vec<ContactManifold>,
    pub time_accumulator: f32,
}

impl DiscreteDynamicsWorld {
    pub fn new() -> Self {
        Self {
            gravity: Vector3::new(0.0, -9.81, 0.0),
            bodies: Vec::new(),
            constraints: Vec::new(),
            broadphase: DbvtBroadphase::new(),
            solver: SequentialImpulseConstraintSolver::new(),
            manifolds: Vec::new(),
            time_accumulator: 0.0,
        }
    }

    pub fn with_gravity(mut self, gravity: Vector3) -> Self {
        self.gravity = gravity;
        self
    }

    pub fn add_rigid_body(&mut self, mut body: RigidBody) -> usize {
        let id = self.bodies.len();
        body.id = id;
        let aabb = body.shape.calculate_world_aabb(&body.transform);
        self.broadphase.insert(id, aabb);
        self.bodies.push(Some(body));
        id
    }

    pub fn remove_rigid_body(&mut self, id: usize) {
        if id < self.bodies.len() {
            self.bodies[id] = None;
            self.broadphase.remove(id);
            self.constraints.retain(|c| c.body_a() != id && c.body_b() != id);
            self.manifolds.retain(|m| m.body_a != id && m.body_b != id);
        }
    }

    pub fn get_rigid_body(&self, id: usize) -> Option<&RigidBody> {
        self.bodies.get(id).and_then(|b| b.as_ref())
    }

    pub fn get_rigid_body_mut(&mut self, id: usize) -> Option<&mut RigidBody> {
        self.bodies.get_mut(id).and_then(|b| b.as_mut())
    }

    pub fn add_constraint<C: Constraint + 'static>(&mut self, constraint: C) {
        self.constraints.push(Box::new(constraint));
    }

    /// Step simulation with fixed sub-stepping for deterministic behavior.
    /// Direct equivalent to Bullet's `stepSimulation`.
    pub fn step_simulation(
        &mut self,
        time_step: f32,
        max_sub_steps: usize,
        fixed_time_step: f32,
    ) -> usize {
        let mut sub_steps = 0;
        if max_sub_steps != 0 {
            self.time_accumulator += time_step;
            if self.time_accumulator >= fixed_time_step {
                sub_steps = (self.time_accumulator / fixed_time_step) as usize;
                sub_steps = sub_steps.min(max_sub_steps);
                self.time_accumulator -= sub_steps as f32 * fixed_time_step;
            }
        } else {
            sub_steps = 1;
        }

        let dt = if max_sub_steps != 0 {
            fixed_time_step
        } else {
            time_step
        };

        for _ in 0..sub_steps {
            self.internal_single_step_simulation(dt);
        }

        sub_steps
    }

    fn internal_single_step_simulation(&mut self, dt: f32) {
        // 1. Predict unconstrained motion: integrate forces & velocities
        for body in self.bodies.iter_mut().flatten() {
            body.integrate_velocities(dt, self.gravity);
        }

        // 2. Refresh existing contact manifolds
        for manifold in &mut self.manifolds {
            let (t_a, t_b) = match (
                self.bodies[manifold.body_a].as_ref(),
                self.bodies[manifold.body_b].as_ref(),
            ) {
                (Some(a), Some(b)) => (a.transform, b.transform),
                _ => continue,
            };
            manifold.refresh_contact_points(&t_a, &t_b);
        }

        // 3. Broadphase collision detection
        let pairs = self.broadphase.compute_pairs();

        // 4. Narrowphase collision detection
        for pair in pairs {
            let (obj_a, obj_b) = match (
                self.bodies[pair.body_a].as_ref(),
                self.bodies[pair.body_b].as_ref(),
            ) {
                (Some(a), Some(b)) => (a, b),
                _ => continue,
            };

            // Skip static-static collisions
            if obj_a.is_static() && obj_b.is_static() {
                continue;
            }

            // Check if bodies are both sleeping
            if !obj_a.is_active && !obj_b.is_active {
                continue;
            }

            if let Some(new_manifold) = detect_collision(
                obj_a.id,
                &obj_a.shape,
                &obj_a.transform,
                obj_b.id,
                &obj_b.shape,
                &obj_b.transform,
            ) {
                // Find existing manifold or insert new
                if let Some(existing) = self
                    .manifolds
                    .iter_mut()
                    .find(|m| m.body_a == new_manifold.body_a && m.body_b == new_manifold.body_b)
                {
                    for pt in new_manifold.points {
                        existing.add_contact_point(pt);
                    }
                } else {
                    self.manifolds.push(new_manifold);
                }
            }
        }

        // Remove empty manifolds
        self.manifolds.retain(|m| !m.points.is_empty());

        // 5. Solve constraints and collisions via Sequential Impulse Solver
        self.solver.solve(
            &mut self.bodies,
            &mut self.manifolds,
            &mut self.constraints,
            dt,
        );

        // 6. Integrate transforms & update broadphase
        for body in self.bodies.iter_mut().flatten() {
            body.integrate_transform(dt);
            body.clear_forces();
            body.update_deactivation(dt);
        }

        // 7. Update AABBs in DBVT broadphase
        for body in self.bodies.iter().flatten() {
            let aabb = body.shape.calculate_world_aabb(&body.transform);
            self.broadphase.update(body.id, aabb);
        }
    }

    /// Raycast test against all bodies in the world.
    pub fn ray_test(&self, ray_from: Vector3, ray_to: Vector3) -> Option<RayTestResult> {
        let candidate_ids = self.broadphase.ray_test(ray_from, ray_to);
        let ray_dir = ray_to - ray_from;
        let ray_len = ray_dir.length();
        if ray_len < 1e-6 {
            return None;
        }

        let mut closest_fraction = 1.0f32;
        let mut best_result = None;

        for id in candidate_ids {
            let body = match self.bodies.get(id) {
                Some(Some(b)) => b,
                _ => continue,
            };

            if let Some((frac, norm)) = ray_test_shape(&body.shape, &body.transform, ray_from, ray_to) {
                if frac < closest_fraction {
                    closest_fraction = frac;
                    best_result = Some(RayTestResult {
                        body_id: id,
                        hit_fraction: frac,
                        hit_point_world: ray_from + ray_dir * frac,
                        hit_normal_world: norm,
                    });
                }
            }
        }

        best_result
    }
}

impl Default for DiscreteDynamicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

fn ray_test_shape(
    shape: &Shape,
    transform: &Transform,
    ray_from: Vector3,
    ray_to: Vector3,
) -> Option<(f32, Vector3)> {
    let local_from = transform.inverse_transform_point(ray_from);
    let local_to = transform.inverse_transform_point(ray_to);
    let local_dir = local_to - local_from;

    match shape {
        Shape::Sphere(s) => {
            let r = s.radius;
            let a = local_dir.dot(local_dir);
            let b = 2.0 * local_from.dot(local_dir);
            let c = local_from.dot(local_from) - r * r;
            let discr = b * b - 4.0 * a * c;

            if discr < 0.0 {
                return None;
            }

            let sqrt_d = discr.sqrt();
            let mut t = (-b - sqrt_d) / (2.0 * a);
            if t < 0.0 {
                t = (-b + sqrt_d) / (2.0 * a);
            }

            if t >= 0.0 && t <= 1.0 {
                let local_hit = local_from + local_dir * t;
                let local_norm = local_hit.normalize();
                let world_norm = transform.transform_vector(local_norm).normalize();
                Some((t, world_norm))
            } else {
                None
            }
        }
        Shape::Box(b) => {
            let aabb = b.calculate_local_aabb();
            let t = aabb.ray_test(local_from, local_to)?;
            if t >= 0.0 && t <= 1.0 {
                let local_hit = local_from + local_dir * t;
                let h = b.half_extents;
                let mut norm = Vector3::Y;
                let mut min_diff = f32::INFINITY;

                for (axis, &val) in [
                    (Vector3::X, &(h.x - local_hit.x.abs())),
                    (Vector3::Y, &(h.y - local_hit.y.abs())),
                    (Vector3::Z, &(h.z - local_hit.z.abs())),
                ] {
                    if val < min_diff {
                        min_diff = val;
                        norm = axis;
                    }
                }
                let world_norm = transform.transform_vector(norm).normalize();
                Some((t, world_norm))
            } else {
                None
            }
        }
        Shape::StaticPlane(p) => {
            let denom = local_dir.dot(p.plane_normal);
            if denom.abs() > 1e-6 {
                let t = (p.plane_constant - local_from.dot(p.plane_normal)) / denom;
                if t >= 0.0 && t <= 1.0 {
                    let world_norm = transform.transform_vector(p.plane_normal).normalize();
                    return Some((t, world_norm));
                }
            }
            None
        }
        _ => {
            let aabb = shape.calculate_local_aabb();
            let t = aabb.ray_test(local_from, local_to)?;
            if t >= 0.0 && t <= 1.0 {
                Some((t, Vector3::Y))
            } else {
                None
            }
        }
    }
}
