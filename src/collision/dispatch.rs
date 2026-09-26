use crate::collision::broadphase::DbvtBroadphase;
use crate::collision::narrowphase::{detect_collision, ContactManifold};
use crate::collision::shapes::Shape;
use crate::linear_math::{Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum CollisionFlags {
    Static = 1,
    Kinematic = 2,
    Dynamic = 4,
    Ghost = 8,
}

/// Generic collision object. Equivalent to Bullet's `btCollisionObject`.
#[derive(Debug, Clone)]
pub struct CollisionObject {
    pub id: usize,
    pub shape: Shape,
    pub transform: Transform,
    pub friction: f32,
    pub restitution: f32,
    pub collision_flags: CollisionFlags,
    pub collision_filter_group: i16,
    pub collision_filter_mask: i16,
    pub user_index: i32,
}

impl CollisionObject {
    pub fn new(id: usize, shape: Shape, transform: Transform) -> Self {
        Self {
            id,
            shape,
            transform,
            friction: 0.5,
            restitution: 0.0,
            collision_flags: CollisionFlags::Dynamic,
            collision_filter_group: 1,
            collision_filter_mask: -1,
            user_index: -1,
        }
    }
}

/// Sensor / Trigger ghost object that tracks overlapping objects without collision response.
/// Direct equivalent to Bullet's `btGhostObject` / `btPairCachingGhostObject`.
#[derive(Debug, Clone)]
pub struct GhostObject {
    pub id: usize,
    pub shape: Shape,
    pub transform: Transform,
    pub overlapping_objects: Vec<usize>,
}

impl GhostObject {
    pub fn new(id: usize, shape: Shape, transform: Transform) -> Self {
        Self {
            id,
            shape,
            transform,
            overlapping_objects: Vec::new(),
        }
    }
}

/// Ray test hit result. Equivalent to Bullet's `RayResultCallback`.
#[derive(Debug, Clone, Copy)]
pub struct RayTestResult {
    pub body_id: usize,
    pub hit_fraction: f32,
    pub hit_point_world: Vector3,
    pub hit_normal_world: Vector3,
}

/// High-level collision world managing broadphase and narrowphase dispatch.
/// Equivalent to Bullet's `btCollisionWorld`.
pub struct CollisionWorld {
    pub objects: Vec<Option<CollisionObject>>,
    pub broadphase: DbvtBroadphase,
}

impl CollisionWorld {
    pub fn new() -> Self {
        Self {
            objects: Vec::new(),
            broadphase: DbvtBroadphase::new(),
        }
    }

    pub fn add_collision_object(&mut self, mut object: CollisionObject) -> usize {
        let id = self.objects.len();
        object.id = id;
        let aabb = object.shape.calculate_world_aabb(&object.transform);
        self.broadphase.insert(id, aabb);
        self.objects.push(Some(object));
        id
    }

    pub fn remove_collision_object(&mut self, id: usize) {
        if id < self.objects.len() {
            self.objects[id] = None;
            self.broadphase.remove(id);
        }
    }

    pub fn update_single_aabb(&mut self, id: usize) {
        if let Some(Some(obj)) = self.objects.get(id) {
            let aabb = obj.shape.calculate_world_aabb(&obj.transform);
            self.broadphase.update(id, aabb);
        }
    }

    pub fn perform_discrete_collision_detection(&mut self) -> Vec<ContactManifold> {
        let pairs = self.broadphase.compute_pairs();
        let mut manifolds = Vec::new();

        for pair in pairs {
            let (obj_a, obj_b) = match (self.objects.get(pair.body_a), self.objects.get(pair.body_b)) {
                (Some(Some(a)), Some(Some(b))) => (a, b),
                _ => continue,
            };

            // Skip static-static collisions
            if obj_a.collision_flags == CollisionFlags::Static && obj_b.collision_flags == CollisionFlags::Static {
                continue;
            }

            if let Some(manifold) = detect_collision(
                obj_a.id,
                &obj_a.shape,
                &obj_a.transform,
                obj_b.id,
                &obj_b.shape,
                &obj_b.transform,
            ) {
                manifolds.push(manifold);
            }
        }

        manifolds
    }

    /// Ray test query against all objects in the world.
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
            let obj = match self.objects.get(id) {
                Some(Some(o)) => o,
                _ => continue,
            };

            // Analytic shape ray intersection
            if let Some((frac, norm)) = ray_test_shape(&obj.shape, &obj.transform, ray_from, ray_to) {
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

impl Default for CollisionWorld {
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
            // Ray-sphere intersection
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

            if (0.0..=1.0).contains(&t) {
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
            if (0.0..=1.0).contains(&t) {
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
                if (0.0..=1.0).contains(&t) {
                    let world_norm = transform.transform_vector(p.plane_normal).normalize();
                    return Some((t, world_norm));
                }
            }
            None
        }
        _ => {
            // General local AABB fallback
            let aabb = shape.calculate_local_aabb();
            let t = aabb.ray_test(local_from, local_to)?;
            if (0.0..=1.0).contains(&t) {
                Some((t, Vector3::Y))
            } else {
                None
            }
        }
    }
}
