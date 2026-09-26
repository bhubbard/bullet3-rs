use crate::collision::shapes::{BoxShape, Shape, SphereShape, StaticPlaneShape};
use crate::linear_math::{Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Contact point between two colliding bodies.
/// Equivalent to Bullet's `btManifoldPoint`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ContactPoint {
    pub local_point_a: Vector3,
    pub local_point_b: Vector3,
    pub position_world_a: Vector3,
    pub position_world_b: Vector3,
    /// Contact normal pointing from B towards A in world space.
    pub normal_world_on_b: Vector3,
    /// Distance between shapes along normal. Negative indicates penetration.
    pub distance: f32,
    /// Normal impulse accumulated by solver across iterations and frames (warm-start).
    pub applied_impulse: f32,
    /// Tangent friction 1 impulse accumulated by solver.
    pub applied_friction_1: f32,
    /// Tangent friction 2 impulse accumulated by solver.
    pub applied_friction_2: f32,
}

impl ContactPoint {
    pub fn new(
        local_point_a: Vector3,
        local_point_b: Vector3,
        position_world_a: Vector3,
        position_world_b: Vector3,
        normal_world_on_b: Vector3,
        distance: f32,
    ) -> Self {
        Self {
            local_point_a,
            local_point_b,
            position_world_a,
            position_world_b,
            normal_world_on_b,
            distance,
            applied_impulse: 0.0,
            applied_friction_1: 0.0,
            applied_friction_2: 0.0,
        }
    }
}

/// Contact manifold holding contact points between two bodies.
/// Equivalent to Bullet's `btPersistentManifold`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ContactManifold {
    pub body_a: usize,
    pub body_b: usize,
    pub points: Vec<ContactPoint>,
}

impl ContactManifold {
    pub fn new(body_a: usize, body_b: usize) -> Self {
        Self {
            body_a,
            body_b,
            points: Vec::with_capacity(4),
        }
    }

    /// Add a contact point, maintaining at most 4 stable contact points.
    pub fn add_contact_point(&mut self, point: ContactPoint) {
        if self.points.len() < 4 {
            self.points.push(point);
            return;
        }

        // Find the point closest to the new point; replace if distance is deeper
        let mut min_dist_sq = f32::INFINITY;
        let mut replace_idx = 0;
        for (i, p) in self.points.iter().enumerate() {
            let d2 = p.position_world_a.distance_squared(point.position_world_a);
            if d2 < min_dist_sq {
                min_dist_sq = d2;
                replace_idx = i;
            }
        }

        if min_dist_sq < 0.001 {
            // Keep warm starting impulses if replacing close point
            let old = self.points[replace_idx];
            let mut updated = point;
            updated.applied_impulse = old.applied_impulse;
            updated.applied_friction_1 = old.applied_friction_1;
            updated.applied_friction_2 = old.applied_friction_2;
            self.points[replace_idx] = updated;
        } else {
            // Replace point with smallest penetration
            let mut deepest_idx = 0;
            let mut max_dist = -f32::INFINITY;
            for (i, p) in self.points.iter().enumerate() {
                if p.distance > max_dist {
                    max_dist = p.distance;
                    deepest_idx = i;
                }
            }
            self.points[deepest_idx] = point;
        }
    }

    /// Refresh contact points from updated body transforms.
    pub fn refresh_contact_points(&mut self, transform_a: &Transform, transform_b: &Transform) {
        self.points.retain_mut(|p| {
            let wa = transform_a.transform_point(p.local_point_a);
            let wb = transform_b.transform_point(p.local_point_b);
            let projected_diff = (wa - wb).dot(p.normal_world_on_b);
            p.position_world_a = wa;
            p.position_world_b = wb;
            p.distance = projected_diff;

            // Retain point if bodies haven't slid or drifted too far apart
            let lateral_diff = (wa - wb) - (p.normal_world_on_b * projected_diff);
            lateral_diff.length_squared() < 0.04 && p.distance < 0.05
        });
    }
}

// ---------------- Narrowphase Dispatcher & Collision Detectors ----------------

pub fn detect_collision(
    body_a: usize,
    shape_a: &Shape,
    transform_a: &Transform,
    body_b: usize,
    shape_b: &Shape,
    transform_b: &Transform,
) -> Option<ContactManifold> {
    match (shape_a, shape_b) {
        (Shape::Sphere(sa), Shape::Sphere(sb)) => {
            collide_sphere_sphere(body_a, sa, transform_a, body_b, sb, transform_b)
        }
        (Shape::Sphere(sa), Shape::StaticPlane(pb)) => {
            collide_sphere_plane(body_a, sa, transform_a, body_b, pb, transform_b)
        }
        (Shape::StaticPlane(pa), Shape::Sphere(sb)) => {
            collide_sphere_plane(body_b, sb, transform_b, body_a, pa, transform_a).map(|mut m| {
                invert_manifold(&mut m);
                m
            })
        }
        (Shape::Sphere(sa), Shape::Box(bb)) => {
            collide_sphere_box(body_a, sa, transform_a, body_b, bb, transform_b)
        }
        (Shape::Box(ba), Shape::Sphere(sb)) => {
            collide_sphere_box(body_b, sb, transform_b, body_a, ba, transform_a).map(|mut m| {
                invert_manifold(&mut m);
                m
            })
        }
        (Shape::Box(ba), Shape::StaticPlane(pb)) => {
            collide_box_plane(body_a, ba, transform_a, body_b, pb, transform_b)
        }
        (Shape::StaticPlane(pa), Shape::Box(bb)) => {
            collide_box_plane(body_b, bb, transform_b, body_a, pa, transform_a).map(|mut m| {
                invert_manifold(&mut m);
                m
            })
        }
        (Shape::Box(ba), Shape::Box(bb)) => {
            collide_box_box(body_a, ba, transform_a, body_b, bb, transform_b)
        }
        // GJK fallback for general convex polyhedra / hulls / capsules
        _ => collide_gjk(body_a, shape_a, transform_a, body_b, shape_b, transform_b),
    }
}

fn invert_manifold(manifold: &mut ContactManifold) {
    std::mem::swap(&mut manifold.body_a, &mut manifold.body_b);
    for p in &mut manifold.points {
        std::mem::swap(&mut p.local_point_a, &mut p.local_point_b);
        std::mem::swap(&mut p.position_world_a, &mut p.position_world_b);
        p.normal_world_on_b = -p.normal_world_on_b;
    }
}

pub fn collide_sphere_sphere(
    body_a: usize,
    sphere_a: &SphereShape,
    t_a: &Transform,
    body_b: usize,
    sphere_b: &SphereShape,
    t_b: &Transform,
) -> Option<ContactManifold> {
    let diff = t_a.origin - t_b.origin;
    let dist_sq = diff.length_squared();
    let sum_r = sphere_a.radius + sphere_b.radius;

    if dist_sq >= sum_r * sum_r {
        return None;
    }

    let (normal_b_to_a, dist) = if dist_sq > 1e-12 {
        let d = dist_sq.sqrt();
        (diff / d, d)
    } else {
        (Vector3::Y, 0.0)
    };

    let penetration = dist - sum_r;
    let pos_a = t_a.origin - normal_b_to_a * sphere_a.radius;
    let pos_b = t_b.origin + normal_b_to_a * sphere_b.radius;

    let mut manifold = ContactManifold::new(body_a, body_b);
    manifold.add_contact_point(ContactPoint::new(
        t_a.inverse_transform_point(pos_a),
        t_b.inverse_transform_point(pos_b),
        pos_a,
        pos_b,
        normal_b_to_a,
        penetration,
    ));

    Some(manifold)
}

pub fn collide_sphere_plane(
    body_a: usize,
    sphere: &SphereShape,
    t_a: &Transform,
    body_b: usize,
    plane: &StaticPlaneShape,
    t_b: &Transform,
) -> Option<ContactManifold> {
    let plane_normal_world = t_b.transform_vector(plane.plane_normal).normalize();
    let plane_point_world = t_b.transform_point(plane.plane_normal * plane.plane_constant);

    let center_to_plane = (t_a.origin - plane_point_world).dot(plane_normal_world);
    let penetration = center_to_plane - sphere.radius;

    if penetration >= 0.0 {
        return None;
    }

    let pos_b = t_a.origin - plane_normal_world * center_to_plane;
    let pos_a = t_a.origin - plane_normal_world * sphere.radius;

    let mut manifold = ContactManifold::new(body_a, body_b);
    manifold.add_contact_point(ContactPoint::new(
        t_a.inverse_transform_point(pos_a),
        t_b.inverse_transform_point(pos_b),
        pos_a,
        pos_b,
        plane_normal_world,
        penetration,
    ));

    Some(manifold)
}

pub fn collide_sphere_box(
    body_a: usize,
    sphere: &SphereShape,
    t_a: &Transform,
    body_b: usize,
    box_b: &BoxShape,
    t_b: &Transform,
) -> Option<ContactManifold> {
    // Transform sphere center into box local space
    let local_sphere_center = t_b.inverse_transform_point(t_a.origin);

    // Clamped point on box in local space
    let clamped_local = Vector3::new(
        local_sphere_center.x.clamp(-box_b.half_extents.x, box_b.half_extents.x),
        local_sphere_center.y.clamp(-box_b.half_extents.y, box_b.half_extents.y),
        local_sphere_center.z.clamp(-box_b.half_extents.z, box_b.half_extents.z),
    );

    let diff = local_sphere_center - clamped_local;
    let dist_sq = diff.length_squared();

    if dist_sq > sphere.radius * sphere.radius {
        return None;
    }

    let (local_normal, dist) = if dist_sq > 1e-10 {
        let d = dist_sq.sqrt();
        (diff / d, d)
    } else {
        // Center is inside box: push along closest axis
        let dx = box_b.half_extents.x - local_sphere_center.x.abs();
        let dy = box_b.half_extents.y - local_sphere_center.y.abs();
        let dz = box_b.half_extents.z - local_sphere_center.z.abs();
        if dx < dy && dx < dz {
            (Vector3::new(local_sphere_center.x.signum(), 0.0, 0.0), -dx)
        } else if dy < dz {
            (Vector3::new(0.0, local_sphere_center.y.signum(), 0.0), -dy)
        } else {
            (Vector3::new(0.0, 0.0, local_sphere_center.z.signum()), -dz)
        }
    };

    let world_normal = t_b.transform_vector(local_normal).normalize();
    let penetration = dist - sphere.radius;

    let pos_b = t_b.transform_point(clamped_local);
    let pos_a = t_a.origin - world_normal * sphere.radius;

    let mut manifold = ContactManifold::new(body_a, body_b);
    manifold.add_contact_point(ContactPoint::new(
        t_a.inverse_transform_point(pos_a),
        clamped_local,
        pos_a,
        pos_b,
        world_normal,
        penetration,
    ));

    Some(manifold)
}

pub fn collide_box_plane(
    body_a: usize,
    box_a: &BoxShape,
    t_a: &Transform,
    body_b: usize,
    plane: &StaticPlaneShape,
    t_b: &Transform,
) -> Option<ContactManifold> {
    let plane_normal_world = t_b.transform_vector(plane.plane_normal).normalize();
    let plane_point_world = t_b.transform_point(plane.plane_normal * plane.plane_constant);

    let h = box_a.half_extents;
    let corners = [
        Vector3::new(h.x, h.y, h.z),
        Vector3::new(-h.x, h.y, h.z),
        Vector3::new(h.x, -h.y, h.z),
        Vector3::new(-h.x, -h.y, h.z),
        Vector3::new(h.x, h.y, -h.z),
        Vector3::new(-h.x, h.y, -h.z),
        Vector3::new(h.x, -h.y, -h.z),
        Vector3::new(-h.x, -h.y, -h.z),
    ];

    let mut manifold = ContactManifold::new(body_a, body_b);

    for corner_local in &corners {
        let corner_world = t_a.transform_point(*corner_local);
        let dist = (corner_world - plane_point_world).dot(plane_normal_world);

        if dist < 0.0 {
            let pos_b = corner_world - plane_normal_world * dist;
            manifold.add_contact_point(ContactPoint::new(
                *corner_local,
                t_b.inverse_transform_point(pos_b),
                corner_world,
                pos_b,
                plane_normal_world,
                dist,
            ));
        }
    }

    if manifold.points.is_empty() {
        None
    } else {
        Some(manifold)
    }
}

/// Separating Axis Theorem (SAT) collision detector for two boxes.
pub fn collide_box_box(
    body_a: usize,
    box_a: &BoxShape,
    t_a: &Transform,
    body_b: usize,
    box_b: &BoxShape,
    t_b: &Transform,
) -> Option<ContactManifold> {
    let axes_a = [t_a.basis.column(0), t_a.basis.column(1), t_a.basis.column(2)];
    let axes_b = [t_b.basis.column(0), t_b.basis.column(1), t_b.basis.column(2)];
    let translation = t_a.origin - t_b.origin;

    let mut min_penetration = f32::INFINITY;
    let mut best_axis = Vector3::ZERO;

    // Test 15 potential separating axes: 3 face normals of A, 3 face normals of B, 9 edge cross products
    for axis in &axes_a {
        let pen = test_sat_axis(*axis, box_a, t_a, box_b, t_b, translation)?;
        if pen < min_penetration {
            min_penetration = pen;
            best_axis = *axis;
        }
    }

    for axis in &axes_b {
        let pen = test_sat_axis(*axis, box_a, t_a, box_b, t_b, translation)?;
        if pen < min_penetration {
            min_penetration = pen;
            best_axis = *axis;
        }
    }

    for a in &axes_a {
        for b in &axes_b {
            let cross = a.cross(*b);
            if cross.length_squared() > 1e-6 {
                let norm_cross = cross.normalize();
                let pen = test_sat_axis(norm_cross, box_a, t_a, box_b, t_b, translation)?;
                if pen < min_penetration {
                    min_penetration = pen;
                    best_axis = norm_cross;
                }
            }
        }
    }

    // Ensure normal points from B to A
    if best_axis.dot(translation) < 0.0 {
        best_axis = -best_axis;
    }

    // Generate contact points on surface
    let mut manifold = ContactManifold::new(body_a, body_b);
    let contact_b = t_b.origin + best_axis * (box_b.half_extents.x.min(box_b.half_extents.y).min(box_b.half_extents.z));
    let contact_a = contact_b + best_axis * min_penetration;

    manifold.add_contact_point(ContactPoint::new(
        t_a.inverse_transform_point(contact_a),
        t_b.inverse_transform_point(contact_b),
        contact_a,
        contact_b,
        best_axis,
        -min_penetration,
    ));

    Some(manifold)
}

fn test_sat_axis(
    axis: Vector3,
    box_a: &BoxShape,
    t_a: &Transform,
    box_b: &BoxShape,
    t_b: &Transform,
    translation: Vector3,
) -> Option<f32> {
    let r_a = project_box(box_a, t_a, axis);
    let r_b = project_box(box_b, t_b, axis);
    let dist = translation.dot(axis).abs();

    let penetration = (r_a + r_b) - dist;
    if penetration > 0.0 {
        Some(penetration)
    } else {
        None
    }
}

fn project_box(b: &BoxShape, t: &Transform, axis: Vector3) -> f32 {
    let m = &t.basis.m;
    let local_axis = Vector3::new(
        m[0][0] * axis.x + m[1][0] * axis.y + m[2][0] * axis.z,
        m[0][1] * axis.x + m[1][1] * axis.y + m[2][1] * axis.z,
        m[0][2] * axis.x + m[1][2] * axis.y + m[2][2] * axis.z,
    );
    b.half_extents.x * local_axis.x.abs()
        + b.half_extents.y * local_axis.y.abs()
        + b.half_extents.z * local_axis.z.abs()
}

/// Gilbert-Johnson-Keerthi (GJK) convex collision detector.
pub fn collide_gjk(
    body_a: usize,
    shape_a: &Shape,
    t_a: &Transform,
    body_b: usize,
    shape_b: &Shape,
    t_b: &Transform,
) -> Option<ContactManifold> {
    let mut direction = t_a.origin - t_b.origin;
    if direction.length_squared() < 1e-6 {
        direction = Vector3::Y;
    }

    let mut simplex: Vec<Vector3> = Vec::with_capacity(4);
    let mut support_a_pts: Vec<Vector3> = Vec::with_capacity(4);
    let mut support_b_pts: Vec<Vector3> = Vec::with_capacity(4);

    let (sup, sa, sb) = support_minkowski(shape_a, t_a, shape_b, t_b, direction);
    simplex.push(sup);
    support_a_pts.push(sa);
    support_b_pts.push(sb);
    direction = -sup;

    let mut max_iters = 30;
    while max_iters > 0 {
        max_iters -= 1;
        let (new_sup, new_sa, new_sb) = support_minkowski(shape_a, t_a, shape_b, t_b, direction);
        if new_sup.dot(direction) < 0.0 {
            return None; // No collision
        }

        simplex.push(new_sup);
        support_a_pts.push(new_sa);
        support_b_pts.push(new_sb);

        if update_simplex_and_direction(&mut simplex, &mut support_a_pts, &mut support_b_pts, &mut direction) {
            // Collision detected! Simplex contains the origin.
            let normal = (t_a.origin - t_b.origin).normalize();
            let pos_a = support_a_pts.last().copied().unwrap_or(t_a.origin);
            let pos_b = support_b_pts.last().copied().unwrap_or(t_b.origin);
            let depth = -0.01; // Conservative penetration distance

            let mut manifold = ContactManifold::new(body_a, body_b);
            manifold.add_contact_point(ContactPoint::new(
                t_a.inverse_transform_point(pos_a),
                t_b.inverse_transform_point(pos_b),
                pos_a,
                pos_b,
                normal,
                depth,
            ));
            return Some(manifold);
        }
    }

    None
}

fn support_minkowski(
    shape_a: &Shape,
    t_a: &Transform,
    shape_b: &Shape,
    t_b: &Transform,
    dir: Vector3,
) -> (Vector3, Vector3, Vector3) {
    let local_dir_a = t_a.inverse_transform_vector(dir);
    let local_dir_b = t_b.inverse_transform_vector(-dir);

    let local_sup_a = shape_a.local_supporting_vertex(local_dir_a);
    let local_sup_b = shape_b.local_supporting_vertex(local_dir_b);

    let world_sup_a = t_a.transform_point(local_sup_a);
    let world_sup_b = t_b.transform_point(local_sup_b);

    (world_sup_a - world_sup_b, world_sup_a, world_sup_b)
}

fn update_simplex_and_direction(
    simplex: &mut Vec<Vector3>,
    _sa: &mut Vec<Vector3>,
    _sb: &mut Vec<Vector3>,
    direction: &mut Vector3,
) -> bool {
    let len = simplex.len();
    if len == 2 {
        let a = simplex[1];
        let b = simplex[0];
        let ab = b - a;
        let ao = -a;
        if ab.dot(ao) > 0.0 {
            *direction = ab.cross(ao).cross(ab);
            if direction.length_squared() < 1e-8 {
                *direction = Vector3::new(-ab.y, ab.x, 0.0);
            }
        } else {
            simplex.remove(0);
            *direction = ao;
        }
        false
    } else if len == 3 {
        let a = simplex[2];
        let b = simplex[1];
        let c = simplex[0];
        let ab = b - a;
        let ac = c - a;
        let ao = -a;
        let abc = ab.cross(ac);

        if abc.cross(ac).dot(ao) > 0.0 {
            if ac.dot(ao) > 0.0 {
                simplex.remove(1); // remove B
                *direction = ac.cross(ao).cross(ac);
            } else if ab.dot(ao) > 0.0 {
                simplex.remove(0); // remove C
                *direction = ab.cross(ao).cross(ab);
            } else {
                simplex.remove(0);
                simplex.remove(0);
                *direction = ao;
            }
            false
        } else if ab.cross(abc).dot(ao) > 0.0 {
            if ab.dot(ao) > 0.0 {
                simplex.remove(0); // remove C
                *direction = ab.cross(ao).cross(ab);
            } else {
                simplex.remove(0);
                simplex.remove(0);
                *direction = ao;
            }
            false
        } else {
            if abc.dot(ao) > 0.0 {
                *direction = abc;
            } else {
                simplex.swap(0, 1);
                *direction = -abc;
            }
            false
        }
    } else if len == 4 {
        // Tetrahedron simplex test
        let a = simplex[3];
        let b = simplex[2];
        let c = simplex[1];
        let d = simplex[0];
        let ao = -a;

        let abc = (b - a).cross(c - a);
        let acd = (c - a).cross(d - a);
        let adb = (d - a).cross(b - a);

        if abc.dot(ao) > 0.0 {
            simplex.remove(0); // remove D
            *direction = abc;
            false
        } else if acd.dot(ao) > 0.0 {
            simplex.remove(2); // remove B
            *direction = acd;
            false
        } else if adb.dot(ao) > 0.0 {
            simplex.remove(1); // remove C
            *direction = adb;
            false
        } else {
            // Origin is inside the tetrahedron simplex!
            true
        }
    } else {
        false
    }
}
