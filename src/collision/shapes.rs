use crate::linear_math::{Aabb, Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Type of collision shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ShapeType {
    Sphere,
    Box,
    Capsule,
    Cylinder,
    StaticPlane,
    ConvexHull,
    Compound,
    Cone,
}

/// Dynamic collision shape enum providing fast dispatch and zero-allocation handling.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Shape {
    Sphere(SphereShape),
    Box(BoxShape),
    Capsule(CapsuleShape),
    Cylinder(CylinderShape),
    StaticPlane(StaticPlaneShape),
    ConvexHull(ConvexHullShape),
    Compound(CompoundShape),
    Cone(ConeShape),
}

impl Shape {
    pub fn shape_type(&self) -> ShapeType {
        match self {
            Shape::Sphere(_) => ShapeType::Sphere,
            Shape::Box(_) => ShapeType::Box,
            Shape::Capsule(_) => ShapeType::Capsule,
            Shape::Cylinder(_) => ShapeType::Cylinder,
            Shape::StaticPlane(_) => ShapeType::StaticPlane,
            Shape::ConvexHull(_) => ShapeType::ConvexHull,
            Shape::Compound(_) => ShapeType::Compound,
            Shape::Cone(_) => ShapeType::Cone,
        }
    }

    /// Calculate principal moments of inertia along body axes.
    /// Equivalent to Bullet's `calculateLocalInertia`.
    pub fn calculate_local_inertia(&self, mass: f32) -> Vector3 {
        if mass <= 0.0 {
            return Vector3::ZERO;
        }
        match self {
            Shape::Sphere(s) => s.calculate_local_inertia(mass),
            Shape::Box(b) => b.calculate_local_inertia(mass),
            Shape::Capsule(c) => c.calculate_local_inertia(mass),
            Shape::Cylinder(cyl) => cyl.calculate_local_inertia(mass),
            Shape::StaticPlane(_) => Vector3::ZERO,
            Shape::ConvexHull(hull) => hull.calculate_local_inertia(mass),
            Shape::Compound(comp) => comp.calculate_local_inertia(mass),
            Shape::Cone(cone) => cone.calculate_local_inertia(mass),
        }
    }

    /// Compute local AABB.
    pub fn calculate_local_aabb(&self) -> Aabb {
        match self {
            Shape::Sphere(s) => s.calculate_local_aabb(),
            Shape::Box(b) => b.calculate_local_aabb(),
            Shape::Capsule(c) => c.calculate_local_aabb(),
            Shape::Cylinder(cyl) => cyl.calculate_local_aabb(),
            Shape::StaticPlane(p) => p.calculate_local_aabb(),
            Shape::ConvexHull(hull) => hull.calculate_local_aabb(),
            Shape::Compound(comp) => comp.calculate_local_aabb(),
            Shape::Cone(cone) => cone.calculate_local_aabb(),
        }
    }

    /// Compute transformed world-space AABB.
    pub fn calculate_world_aabb(&self, transform: &Transform) -> Aabb {
        match self {
            Shape::Sphere(s) => {
                let r = s.radius + s.margin;
                let center = transform.origin;
                Aabb::new(center - Vector3::splat(r), center + Vector3::splat(r))
            }
            Shape::Box(_)
            | Shape::ConvexHull(_)
            | Shape::Compound(_)
            | Shape::Capsule(_)
            | Shape::Cylinder(_)
            | Shape::Cone(_) => {
                let local = self.calculate_local_aabb();
                local.transform(transform)
            }
            Shape::StaticPlane(p) => p.calculate_world_aabb(transform),
        }
    }

    /// Extreme point in direction `dir` (support mapping for GJK/EPA).
    pub fn local_supporting_vertex(&self, dir: Vector3) -> Vector3 {
        match self {
            Shape::Sphere(s) => s.local_supporting_vertex(dir),
            Shape::Box(b) => b.local_supporting_vertex(dir),
            Shape::Capsule(c) => c.local_supporting_vertex(dir),
            Shape::Cylinder(cyl) => cyl.local_supporting_vertex(dir),
            Shape::StaticPlane(p) => p.local_supporting_vertex(dir),
            Shape::ConvexHull(hull) => hull.local_supporting_vertex(dir),
            Shape::Compound(_) => Vector3::ZERO,
            Shape::Cone(cone) => cone.local_supporting_vertex(dir),
        }
    }

    pub fn margin(&self) -> f32 {
        match self {
            Shape::Sphere(s) => s.margin,
            Shape::Box(b) => b.margin,
            Shape::Capsule(c) => c.margin,
            Shape::Cylinder(cyl) => cyl.margin,
            Shape::StaticPlane(_) => 0.0,
            Shape::ConvexHull(hull) => hull.margin,
            Shape::Compound(_) => 0.0,
            Shape::Cone(cone) => cone.margin,
        }
    }
}

/// Sphere collision shape (`btSphereShape`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SphereShape {
    pub radius: f32,
    pub margin: f32,
}

impl SphereShape {
    pub fn new(radius: f32) -> Self {
        Self {
            radius,
            margin: 0.04,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vector3 {
        // I = 2/5 * m * r^2
        let elem = 0.4 * mass * self.radius * self.radius;
        Vector3::splat(elem)
    }

    pub fn calculate_local_aabb(&self) -> Aabb {
        let r = self.radius + self.margin;
        Aabb::new(-Vector3::splat(r), Vector3::splat(r))
    }

    pub fn local_supporting_vertex(&self, dir: Vector3) -> Vector3 {
        let norm = dir.normalize();
        norm * (self.radius + self.margin)
    }
}

/// Box collision shape (`btBoxShape`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BoxShape {
    pub half_extents: Vector3,
    pub margin: f32,
}

impl BoxShape {
    pub fn new(half_extents: Vector3) -> Self {
        Self {
            half_extents,
            margin: 0.04,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vector3 {
        // Ix = m/12 * (4*hy^2 + 4*hz^2) = m/3 * (hy^2 + hz^2)
        let lx = 2.0 * self.half_extents.x;
        let ly = 2.0 * self.half_extents.y;
        let lz = 2.0 * self.half_extents.z;
        let factor = mass / 12.0;
        Vector3::new(
            factor * (ly * ly + lz * lz),
            factor * (lx * lx + lz * lz),
            factor * (lx * lx + ly * ly),
        )
    }

    pub fn calculate_local_aabb(&self) -> Aabb {
        let ext = self.half_extents + Vector3::splat(self.margin);
        Aabb::new(-ext, ext)
    }

    pub fn local_supporting_vertex(&self, dir: Vector3) -> Vector3 {
        Vector3::new(
            if dir.x < 0.0 { -self.half_extents.x } else { self.half_extents.x },
            if dir.y < 0.0 { -self.half_extents.y } else { self.half_extents.y },
            if dir.z < 0.0 { -self.half_extents.z } else { self.half_extents.z },
        )
    }
}

/// Capsule collision shape (`btCapsuleShape`).
/// Aligned along the Y axis by default.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CapsuleShape {
    pub radius: f32,
    pub half_height: f32, // Distance from center to cylinder edge along Y
    pub margin: f32,
}

impl CapsuleShape {
    pub fn new(radius: f32, height: f32) -> Self {
        Self {
            radius,
            half_height: height * 0.5,
            margin: 0.04,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vector3 {
        let lx = 2.0 * self.radius;
        let ly = 2.0 * (self.half_height + self.radius);
        let lz = 2.0 * self.radius;
        let factor = mass / 12.0;
        Vector3::new(
            factor * (ly * ly + lz * lz),
            factor * (lx * lx + lz * lz),
            factor * (lx * lx + ly * ly),
        )
    }

    pub fn calculate_local_aabb(&self) -> Aabb {
        let r = self.radius + self.margin;
        let ext = Vector3::new(r, self.half_height + r, r);
        Aabb::new(-ext, ext)
    }

    pub fn local_supporting_vertex(&self, dir: Vector3) -> Vector3 {
        let norm = dir.normalize();
        let center_y = if dir.y > 0.0 {
            self.half_height
        } else {
            -self.half_height
        };
        Vector3::new(norm.x * self.radius, center_y + norm.y * self.radius, norm.z * self.radius)
    }
}

/// Cylinder collision shape (`btCylinderShape`).
/// Aligned along the Y axis by default.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CylinderShape {
    pub half_extents: Vector3,
    pub margin: f32,
}

impl CylinderShape {
    pub fn new(half_extents: Vector3) -> Self {
        Self {
            half_extents,
            margin: 0.04,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vector3 {
        let r = self.half_extents.x;
        let h = self.half_extents.y * 2.0;
        let iy = 0.5 * mass * r * r;
        let ixz = (mass / 12.0) * (3.0 * r * r + h * h);
        Vector3::new(ixz, iy, ixz)
    }

    pub fn calculate_local_aabb(&self) -> Aabb {
        let ext = self.half_extents + Vector3::splat(self.margin);
        Aabb::new(-ext, ext)
    }

    pub fn local_supporting_vertex(&self, dir: Vector3) -> Vector3 {
        let radius = self.half_extents.x;
        let half_h = self.half_extents.y;
        let planar = Vector3::new(dir.x, 0.0, dir.z);
        let planar_len = planar.length();
        let (px, pz) = if planar_len > 1e-6 {
            (dir.x / planar_len * radius, dir.z / planar_len * radius)
        } else {
            (0.0, 0.0)
        };
        let py = if dir.y > 0.0 { half_h } else { -half_h };
        Vector3::new(px, py, pz)
    }
}

/// Static infinite plane (`btStaticPlaneShape`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct StaticPlaneShape {
    pub plane_normal: Vector3,
    pub plane_constant: f32,
}

impl StaticPlaneShape {
    pub fn new(plane_normal: Vector3, plane_constant: f32) -> Self {
        Self {
            plane_normal: plane_normal.normalize(),
            plane_constant,
        }
    }

    pub fn calculate_local_aabb(&self) -> Aabb {
        Aabb::new(
            Vector3::splat(-1e6),
            Vector3::splat(1e6),
        )
    }

    pub fn calculate_world_aabb(&self, _t: &Transform) -> Aabb {
        self.calculate_local_aabb()
    }

    pub fn local_supporting_vertex(&self, _dir: Vector3) -> Vector3 {
        self.plane_normal * self.plane_constant
    }
}

/// Arbitrary convex polyhedron shape defined by vertex cloud (`btConvexHullShape`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConvexHullShape {
    pub vertices: Vec<Vector3>,
    pub margin: f32,
}

impl ConvexHullShape {
    pub fn new(vertices: Vec<Vector3>) -> Self {
        Self {
            vertices,
            margin: 0.04,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vector3 {
        // Approximate inertia via bounding box
        let aabb = self.calculate_local_aabb();
        let half = aabb.half_extents();
        let box_shape = BoxShape::new(half);
        box_shape.calculate_local_inertia(mass)
    }

    pub fn calculate_local_aabb(&self) -> Aabb {
        if self.vertices.is_empty() {
            return Aabb::new(-Vector3::splat(self.margin), Vector3::splat(self.margin));
        }
        let mut min = Vector3::splat(f32::INFINITY);
        let mut max = Vector3::splat(-f32::INFINITY);
        for &v in &self.vertices {
            min = min.min(v);
            max = max.max(v);
        }
        Aabb::new(min - Vector3::splat(self.margin), max + Vector3::splat(self.margin))
    }

    pub fn local_supporting_vertex(&self, dir: Vector3) -> Vector3 {
        if self.vertices.is_empty() {
            return Vector3::ZERO;
        }
        let mut max_dot = -f32::INFINITY;
        let mut best_v = self.vertices[0];
        for &v in &self.vertices {
            let dot = v.dot(dir);
            if dot > max_dot {
                max_dot = dot;
                best_v = v;
            }
        }
        if self.margin > 0.0 {
            best_v + dir.normalize() * self.margin
        } else {
            best_v
        }
    }
}

/// Compound shape consisting of multiple child shapes with local transforms (`btCompoundShape`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CompoundShape {
    pub children: Vec<(Transform, Shape)>,
}

impl CompoundShape {
    pub fn new() -> Self {
        Self {
            children: Vec::new(),
        }
    }

    pub fn add_child_shape(&mut self, local_transform: Transform, shape: Shape) {
        self.children.push((local_transform, shape));
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vector3 {
        if self.children.is_empty() || mass <= 0.0 {
            return Vector3::ZERO;
        }
        let child_mass = mass / self.children.len() as f32;
        let mut inertia = Vector3::ZERO;
        for (tr, sh) in &self.children {
            let child_in = sh.calculate_local_inertia(child_mass);
            // Parallel axis theorem: I = I_cm + m * r^2
            let added = Vector3::new(
                child_mass * (tr.origin.y * tr.origin.y + tr.origin.z * tr.origin.z),
                child_mass * (tr.origin.x * tr.origin.x + tr.origin.z * tr.origin.z),
                child_mass * (tr.origin.x * tr.origin.x + tr.origin.y * tr.origin.y),
            );
            inertia += child_in + added;
        }
        inertia
    }

    pub fn calculate_local_aabb(&self) -> Aabb {
        let mut aabb = Aabb::EMPTY;
        for (tr, sh) in &self.children {
            let child_aabb = sh.calculate_world_aabb(tr);
            aabb = aabb.merge(&child_aabb);
        }
        aabb
    }
}

impl Default for CompoundShape {
    fn default() -> Self {
        Self::new()
    }
}

/// Cone collision shape aligned along the Y axis (`btConeShape`).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConeShape {
    pub radius: f32,
    pub height: f32,
    pub margin: f32,
}

impl ConeShape {
    pub fn new(radius: f32, height: f32) -> Self {
        Self {
            radius,
            height,
            margin: 0.04,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vector3 {
        let r = self.radius;
        let h = self.height;
        let iy = 0.3 * mass * r * r;
        let ixz = (3.0 / 80.0) * mass * (4.0 * r * r + h * h);
        Vector3::new(ixz, iy, ixz)
    }

    pub fn calculate_local_aabb(&self) -> Aabb {
        let half_h = self.height * 0.5 + self.margin;
        let r = self.radius + self.margin;
        Aabb::new(Vector3::new(-r, -half_h, -r), Vector3::new(r, half_h, r))
    }

    pub fn local_supporting_vertex(&self, dir: Vector3) -> Vector3 {
        let half_h = self.height * 0.5;
        // Apex is at (0, half_h, 0)
        let apex = Vector3::new(0.0, half_h, 0.0);
        let apex_dot = apex.dot(dir);

        // Base disk at y = -half_h
        let planar = Vector3::new(dir.x, 0.0, dir.z);
        let planar_len = planar.length();
        let base_pt = if planar_len > 1e-6 {
            Vector3::new(
                dir.x / planar_len * self.radius,
                -half_h,
                dir.z / planar_len * self.radius,
            )
        } else {
            Vector3::new(0.0, -half_h, 0.0)
        };

        let base_dot = base_pt.dot(dir);
        let best = if apex_dot > base_dot { apex } else { base_pt };

        if self.margin > 0.0 {
            best + dir.normalize() * self.margin
        } else {
            best
        }
    }
}
