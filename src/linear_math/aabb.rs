use super::transform::Transform;
use super::vector3::Vector3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Axis-Aligned Bounding Box (AABB) for broadphase and quick culling.
/// Direct equivalent to Bullet's `btAabb` / `btVector3 min, max`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Aabb {
    pub min: Vector3,
    pub max: Vector3,
}

impl Aabb {
    pub const EMPTY: Self = Self {
        min: Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
        max: Vector3::new(-f32::INFINITY, -f32::INFINITY, -f32::INFINITY),
    };

    #[inline]
    pub const fn new(min: Vector3, max: Vector3) -> Self {
        Self { min, max }
    }

    #[inline]
    pub fn from_center_half_extents(center: Vector3, half_extents: Vector3) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    #[inline]
    pub fn center(&self) -> Vector3 {
        (self.min + self.max) * 0.5
    }

    #[inline]
    pub fn half_extents(&self) -> Vector3 {
        (self.max - self.min) * 0.5
    }

    #[inline]
    pub fn extents(&self) -> Vector3 {
        self.max - self.min
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z
    }

    #[inline]
    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    #[inline]
    pub fn contains_point(&self, p: Vector3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    #[inline]
    pub fn contains_aabb(&self, other: &Self) -> bool {
        other.min.x >= self.min.x
            && other.max.x <= self.max.x
            && other.min.y >= self.min.y
            && other.max.y <= self.max.y
            && other.min.z >= self.min.z
            && other.max.z <= self.max.z
    }

    #[inline]
    pub fn merge(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    #[inline]
    pub fn expand(&mut self, margin: f32) {
        let v = Vector3::splat(margin);
        self.min -= v;
        self.max += v;
    }

    #[inline]
    pub fn expanded(&self, margin: f32) -> Self {
        let mut copy = *self;
        copy.expand(margin);
        copy
    }

    /// Transforms the AABB by a rigid Transform and returns a conservative world-space AABB.
    pub fn transform(&self, t: &Transform) -> Self {
        let center = self.center();
        let half = self.half_extents();

        let world_center = t.transform_point(center);

        // Compute new half extents using absolute values of rotation matrix
        let m = &t.basis.m;
        let world_half = Vector3::new(
            m[0][0].abs() * half.x + m[0][1].abs() * half.y + m[0][2].abs() * half.z,
            m[1][0].abs() * half.x + m[1][1].abs() * half.y + m[1][2].abs() * half.z,
            m[2][0].abs() * half.x + m[2][1].abs() * half.y + m[2][2].abs() * half.z,
        );

        Self::from_center_half_extents(world_center, world_half)
    }

    /// Ray intersection test using the Kay-Kajiya slab algorithm.
    /// Returns the normalized distance parameter `t` in `[0, 1]` if intersecting.
    pub fn ray_test(&self, ray_from: Vector3, ray_to: Vector3) -> Option<f32> {
        let dir = ray_to - ray_from;
        let mut tmin = 0.0f32;
        let mut tmax = 1.0f32;

        for i in 0..3 {
            let d = dir[i];
            let from = ray_from[i];
            let min_val = self.min[i];
            let max_val = self.max[i];

            if d.abs() < 1e-8 {
                if from < min_val || from > max_val {
                    return None;
                }
            } else {
                let inv_d = 1.0 / d;
                let mut t1 = (min_val - from) * inv_d;
                let mut t2 = (max_val - from) * inv_d;

                if t1 > t2 {
                    std::mem::swap(&mut t1, &mut t2);
                }

                tmin = tmin.max(t1);
                tmax = tmax.min(t2);

                if tmin > tmax {
                    return None;
                }
            }
        }

        Some(tmin)
    }
}

impl Default for Aabb {
    #[inline]
    fn default() -> Self {
        Self::EMPTY
    }
}
