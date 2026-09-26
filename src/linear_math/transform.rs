use super::matrix3x3::Matrix3x3;
use super::quaternion::Quaternion;
use super::vector3::Vector3;
use std::ops::Mul;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Rigid 3D transform combining rotation (basis matrix) and translation (origin vector).
/// Direct equivalent to Bullet's `btTransform`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Transform {
    pub basis: Matrix3x3,
    pub origin: Vector3,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        basis: Matrix3x3::IDENTITY,
        origin: Vector3::ZERO,
    };

    #[inline]
    pub const fn new(basis: Matrix3x3, origin: Vector3) -> Self {
        Self { basis, origin }
    }

    #[inline]
    pub const fn from_translation(origin: Vector3) -> Self {
        Self {
            basis: Matrix3x3::IDENTITY,
            origin,
        }
    }

    #[inline]
    pub fn from_rotation(basis: Matrix3x3) -> Self {
        Self {
            basis,
            origin: Vector3::ZERO,
        }
    }

    #[inline]
    pub fn from_rotation_quaternion(origin: Vector3, rotation: Quaternion) -> Self {
        Self {
            basis: Matrix3x3::from_quaternion(rotation),
            origin,
        }
    }

    #[inline]
    pub fn rotation(&self) -> Quaternion {
        self.basis.to_quaternion()
    }

    #[inline]
    pub fn set_rotation(&mut self, q: Quaternion) {
        self.basis = Matrix3x3::from_quaternion(q);
    }

    #[inline]
    pub fn transform_point(&self, point: Vector3) -> Vector3 {
        (self.basis * point) + self.origin
    }

    #[inline]
    pub fn transform_vector(&self, vector: Vector3) -> Vector3 {
        self.basis * vector
    }

    #[inline]
    pub fn inverse_transform_point(&self, point: Vector3) -> Vector3 {
        self.basis.transpose() * (point - self.origin)
    }

    #[inline]
    pub fn inverse_transform_vector(&self, vector: Vector3) -> Vector3 {
        self.basis.transpose() * vector
    }

    #[inline]
    pub fn inverse(&self) -> Self {
        let inv_basis = self.basis.transpose();
        Self {
            basis: inv_basis,
            origin: -(inv_basis * self.origin),
        }
    }
}

impl Default for Transform {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mul for Transform {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self {
            basis: self.basis * rhs.basis,
            origin: self.transform_point(rhs.origin),
        }
    }
}

impl Mul<Vector3> for Transform {
    type Output = Vector3;
    #[inline]
    fn mul(self, rhs: Vector3) -> Vector3 {
        self.transform_point(rhs)
    }
}
