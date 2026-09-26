use super::vector3::Vector3;
use std::ops::{Add, Mul, Neg, Sub};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A unit quaternion representing 3D rotations without gimbal lock.
/// Direct equivalent to Bullet's `btQuaternion`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quaternion {
    pub const IDENTITY: Self = Self::new(0.0, 0.0, 0.0, 1.0);

    #[inline]
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    #[inline]
    pub fn from_axis_angle(axis: Vector3, angle_radians: f32) -> Self {
        let half_angle = angle_radians * 0.5;
        let s = half_angle.sin();
        let norm_axis = axis.normalize();
        Self {
            x: norm_axis.x * s,
            y: norm_axis.y * s,
            z: norm_axis.z * s,
            w: half_angle.cos(),
        }
    }

    #[inline]
    pub fn from_euler(yaw_y: f32, pitch_x: f32, roll_z: f32) -> Self {
        let cy = (yaw_y * 0.5).cos();
        let sy = (yaw_y * 0.5).sin();
        let cp = (pitch_x * 0.5).cos();
        let sp = (pitch_x * 0.5).sin();
        let cr = (roll_z * 0.5).cos();
        let sr = (roll_z * 0.5).sin();

        Self {
            w: cy * cp * cr + sy * sp * sr,
            x: cy * sp * cr + sy * cp * sr,
            y: sy * cp * cr - cy * sp * sr,
            z: cy * cp * sr - sy * sp * cr,
        }
    }

    #[inline]
    pub fn to_axis_angle(self) -> (Vector3, f32) {
        let q = if self.w > 1.0 {
            self.normalize()
        } else {
            self
        };
        let angle = 2.0 * q.w.acos();
        let s = (1.0 - q.w * q.w).sqrt();
        let axis = if s < 1e-6 {
            Vector3::X
        } else {
            Vector3::new(q.x / s, q.y / s, q.z / s)
        };
        (axis, angle)
    }

    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
    }

    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    #[inline]
    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    #[inline]
    pub fn normalize(self) -> Self {
        let len_sq = self.length_squared();
        if len_sq > 1e-12 {
            let inv_len = 1.0 / len_sq.sqrt();
            Self {
                x: self.x * inv_len,
                y: self.y * inv_len,
                z: self.z * inv_len,
                w: self.w * inv_len,
            }
        } else {
            Self::IDENTITY
        }
    }

    #[inline]
    pub fn conjugate(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    #[inline]
    pub fn inverse(self) -> Self {
        let len_sq = self.length_squared();
        if len_sq > 1e-12 {
            let inv = 1.0 / len_sq;
            Self {
                x: -self.x * inv,
                y: -self.y * inv,
                z: -self.z * inv,
                w: self.w * inv,
            }
        } else {
            Self::IDENTITY
        }
    }

    #[inline]
    pub fn rotate_vector(self, v: Vector3) -> Vector3 {
        let qv = Vector3::new(self.x, self.y, self.z);
        let uv = qv.cross(v);
        let uuv = qv.cross(uv);
        v + ((uv * self.w) + uuv) * 2.0
    }

    #[inline]
    pub fn slerp(self, mut to: Self, t: f32) -> Self {
        let mut cos_theta = self.dot(to);
        if cos_theta < 0.0 {
            to = -to;
            cos_theta = -cos_theta;
        }

        if cos_theta > 0.9995 {
            // Linear interpolation when very close to avoid division by zero
            return (self * (1.0 - t) + to * t).normalize();
        }

        let theta = cos_theta.acos();
        let sin_theta = theta.sin();
        let w1 = ((1.0 - t) * theta).sin() / sin_theta;
        let w2 = (t * theta).sin() / sin_theta;

        self * w1 + to * w2
    }
}

impl Default for Quaternion {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mul for Quaternion {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl Mul<f32> for Quaternion {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
            w: self.w * rhs,
        }
    }
}

impl Add for Quaternion {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
            w: self.w + rhs.w,
        }
    }
}

impl Sub for Quaternion {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
            w: self.w - rhs.w,
        }
    }
}

impl Neg for Quaternion {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: -self.w,
        }
    }
}
