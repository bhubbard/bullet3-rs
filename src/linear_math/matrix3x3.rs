use super::quaternion::Quaternion;
use super::vector3::Vector3;
use std::ops::{Add, Mul, Sub};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A 3x3 matrix representing orientations, rotations, and inertia tensors.
/// Direct equivalent to Bullet's `btMatrix3x3`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Matrix3x3 {
    pub m: [[f32; 3]; 3], // Row-major: m[row][col]
}

impl Matrix3x3 {
    pub const ZERO: Self = Self {
        m: [[0.0; 3]; 3],
    };

    pub const IDENTITY: Self = Self {
        m: [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
    };

    #[inline]
    pub const fn from_rows(r0: Vector3, r1: Vector3, r2: Vector3) -> Self {
        Self {
            m: [
                [r0.x, r0.y, r0.z],
                [r1.x, r1.y, r1.z],
                [r2.x, r2.y, r2.z],
            ],
        }
    }

    #[inline]
    pub const fn from_columns(c0: Vector3, c1: Vector3, c2: Vector3) -> Self {
        Self {
            m: [
                [c0.x, c1.x, c2.x],
                [c0.y, c1.y, c2.y],
                [c0.z, c1.z, c2.z],
            ],
        }
    }

    #[inline]
    pub const fn from_diagonal(diag: Vector3) -> Self {
        Self {
            m: [
                [diag.x, 0.0, 0.0],
                [0.0, diag.y, 0.0],
                [0.0, 0.0, diag.z],
            ],
        }
    }

    #[inline]
    pub fn row(&self, i: usize) -> Vector3 {
        Vector3::new(self.m[i][0], self.m[i][1], self.m[i][2])
    }

    #[inline]
    pub fn column(&self, j: usize) -> Vector3 {
        Vector3::new(self.m[0][j], self.m[1][j], self.m[2][j])
    }

    #[inline]
    pub fn transpose(&self) -> Self {
        Self {
            m: [
                [self.m[0][0], self.m[1][0], self.m[2][0]],
                [self.m[0][1], self.m[1][1], self.m[2][1]],
                [self.m[0][2], self.m[1][2], self.m[2][2]],
            ],
        }
    }

    #[inline]
    pub fn determinant(&self) -> f32 {
        self.m[0][0] * (self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1])
            - self.m[0][1] * (self.m[1][0] * self.m[2][2] - self.m[1][2] * self.m[2][0])
            + self.m[0][2] * (self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0])
    }

    #[inline]
    pub fn inverse(&self) -> Self {
        let det = self.determinant();
        if det.abs() < 1e-12 {
            return Self::IDENTITY;
        }
        let inv_det = 1.0 / det;

        Self {
            m: [
                [
                    (self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1]) * inv_det,
                    (self.m[0][2] * self.m[2][1] - self.m[0][1] * self.m[2][2]) * inv_det,
                    (self.m[0][1] * self.m[1][2] - self.m[0][2] * self.m[1][1]) * inv_det,
                ],
                [
                    (self.m[1][2] * self.m[2][0] - self.m[1][0] * self.m[2][2]) * inv_det,
                    (self.m[0][0] * self.m[2][2] - self.m[0][2] * self.m[2][0]) * inv_det,
                    (self.m[0][2] * self.m[1][0] - self.m[0][0] * self.m[1][2]) * inv_det,
                ],
                [
                    (self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0]) * inv_det,
                    (self.m[0][1] * self.m[2][0] - self.m[0][0] * self.m[2][1]) * inv_det,
                    (self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0]) * inv_det,
                ],
            ],
        }
    }

    #[inline]
    pub fn from_quaternion(q: Quaternion) -> Self {
        let d = q.length_squared();
        if d < 1e-12 {
            return Self::IDENTITY;
        }
        let s = 2.0 / d;
        let xs = q.x * s;
        let ys = q.y * s;
        let zs = q.z * s;
        let wx = q.w * xs;
        let wy = q.w * ys;
        let wz = q.w * zs;
        let xx = q.x * xs;
        let xy = q.x * ys;
        let xz = q.x * zs;
        let yy = q.y * ys;
        let yz = q.y * zs;
        let zz = q.z * zs;

        Self {
            m: [
                [1.0 - (yy + zz), xy - wz, xz + wy],
                [xy + wz, 1.0 - (xx + zz), yz - wx],
                [xz - wy, yz + wx, 1.0 - (xx + yy)],
            ],
        }
    }

    #[inline]
    pub fn to_quaternion(&self) -> Quaternion {
        let trace = self.m[0][0] + self.m[1][1] + self.m[2][2];
        if trace > 0.0 {
            let mut s = (trace + 1.0).sqrt();
            let w = s * 0.5;
            s = 0.5 / s;
            Quaternion::new(
                (self.m[2][1] - self.m[1][2]) * s,
                (self.m[0][2] - self.m[2][0]) * s,
                (self.m[1][0] - self.m[0][1]) * s,
                w,
            )
        } else {
            let i = if self.m[1][1] > self.m[0][0] {
                if self.m[2][2] > self.m[1][1] {
                    2
                } else {
                    1
                }
            } else if self.m[2][2] > self.m[0][0] {
                2
            } else {
                0
            };
            let j = (i + 1) % 3;
            let k = (i + 2) % 3;
            let mut s = (self.m[i][i] - (self.m[j][j] + self.m[k][k]) + 1.0).sqrt();
            let mut q = [0.0f32; 4];
            q[i] = s * 0.5;
            s = 0.5 / s;
            q[3] = (self.m[k][j] - self.m[j][k]) * s;
            q[j] = (self.m[j][i] + self.m[i][j]) * s;
            q[k] = (self.m[k][i] + self.m[i][k]) * s;
            Quaternion::new(q[0], q[1], q[2], q[3])
        }
    }

    #[inline]
    pub fn from_axis_angle(axis: Vector3, angle_radians: f32) -> Self {
        let q = Quaternion::from_axis_angle(axis, angle_radians);
        Self::from_quaternion(q)
    }

    /// Skew symmetric cross product matrix [v]_x such that [v]_x * w = v.cross(w)
    #[inline]
    pub fn skew_symmetric(v: Vector3) -> Self {
        Self {
            m: [
                [0.0, -v.z, v.y],
                [v.z, 0.0, -v.x],
                [-v.y, v.x, 0.0],
            ],
        }
    }

    #[inline]
    pub fn times_transpose(&self, other: &Self) -> Self {
        *self * other.transpose()
    }

    #[inline]
    pub fn transpose_times(&self, other: &Self) -> Self {
        self.transpose() * *other
    }

    #[inline]
    pub fn mul_transpose_vector(&self, v: Vector3) -> Vector3 {
        self.transpose() * v
    }

    #[inline]
    pub fn scaled(&self, scale: Vector3) -> Self {
        Self {
            m: [
                [self.m[0][0] * scale.x, self.m[0][1] * scale.y, self.m[0][2] * scale.z],
                [self.m[1][0] * scale.x, self.m[1][1] * scale.y, self.m[1][2] * scale.z],
                [self.m[2][0] * scale.x, self.m[2][1] * scale.y, self.m[2][2] * scale.z],
            ],
        }
    }
}

impl Default for Matrix3x3 {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mul<Vector3> for Matrix3x3 {
    type Output = Vector3;
    #[inline]
    fn mul(self, v: Vector3) -> Vector3 {
        Vector3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }
}

impl Mul<Matrix3x3> for Matrix3x3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        let mut res = Self::ZERO;
        for i in 0..3 {
            for j in 0..3 {
                res.m[i][j] = self.m[i][0] * rhs.m[0][j]
                    + self.m[i][1] * rhs.m[1][j]
                    + self.m[i][2] * rhs.m[2][j];
            }
        }
        res
    }
}

impl Mul<f32> for Matrix3x3 {
    type Output = Self;
    #[inline]
    fn mul(self, scalar: f32) -> Self {
        Self {
            m: [
                [self.m[0][0] * scalar, self.m[0][1] * scalar, self.m[0][2] * scalar],
                [self.m[1][0] * scalar, self.m[1][1] * scalar, self.m[1][2] * scalar],
                [self.m[2][0] * scalar, self.m[2][1] * scalar, self.m[2][2] * scalar],
            ],
        }
    }
}

impl Mul<Matrix3x3> for f32 {
    type Output = Matrix3x3;
    #[inline]
    fn mul(self, mat: Matrix3x3) -> Matrix3x3 {
        mat * self
    }
}

impl Add for Matrix3x3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        let mut m = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] = self.m[i][j] + rhs.m[i][j];
            }
        }
        Self { m }
    }
}

impl Sub for Matrix3x3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        let mut m = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] = self.m[i][j] - rhs.m[i][j];
            }
        }
        Self { m }
    }
}
