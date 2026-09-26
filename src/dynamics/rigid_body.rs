use crate::collision::shapes::Shape;
use crate::linear_math::{Matrix3x3, Quaternion, Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MotionType {
    Static,
    Kinematic,
    Dynamic,
}

/// Construction info for building a `RigidBody`.
/// Equivalent to Bullet's `btRigidBodyConstructionInfo`.
#[derive(Debug, Clone)]
pub struct RigidBodyConstructionInfo {
    pub mass: f32,
    pub shape: Shape,
    pub start_transform: Transform,
    pub local_inertia: Option<Vector3>,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub friction: f32,
    pub rolling_friction: f32,
    pub restitution: f32,
    pub linear_sleeping_threshold: f32,
    pub angular_sleeping_threshold: f32,
    pub additional_damping: bool,
}

impl RigidBodyConstructionInfo {
    pub fn new(mass: f32, shape: Shape, start_transform: Transform) -> Self {
        let local_inertia = if mass > 0.0 {
            Some(shape.calculate_local_inertia(mass))
        } else {
            None
        };

        Self {
            mass,
            shape,
            start_transform,
            local_inertia,
            linear_damping: 0.04,
            angular_damping: 0.1,
            friction: 0.5,
            rolling_friction: 0.0,
            restitution: 0.0,
            linear_sleeping_threshold: 0.2,
            angular_sleeping_threshold: 0.25,
            additional_damping: false,
        }
    }

    pub fn with_restitution(mut self, restitution: f32) -> Self {
        self.restitution = restitution;
        self
    }

    pub fn with_friction(mut self, friction: f32) -> Self {
        self.friction = friction;
        self
    }

    pub fn with_damping(mut self, linear: f32, angular: f32) -> Self {
        self.linear_damping = linear;
        self.angular_damping = angular;
        self
    }
}

/// Rigid body dynamics entity.
/// Direct equivalent to Bullet's `btRigidBody`.
#[derive(Debug, Clone)]
pub struct RigidBody {
    pub id: usize,
    pub motion_type: MotionType,
    pub mass: f32,
    pub inv_mass: f32,
    pub shape: Shape,
    pub transform: Transform,
    pub linear_velocity: Vector3,
    pub angular_velocity: Vector3,
    pub linear_factor: Vector3,
    pub angular_factor: Vector3,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub inv_inertia_local: Matrix3x3,
    pub inv_inertia_world: Matrix3x3,
    pub total_force: Vector3,
    pub total_torque: Vector3,
    pub gravity: Option<Vector3>,
    pub friction: f32,
    pub rolling_friction: f32,
    pub restitution: f32,
    pub is_active: bool,
    pub deactivation_time: f32,
    pub linear_sleeping_threshold: f32,
    pub angular_sleeping_threshold: f32,
}

impl RigidBody {
    pub fn new(info: RigidBodyConstructionInfo) -> Self {
        let (motion_type, inv_mass, inv_inertia_local) = if info.mass > 0.0 {
            let inv_m = 1.0 / info.mass;
            let inertia = info
                .local_inertia
                .unwrap_or_else(|| info.shape.calculate_local_inertia(info.mass));
            let inv_inertia = Matrix3x3::from_diagonal(Vector3::new(
                if inertia.x > 0.0 { 1.0 / inertia.x } else { 0.0 },
                if inertia.y > 0.0 { 1.0 / inertia.y } else { 0.0 },
                if inertia.z > 0.0 { 1.0 / inertia.z } else { 0.0 },
            ));
            (MotionType::Dynamic, inv_m, inv_inertia)
        } else {
            (MotionType::Static, 0.0, Matrix3x3::ZERO)
        };

        let mut body = Self {
            id: 0,
            motion_type,
            mass: info.mass,
            inv_mass,
            shape: info.shape,
            transform: info.start_transform,
            linear_velocity: Vector3::ZERO,
            angular_velocity: Vector3::ZERO,
            linear_factor: Vector3::ONE,
            angular_factor: Vector3::ONE,
            linear_damping: info.linear_damping,
            angular_damping: info.angular_damping,
            inv_inertia_local,
            inv_inertia_world: Matrix3x3::ZERO,
            total_force: Vector3::ZERO,
            total_torque: Vector3::ZERO,
            gravity: None,
            friction: info.friction,
            rolling_friction: info.rolling_friction,
            restitution: info.restitution,
            is_active: motion_type != MotionType::Static,
            deactivation_time: 0.0,
            linear_sleeping_threshold: info.linear_sleeping_threshold,
            angular_sleeping_threshold: info.angular_sleeping_threshold,
        };

        body.update_inertia_tensor();
        body
    }

    pub fn is_static(&self) -> bool {
        self.motion_type == MotionType::Static
    }

    pub fn is_kinematic(&self) -> bool {
        self.motion_type == MotionType::Kinematic
    }

    pub fn is_dynamic(&self) -> bool {
        self.motion_type == MotionType::Dynamic
    }

    pub fn update_inertia_tensor(&mut self) {
        if self.motion_type != MotionType::Dynamic {
            self.inv_inertia_world = Matrix3x3::ZERO;
            return;
        }
        // I_inv_world = R * I_inv_local * R^T
        self.inv_inertia_world =
            self.transform.basis * self.inv_inertia_local * self.transform.basis.transpose();
    }

    pub fn apply_central_force(&mut self, force: Vector3) {
        if self.is_dynamic() {
            self.total_force += force * self.linear_factor;
            self.activate();
        }
    }

    pub fn apply_force(&mut self, force: Vector3, rel_pos: Vector3) {
        if self.is_dynamic() {
            self.apply_central_force(force);
            self.apply_torque(rel_pos.cross(force * self.linear_factor));
        }
    }

    pub fn apply_torque(&mut self, torque: Vector3) {
        if self.is_dynamic() {
            self.total_torque += torque * self.angular_factor;
            self.activate();
        }
    }

    pub fn apply_central_impulse(&mut self, impulse: Vector3) {
        if self.is_dynamic() {
            self.linear_velocity += impulse * (self.inv_mass * self.linear_factor);
            self.activate();
        }
    }

    pub fn apply_impulse(&mut self, impulse: Vector3, rel_pos: Vector3) {
        if self.is_dynamic() {
            self.apply_central_impulse(impulse);
            let torque_impulse = rel_pos.cross(impulse * self.linear_factor);
            self.apply_torque_impulse(torque_impulse);
        }
    }

    pub fn apply_torque_impulse(&mut self, torque_impulse: Vector3) {
        if self.is_dynamic() {
            self.angular_velocity +=
                (self.inv_inertia_world * torque_impulse) * self.angular_factor;
            self.activate();
        }
    }

    pub fn activate(&mut self) {
        if self.is_dynamic() {
            self.is_active = true;
            self.deactivation_time = 0.0;
        }
    }

    pub fn sleep(&mut self) {
        if self.is_dynamic() {
            self.is_active = false;
            self.linear_velocity = Vector3::ZERO;
            self.angular_velocity = Vector3::ZERO;
        }
    }

    pub fn velocity_at_point(&self, rel_pos: Vector3) -> Vector3 {
        self.linear_velocity + self.angular_velocity.cross(rel_pos)
    }

    pub fn clear_forces(&mut self) {
        self.total_force = Vector3::ZERO;
        self.total_torque = Vector3::ZERO;
    }

    /// Symplectic Euler integration of velocities.
    pub fn integrate_velocities(&mut self, dt: f32, world_gravity: Vector3) {
        if !self.is_dynamic() || !self.is_active {
            return;
        }

        let effective_gravity = self.gravity.unwrap_or(world_gravity);
        let linear_acc = (self.total_force * self.inv_mass) + effective_gravity;
        self.linear_velocity += linear_acc * dt * self.linear_factor;

        let angular_acc = self.inv_inertia_world * self.total_torque;
        self.angular_velocity += angular_acc * dt * self.angular_factor;

        // Apply damping
        self.linear_velocity *= (1.0 - self.linear_damping * dt).clamp(0.0, 1.0);
        self.angular_velocity *= (1.0 - self.angular_damping * dt).clamp(0.0, 1.0);
    }

    /// Integrate positions and orientations.
    pub fn integrate_transform(&mut self, dt: f32) {
        if !self.is_dynamic() || !self.is_active {
            return;
        }

        // Translation update
        self.transform.origin += self.linear_velocity * dt;

        // Orientation update via angular velocity quaternion derivative
        let ang_vel_len = self.angular_velocity.length();
        if ang_vel_len > 1e-6 {
            let axis = self.angular_velocity / ang_vel_len;
            let angle = ang_vel_len * dt;
            let dq = Quaternion::from_axis_angle(axis, angle);
            let current_q = self.transform.rotation();
            let new_q = (dq * current_q).normalize();
            self.transform.set_rotation(new_q);
            self.update_inertia_tensor();
        }
    }

    /// Check if body qualifies for sleeping/deactivation.
    pub fn update_deactivation(&mut self, dt: f32) {
        if !self.is_dynamic() || !self.is_active {
            return;
        }

        let lin_speed_sq = self.linear_velocity.length_squared();
        let ang_speed_sq = self.angular_velocity.length_squared();

        let lin_thresh_sq = self.linear_sleeping_threshold * self.linear_sleeping_threshold;
        let ang_thresh_sq = self.angular_sleeping_threshold * self.angular_sleeping_threshold;

        if lin_speed_sq < lin_thresh_sq && ang_speed_sq < ang_thresh_sq {
            self.deactivation_time += dt;
            if self.deactivation_time > 2.0 {
                self.sleep();
            }
        } else {
            self.deactivation_time = 0.0;
        }
    }
}
