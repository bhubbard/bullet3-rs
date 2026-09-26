use crate::dynamics::world::DiscreteDynamicsWorld;
use crate::linear_math::{Quaternion, Transform, Vector3};

/// Configuration and state for a single vehicle wheel (`btWheelInfo`).
#[derive(Debug, Clone)]
pub struct WheelInfo {
    pub chassis_connection_point: Vector3,
    pub wheel_direction: Vector3,
    pub wheel_axle: Vector3,
    pub suspension_rest_length: f32,
    pub max_suspension_travel: f32,
    pub wheel_radius: f32,
    pub suspension_stiffness: f32,
    pub damping_compression: f32,
    pub damping_relaxation: f32,
    pub friction_slip: f32,
    pub steering: f32,
    pub engine_force: f32,
    pub brake: f32,

    // Runtime state
    pub raycast_hit: bool,
    pub raycast_hit_point: Vector3,
    pub raycast_hit_normal: Vector3,
    pub raycast_distance: f32,
    pub suspension_length: f32,
    pub rotation: f32,
    pub delta_rotation: f32,
    pub world_transform: Transform,
}

impl WheelInfo {
    pub fn new(
        chassis_connection_point: Vector3,
        wheel_direction: Vector3,
        wheel_axle: Vector3,
        suspension_rest_length: f32,
        wheel_radius: f32,
    ) -> Self {
        Self {
            chassis_connection_point,
            wheel_direction: wheel_direction.normalize(),
            wheel_axle: wheel_axle.normalize(),
            suspension_rest_length,
            max_suspension_travel: suspension_rest_length * 0.5,
            wheel_radius,
            suspension_stiffness: 20.0,
            damping_compression: 2.3,
            damping_relaxation: 4.4,
            friction_slip: 1.2,
            steering: 0.0,
            engine_force: 0.0,
            brake: 0.0,
            raycast_hit: false,
            raycast_hit_point: Vector3::ZERO,
            raycast_hit_normal: Vector3::Y,
            raycast_distance: suspension_rest_length,
            suspension_length: suspension_rest_length,
            rotation: 0.0,
            delta_rotation: 0.0,
            world_transform: Transform::IDENTITY,
        }
    }
}

/// Raycast vehicle dynamics model.
/// Direct equivalent to Bullet's famous `btRaycastVehicle`.
pub struct RaycastVehicle {
    pub chassis_body_id: usize,
    pub wheels: Vec<WheelInfo>,
}

impl RaycastVehicle {
    pub fn new(chassis_body_id: usize) -> Self {
        Self {
            chassis_body_id,
            wheels: Vec::with_capacity(4),
        }
    }

    pub fn add_wheel(
        &mut self,
        connection_point: Vector3,
        wheel_direction: Vector3,
        wheel_axle: Vector3,
        suspension_rest_length: f32,
        wheel_radius: f32,
    ) -> usize {
        let idx = self.wheels.len();
        self.wheels.push(WheelInfo::new(
            connection_point,
            wheel_direction,
            wheel_axle,
            suspension_rest_length,
            wheel_radius,
        ));
        idx
    }

    pub fn set_steering_value(&mut self, wheel_idx: usize, steering: f32) {
        if let Some(wheel) = self.wheels.get_mut(wheel_idx) {
            wheel.steering = steering;
        }
    }

    pub fn apply_engine_force(&mut self, wheel_idx: usize, force: f32) {
        if let Some(wheel) = self.wheels.get_mut(wheel_idx) {
            wheel.engine_force = force;
        }
    }

    pub fn set_brake(&mut self, wheel_idx: usize, brake: f32) {
        if let Some(wheel) = self.wheels.get_mut(wheel_idx) {
            wheel.brake = brake;
        }
    }

    /// Update vehicle suspension, wheel ground raycasts, and forces.
    pub fn update_vehicle(&mut self, world: &mut DiscreteDynamicsWorld, _step: f32) {
        let chassis_tf = match world.get_rigid_body(self.chassis_body_id) {
            Some(b) => b.transform,
            None => return,
        };

        for wheel in &mut self.wheels {
            // Ray from wheel connection point downwards along suspension
            let source = chassis_tf.transform_point(wheel.chassis_connection_point);
            let down = chassis_tf.transform_vector(wheel.wheel_direction);
            let ray_len = wheel.suspension_rest_length + wheel.wheel_radius;
            let target = source + down * ray_len;

            if let Some(hit) = world.ray_test_filtered(source, target, Some(self.chassis_body_id)) {
                wheel.raycast_hit = true;
                wheel.raycast_hit_point = hit.hit_point_world;
                wheel.raycast_hit_normal = hit.hit_normal_world;
                let dist = source.distance(hit.hit_point_world);
                wheel.raycast_distance = dist;
                wheel.suspension_length = (dist - wheel.wheel_radius).max(0.0);
            } else {
                wheel.raycast_hit = false;
                wheel.suspension_length = wheel.suspension_rest_length;
            }

            // Calculate wheel world transform
            let wheel_pos = source + down * wheel.suspension_length;
            let steer_q = Quaternion::from_axis_angle(Vector3::Y, wheel.steering);
            let wheel_tf = Transform::from_rotation_quaternion(wheel_pos, chassis_tf.rotation() * steer_q);
            wheel.world_transform = wheel_tf;
        }

        // Apply suspension & traction impulses to chassis body
        let (chassis_pos, chassis_lin_vel, chassis_ang_vel) = match world.get_rigid_body(self.chassis_body_id) {
            Some(b) => (b.transform.origin, b.linear_velocity, b.angular_velocity),
            None => return,
        };

        for wheel in &self.wheels {
            if !wheel.raycast_hit {
                continue;
            }

            let rel_pos = wheel.raycast_hit_point - chassis_pos;
            let wheel_vel = chassis_lin_vel + chassis_ang_vel.cross(rel_pos);

            // 1. Suspension force: Spring + Damper along contact normal
            let compression = (wheel.suspension_rest_length - wheel.suspension_length).max(0.0);
            let spring_force = compression * wheel.suspension_stiffness;
            let vel_along_norm = wheel_vel.dot(wheel.raycast_hit_normal);
            let damper_force = -vel_along_norm * wheel.damping_compression;
            let total_suspension_force = (spring_force + damper_force).max(0.0);

            let normal_impulse = wheel.raycast_hit_normal * (total_suspension_force * 0.016);

            // 2. Drive & braking force along wheel forward tangent
            let forward = wheel.world_transform.transform_vector(Vector3::Z).normalize();
            let drive_force = wheel.engine_force - (wheel.brake * wheel_vel.dot(forward).signum() * 10.0);
            let drive_impulse = forward * (drive_force * 0.016);

            // 3. Lateral tire friction to resist sideways sliding
            let side = wheel.world_transform.transform_vector(Vector3::X).normalize();
            let side_vel = wheel_vel.dot(side);
            let lateral_friction = -side_vel * wheel.friction_slip * (total_suspension_force * 0.016);
            let side_impulse = side * lateral_friction;

            if let Some(chassis) = world.get_rigid_body_mut(self.chassis_body_id) {
                chassis.apply_impulse(normal_impulse + drive_impulse + side_impulse, rel_pos);
            }
        }
    }
}
