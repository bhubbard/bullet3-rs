use crate::dynamics::world::DiscreteDynamicsWorld;
use crate::linear_math::Vector3;

/// Kinematic character controller for player/actor navigation (`btKinematicCharacterController`).
pub struct KinematicCharacterController {
    pub ghost_body_id: usize,
    pub step_height: f32,
    pub max_slope_radians: f32,
    pub gravity: f32,
    pub walk_direction: Vector3,
    pub vertical_velocity: f32,
    pub is_on_ground: bool,
    pub jump_speed: f32,
}

impl KinematicCharacterController {
    pub fn new(ghost_body_id: usize, step_height: f32) -> Self {
        Self {
            ghost_body_id,
            step_height,
            max_slope_radians: 0.785, // 45 degrees
            gravity: 9.81,
            walk_direction: Vector3::ZERO,
            vertical_velocity: 0.0,
            is_on_ground: false,
            jump_speed: 6.0,
        }
    }

    pub fn set_walk_direction(&mut self, walk_dir: Vector3) {
        self.walk_direction = walk_dir;
    }

    pub fn jump(&mut self) {
        if self.is_on_ground {
            self.vertical_velocity = self.jump_speed;
            self.is_on_ground = false;
        }
    }

    /// Step the character controller through the dynamics world.
    pub fn update_action(&mut self, world: &mut DiscreteDynamicsWorld, dt: f32) {
        let current_pos = match world.get_rigid_body(self.ghost_body_id) {
            Some(b) => b.transform.origin,
            None => return,
        };

        // 1. Apply gravity
        if !self.is_on_ground {
            self.vertical_velocity -= self.gravity * dt;
        }

        let half_h = match world.get_rigid_body(self.ghost_body_id) {
            Some(b) => match &b.shape {
                crate::collision::Shape::Capsule(c) => c.half_height + c.radius,
                crate::collision::Shape::Box(bx) => bx.half_extents.y,
                crate::collision::Shape::Sphere(s) => s.radius,
                _ => 1.0,
            },
            None => return,
        };

        let mut displacement = self.walk_direction * dt;
        displacement.y += self.vertical_velocity * dt;

        let target_pos = current_pos + displacement;

        // 2. Ground raycast probe downwards to check if standing on a surface
        let ray_from = current_pos;
        let ray_to = current_pos - Vector3::new(0.0, half_h + self.step_height + 0.1, 0.0);

        if let Some(hit) = world.ray_test_filtered(ray_from, ray_to, Some(self.ghost_body_id)) {
            if hit.hit_normal_world.y > 0.5 {
                self.is_on_ground = true;
                self.vertical_velocity = 0.0;
                let ground_y = hit.hit_point_world.y + half_h;
                let corrected_pos = Vector3::new(target_pos.x, ground_y, target_pos.z);
                if let Some(body) = world.get_rigid_body_mut(self.ghost_body_id) {
                    body.transform.origin = corrected_pos;
                }
                return;
            }
        }

        self.is_on_ground = false;
        if let Some(body) = world.get_rigid_body_mut(self.ghost_body_id) {
            body.transform.origin = target_pos;
        }
    }
}
