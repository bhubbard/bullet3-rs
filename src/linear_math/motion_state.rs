use super::transform::Transform;

/// Interface for synchronizing physics bodies with graphics engines / renderers.
/// Direct equivalent to Bullet's `btMotionState`.
pub trait MotionState: Send + Sync {
    /// Get the world transform for the rigid body.
    fn get_world_transform(&self) -> Transform;

    /// Called by the physics simulation when the body transforms are updated.
    fn set_world_transform(&mut self, transform: Transform);
}

/// Default motion state storing graphics transform and center-of-mass offset.
/// Direct equivalent to Bullet's `btDefaultMotionState`.
#[derive(Debug, Clone, PartialEq)]
pub struct DefaultMotionState {
    pub graphics_world_transform: Transform,
    pub center_of_mass_offset: Transform,
    pub start_world_transform: Transform,
}

impl DefaultMotionState {
    pub fn new(start_transform: Transform) -> Self {
        Self {
            graphics_world_transform: start_transform,
            center_of_mass_offset: Transform::IDENTITY,
            start_world_transform: start_transform,
        }
    }

    pub fn with_center_of_mass_offset(mut self, offset: Transform) -> Self {
        self.center_of_mass_offset = offset;
        self
    }
}

impl MotionState for DefaultMotionState {
    fn get_world_transform(&self) -> Transform {
        self.graphics_world_transform * self.center_of_mass_offset.inverse()
    }

    fn set_world_transform(&mut self, center_of_mass_world_trans: Transform) {
        self.graphics_world_transform = center_of_mass_world_trans * self.center_of_mass_offset;
    }
}
