//! # Bullet3-RS: Pure Rust Port and Fork of Bullet 3 Physics SDK
//!
//! A high-performance 3D multi-body physics simulation and collision detection engine
//! written in pure, idiomatic Rust.
//!
//! ## Architecture
//!
//! - **`linear_math`**: Vector3, Matrix3x3, Quaternion, Transform, and AABB mathematics (`LinearMath`).
//! - **`collision`**: Broadphase DBVT AABB tree, collision shapes (Sphere, Box, Capsule, Cylinder, StaticPlane, ConvexHull, Compound), SAT & GJK/EPA narrowphase contact generation (`BulletCollision`).
//! - **`dynamics`**: Rigid body simulation, symplectic Euler integration, constraints (Point2Point, Hinge, Distance), Sequential Impulse Constraint Solver, and discrete dynamics world (`BulletDynamics`).
//!
//! ## Quick Start
//!
//! ```rust
//! use bullet3::prelude::*;
//!
//! // 1. Create discrete dynamics world with standard gravity
//! let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));
//!
//! // 2. Add static ground plane at y = 0
//! let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
//! let ground_info = RigidBodyConstructionInfo::new(0.0, ground_shape, Transform::IDENTITY);
//! world.add_rigid_body(RigidBody::new(ground_info));
//!
//! // 3. Add dynamic falling box at y = 10
//! let box_shape = Shape::Box(BoxShape::new(Vector3::splat(1.0)));
//! let start_tf = Transform::from_translation(Vector3::new(0.0, 10.0, 0.0));
//! let box_info = RigidBodyConstructionInfo::new(1.0, box_shape, start_tf)
//!     .with_restitution(0.5);
//! let box_id = world.add_rigid_body(RigidBody::new(box_info));
//!
//! // 4. Step simulation for 60 frames (1 second at 60 Hz)
//! for _ in 0..60 {
//!     world.step_simulation(1.0 / 60.0, 10, 1.0 / 60.0);
//! }
//!
//! let body = world.get_rigid_body(box_id).unwrap();
//! println!("Box settled at height y = {:.3}", body.transform.origin.y);
//! ```

pub mod collision;
pub mod dynamics;
pub mod linear_math;

/// Commonly used types for convenient importing.
pub mod prelude {
    pub use crate::collision::{
        BoxShape, BroadphasePair, CapsuleShape, CollisionFlags, CollisionObject, CollisionWorld,
        CompoundShape, ConvexHullShape, CylinderShape, DbvtBroadphase, RayTestResult, Shape,
        ShapeType, SphereShape, StaticPlaneShape,
    };
    pub use crate::dynamics::{
        Constraint, DiscreteDynamicsWorld, DistanceConstraint, HingeConstraint, MotionType,
        Point2PointConstraint, RigidBody, RigidBodyConstructionInfo,
        SequentialImpulseConstraintSolver,
    };
    pub use crate::linear_math::{Aabb, Matrix3x3, Quaternion, Transform, Vector3};
}

pub use prelude::*;
